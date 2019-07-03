import os
import time
import pickle
import random
import argparse
import Queue

import cProfile
import pstats
from problem_environments.mover_env import Mover
from problem_environments.one_arm_mover_env import OneArmMover
from generators.PickUniform import PickWithBaseUnif
from generators.PlaceUniform import PlaceUnif
# from operator_utils.grasp_utils import solveTwoArmIKs, compute_two_arm_grasp

from trajectory_representation.operator import Operator
from trajectory_representation.shortest_path_pick_and_place_state import ShortestPathPaPState
from trajectory_representation.one_arm_pap_state import OneArmPaPState
from trajectory_representation.trajectory import Trajectory

from mover_library.utils import set_robot_config, set_obj_xytheta, visualize_path, two_arm_pick_object, \
    two_arm_place_object, get_body_xytheta, grab_obj, release_obj, fold_arms, one_arm_pick_object, one_arm_place_object

import numpy as np
import tensorflow as tf
import openravepy

from generators.uniform import UniformGenerator, PaPUniformGenerator

from mover_library import utils
from manipulation.primitives.display import set_viewer_options, draw_line, draw_point
from manipulation.primitives.savers import DynamicEnvironmentStateSaver

from learn.data_traj import extract_individual_example
from learn.pap_gnn import PaPGNN
import collections

prm_vertices, prm_edges = pickle.load(open('prm.pkl', 'rb'))
# prm_edges = [set(l) - {i} for i,l in enumerate(prm_edges)]
prm_vertices = list(prm_vertices)  # TODO: needs to be a list rather than ndarray

connected = np.array([len(s) >= 2 for s in prm_edges])
prm_indices = {tuple(v): i for i, v in enumerate(prm_vertices)}
DISABLE_COLLISIONS = False
MAX_DISTANCE = 1.0


def get_actions(mover, goal, config):
    actions = []
    for o in mover.entity_names:
        if 'region' in o:
            continue
        for r in mover.entity_names:
            if 'region' not in r or 'entire' in r:
                continue
            if o not in goal and r in goal:
                # you cannot place non-goal object in the goal region
                continue
            if config.domain == 'two_arm_mover':
                action = Operator('two_arm_pick_two_arm_place', {'two_arm_place_object': o, 'two_arm_place_region': r})
                # following two lines are for legacy reasons, will fix later
                action.discrete_parameters['object'] = action.discrete_parameters['two_arm_place_object']
                action.discrete_parameters['region'] = action.discrete_parameters['two_arm_place_region']
            elif config.domain == 'one_arm_mover':
                action = Operator('one_arm_pick_one_arm_place',
                                  {'object': mover.env.GetKinBody(o), 'region': mover.regions[r]})

            else:
                raise NotImplementedError
            actions.append(action)

    return actions


def compute_heuristic(state, action):
    o = action.discrete_parameters['two_arm_place_object']
    r = action.discrete_parameters['two_arm_place_region']
    nodes, edges, actions, _ = extract_individual_example(state, action)
    nodes = nodes[..., 6:]

    object_is_goal = state.nodes[o][8]
    region_is_goal = state.nodes[r][8]
    number_in_goal = sum(state.binary_edges[(i, r)][0] for i in state.nodes for r in mover.regions if
                         i != o and state.nodes[r][8]) + int(region_is_goal)
    # this translates to: number of objects that would be in goal if the action was executed
    # This would be the dual of number of remaining objects after the action was executed, which would be
    # the more traditional heuristic function. The negative sign helps with that.
    redundant = state.binary_edges[(o, r)][0]
    helps_goal = object_is_goal and region_is_goal and not redundant

    redundant = 0
    unhelpful = 0
    # number_in_goal = 0
    helps_goal = 0

    if config.dont_use_gnn:
        return 1 * redundant - number_in_goal - 2 * helps_goal + 2 * unhelpful
    elif config.dont_use_h:
        gnn_pred = -pap_model.predict_with_raw_input_format(nodes[None, ...], edges[None, ...],
                                                            actions[None, ...])
        return gnn_pred
    else:
        gnn_pred = -pap_model.predict_with_raw_input_format(nodes[None, ...], edges[None, ...],
                                                            actions[None, ...])
        return 1 * redundant - number_in_goal - 2 * helps_goal + 2 * unhelpful + gnn_pred


def get_problem(mover):
    tt = time.time()

    obj_names = [obj.GetName() for obj in mover.objects]
    n_objs_pack = config.n_objs_pack

    if config.domain == 'two_arm_mover':
        statecls = ShortestPathPaPState
        goal = ['home_region'] + [obj.GetName() for obj in mover.objects[:n_objs_pack]]
    elif config.domain == 'one_arm_mover':
        statecls = OneArmPaPState
        goal = ['rectangular_packing_box1_region'] + [obj.GetName() for obj in mover.objects[:n_objs_pack]]
    else:
        raise NotImplementedError

    pr = cProfile.Profile()
    pr.enable()
    state = statecls(mover, goal)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(30)
    pstats.Stats(pr).sort_stats('cumtime').print_stats(30)

    state.make_pklable()

    mconfig_type = collections.namedtuple('mconfig_type',
                                          'operator n_msg_passing n_layers num_fc_layers n_hidden no_goal_nodes top_k optimizer lr use_mse batch_size seed num_train val_portion num_test mse_weight diff_weight_msg_passing same_vertex_model weight_initializer loss')

    assert config.num_train <= 5000
    pap_mconfig = mconfig_type(
        operator='two_arm_pick_two_arm_place',
        n_msg_passing=1,
        n_layers=2,
        num_fc_layers=2,
        n_hidden=32,
        no_goal_nodes=False,

        top_k=1,
        optimizer='adam',
        lr=1e-4,
        use_mse=True,

        batch_size='32',
        seed=config.train_seed,
        num_train=config.num_train,
        val_portion=.1,
        num_test=1882,
        mse_weight=1.0,
        diff_weight_msg_passing=False,
        same_vertex_model=False,
        weight_initializer='glorot_uniform',
        loss=config.loss,
    )
    if config.domain == 'two_arm_mover':
        num_entities = 11
        n_regions = 2
    elif config.domain == 'one_arm_mover':
        num_entities = 17
        n_regions = 3
    else:
        raise NotImplementedError
    num_node_features = 10
    num_edge_features = 44
    entity_names = mover.entity_names

    with tf.variable_scope('pap'):
        pap_model = PaPGNN(num_entities, num_node_features, num_edge_features, pap_mconfig, entity_names, n_regions)
    pap_model.load_weights()

    def heuristic(state, action):
        if action.type == 'two_arm_pick_two_arm_place':
            o = action.discrete_parameters['two_arm_place_object']
            r = action.discrete_parameters['two_arm_place_region']
            nodes, edges, actions, _ = extract_individual_example(state, action)
            nodes = nodes[..., 6:]

            object_is_goal = state.nodes[o][8]
            region_is_goal = state.nodes[r][8]
            number_in_goal = sum(state.binary_edges[(i, r)][0] for i in state.nodes for r in mover.regions if
                                 i != o and state.nodes[r][8]) + int(region_is_goal)
            # this translates to: number of objects that would be in goal if the action was executed
            # This would be the dual of number of remaining objects after the action was executed, which would be
            # the more traditional heuristic function. The negative sign helps with that.
            redundant = state.binary_edges[(o, r)][0]
            helps_goal = object_is_goal and region_is_goal and not redundant

            redundant = 0
            unhelpful = 0
            # number_in_goal = 0
            helps_goal = 0

            if config.dont_use_gnn:
                return 1 * redundant - number_in_goal - 2 * helps_goal + 2 * unhelpful
            elif config.dont_use_h:
                gnn_pred = -pap_model.predict_with_raw_input_format(nodes[None, ...], edges[None, ...],
                                                                    actions[None, ...])
                return gnn_pred
            else:
                gnn_pred = -pap_model.predict_with_raw_input_format(nodes[None, ...], edges[None, ...],
                                                                    actions[None, ...])
                return 1 * redundant - number_in_goal - 2 * helps_goal + 2 * unhelpful + gnn_pred

        elif action.type == 'one_arm_pick_one_arm_place':
            o = action.discrete_parameters['object'].GetName()
            r = action.discrete_parameters['region'].name
            nodes, edges, actions, _ = extract_individual_example(state, action)
            nodes = nodes[..., 6:]

            object_is_goal = state.nodes[o][8]
            region_is_goal = state.nodes[r][8]
            number_in_goal = sum(state.binary_edges[(i, r)][0] for i in state.nodes for r in mover.regions if
                                 i != o and r in state.nodes and state.nodes[r][8]) + int(region_is_goal)
            redundant = state.binary_edges[(o, r)][0]
            helps_goal = object_is_goal and region_is_goal and not redundant
            unhelpful = object_is_goal and not region_is_goal

            if config.dont_use_gnn:
                return 1 * redundant - number_in_goal - 2 * helps_goal + 2 * unhelpful
            elif config.dont_use_h:
                gnn_pred = -pap_model.predict_with_raw_input_format(nodes[None, ...], edges[None, ...],
                                                                    actions[None, ...])
                return gnn_pred
            else:
                gnn_pred = -pap_model.predict_with_raw_input_format(nodes[None, ...], edges[None, ...],
                                                                    actions[None, ...])
                return 1 * redundant - number_in_goal - 2 * helps_goal + 2 * unhelpful + gnn_pred
        else:
            raise NotImplementedError

    mover.reset_to_init_state_stripstream()
    if config.visualize_plan:
        mover.env.SetViewer('qtcoin')
        set_viewer_options(mover.env)

    depth_limit = 60

    class Node(object):
        def __init__(self, parent, action, state, reward=0):
            self.parent = parent  # parent.state is initial state
            self.action = action
            self.state = state  # resulting state
            self.reward = reward  # resulting reward

            if parent is None:
                self.depth = 1
            else:
                self.depth = parent.depth + 1

        def backtrack(self):
            node = self
            while node is not None:
                yield node
                node = node.parent

    action_queue = Queue.PriorityQueue()  # (heuristic, nan, operator skeleton, state. trajectory)
    initnode = Node(None, None, state)
    initial_state = state
    actions = get_actions(mover, goal, config)
    for a in actions:
        action_queue.put((heuristic(state, a), float('nan'), a, initnode))  # initial q

    iter = 0
    # beginning of the planner
    while True:
        if time.time() - tt > config.timelimit:
            return None, iter

        iter += 1

        if iter > 3000:
            print('failed to find plan: iteration limit')
            return None, iter

        if action_queue.empty():
            actions = get_actions(mover, goal, config)
            for a in actions:
                action_queue.put((heuristic(initial_state, a), float('nan'), a, initnode))  # initial q

        curr_hval, _, action, node = action_queue.get()
        state = node.state

        print('\n'.join([str(parent.action.discrete_parameters.values()) for parent in list(node.backtrack())[-2::-1]]))
        print("{}".format(action.discrete_parameters.values()))

        if node.depth >= 2 and action.type == 'two_arm_pick' and node.parent.action.discrete_parameters['object'] == \
                action.discrete_parameters['object']:  # and plan[-1][1] == r:
            print('skipping because repeat', action.discrete_parameters['object'])
            continue

        if node.depth > depth_limit:
            print('skipping because depth limit', node.action.discrete_parameters.values())

        # reset to state
        state.restore(mover)
        utils.set_color(action.discrete_parameters['object'], [1, 0, 0])  # visualization purpose

        if action.type == 'two_arm_pick_two_arm_place':
            smpler = PaPUniformGenerator(action, mover, None)
            smpled_param = smpler.sample_next_point(action, n_iter=200, n_parameters_to_try_motion_planning=3,
                                                    cached_collisions=state.collides, cached_holding_collisions=None)
            if smpled_param['is_feasible']:
                action.continuous_parameters = smpled_param
                action.execute()
                print "Action executed"
            else:
                print "Failed to sample an action"
                utils.set_color(action.discrete_parameters['object'], [0, 1, 0])  # visualization purpose
                continue

            is_goal_achieved = \
                np.all([mover.regions['home_region'].contains(mover.env.GetKinBody(o).ComputeAABB()) for o in
                        obj_names[:n_objs_pack]])
            if is_goal_achieved:
                print("found successful plan: {}".format(n_objs_pack))
                trajectory = Trajectory(mover.seed, mover.seed)
                plan = list(newnode.backtrack())[::-1]
                trajectory.states = [nd.state for nd in plan]
                trajectory.actions = [nd.action for nd in plan[1:]]
                trajectory.rewards = [nd.reward for nd in plan[1:]]
                trajectory.state_prime = [nd.state for nd in plan[1:]]
                trajectory.seed = mover.seed
                print(trajectory)
                return trajectory, iter
            else:
                newstate = statecls(mover, goal, node.state, action)
                print "New state computed"
                newstate.make_pklable()
                newnode = Node(node, action, newstate)
                newactions = get_actions(mover, goal, config)
                print "Old h value", curr_hval
                for newaction in newactions:
                    hval = heuristic(newstate, newaction) - 1. * newnode.depth
                    print "New state h value %.4f for %s %s" % (
                    hval, newaction.discrete_parameters['object'], newaction.discrete_parameters['region'])
                    action_queue.put(
                        (hval, float('nan'), newaction, newnode))
            utils.set_color(action.discrete_parameters['object'], [0, 1, 0])  # visualization purpose

        elif action.type == 'one_arm_pick_one_arm_place':
            obj = action.discrete_parameters['object']
            region = action.discrete_parameters['region']
            o = obj.GetName()
            r = region.name

            if (o, r) in state.nocollision_place_op:
                pick_op, place_op = node.state.nocollision_place_op[(o, r)]
                action = Operator(
                    operator_type='one_arm_pick_one_arm_place',
                    discrete_parameters={
                        'object': obj,
                        'region': mover.regions[r],
                    },
                    continuous_parameters={
                        'pick': pick_op.continuous_parameters,
                        'place': place_op.continuous_parameters,
                    }
                )
                action.execute()

                success = True

                newstate = statecls(mover, goal, node.state, action)
                newstate.make_pklable()
                newnode = Node(node, action, newstate)



                if all(
                        mover.regions['rectangular_packing_box1_region'].contains(mover.env.GetKinBody(o).ComputeAABB())
                        for o in obj_names[:n_objs_pack]
                ):
                    print("found successful plan: {}".format(n_objs_pack))
                    trajectory = Trajectory(mover.seed, mover.seed)
                    plan = list(newnode.backtrack())[::-1]
                    trajectory.states = [nd.state for nd in plan]
                    for s in trajectory.states:
                        s.pap_params = None
                        s.pick_params = None
                        s.place_params = None
                        s.nocollision_pick_op = None
                        s.collision_pick_op = None
                        s.nocollision_place_op = None
                        s.collision_place_op = None
                    trajectory.actions = [nd.action for nd in plan[1:]]
                    for op in trajectory.actions:
                        op.discrete_parameters = {
                            key: value.name if 'region' in key else value.GetName()
                            for key, value in op.discrete_parameters.items()
                        }
                    trajectory.rewards = [nd.reward for nd in plan[1:]]
                    trajectory.state_prime = [nd.state for nd in plan[1:]]
                    trajectory.seed = mover.seed
                    print(trajectory)
                    return trajectory, iter

                for o in mover.entity_names:
                    if 'region' in o:
                        continue
                    if o == action.discrete_parameters['object'].GetName():
                        continue
                    for r in mover.entity_names:
                        if 'region' not in r:
                            continue

                        newaction = Operator('one_arm_pick_one_arm_place',
                                             {'object': mover.env.GetKinBody(o), 'region': mover.regions[r]})
                        action_queue.put(
                            (heuristic(newstate, newaction) - 1. * newnode.depth, float('nan'), newaction, newnode))

            if not success:
                print('failed to execute action')
            else:
                print('action successful')

        else:
            raise NotImplementedError


##################################################


##################################################

from planners.subplanners.motion_planner import BaseMotionPlanner
import socket


def generate_training_data_single():
    np.random.seed(config.pidx)
    random.seed(config.pidx)
    if config.domain == 'two_arm_mover':
        mover = Mover(config.pidx)
    elif config.domain == 'one_arm_mover':
        mover = OneArmMover(config.pidx)
    else:
        raise NotImplementedError
    np.random.seed(config.planner_seed)
    random.seed(config.planner_seed)
    mover.set_motion_planner(BaseMotionPlanner(mover, 'prm'))
    mover.seed = config.pidx

    # todo change the root node
    mover.init_saver = DynamicEnvironmentStateSaver(mover.env)

    hostname = socket.gethostname()
    if hostname in {'dell-XPS-15-9560', 'phaedra', 'shakey', 'lab', 'glaucus', 'luke-laptop-1'}:
        root_dir = './'
    else:
        root_dir = '/data/public/rw/pass.port/tamp_q_results/'

    if config.dont_use_gnn:
        solution_file_dir = root_dir + '/test_results/greedy_results_on_mover_domain/' \
                            + '/domain_' + config.domain \
                            + '/n_objs_pack_' + str(config.n_objs_pack) \
                            + '/test_purpose/no_gnn/no_goal_obj_same_region/num_goals/'
    elif config.dont_use_h:
        solution_file_dir = root_dir + '/test_results/greedy_results_on_mover_domain/' \
                            + '/domain_' + config.domain \
                            + '/n_objs_pack_' + str(config.n_objs_pack) \
                            + '/test_purpose/no_h/' \
                            + '/num_train_' + str(config.num_train) + '/'
    else:
        """
        redundant = state.binary_edges[(o, r)][0]
        helps_goal = object_is_goal and region_is_goal and not redundant
        unhelpful = object_is_goal and not region_is_goal
        """
        solution_file_dir = root_dir + '/test_results/greedy_results_on_mover_domain/' \
                            + '/domain_' + config.domain \
                            + '/n_objs_pack_' + str(config.n_objs_pack) \
                            + '/test_purpose/no_goal_obj_same_region/num_goals/' \
                            + '/num_train_' + str(config.num_train) + '/'

    solution_file_name = 'pidx_' + str(config.pidx) + \
                         '_planner_seed_' + str(config.planner_seed) + \
                         '_train_seed_' + str(config.train_seed) + \
                         '_domain_' + str(config.domain) + '.pkl'
    if not os.path.isdir(solution_file_dir):
        os.makedirs(solution_file_dir)

    solution_file_name = solution_file_dir + solution_file_name

    is_problem_solved_before = os.path.isfile(solution_file_name)
    if is_problem_solved_before and not config.plan:
        with open(solution_file_name, 'rb') as f:
            trajectory = pickle.load(f)
            success = trajectory.metrics['success']
            tottime = trajectory.metrics['tottime']
    else:
        t = time.time()
        trajectory, num_nodes = get_problem(mover)
        tottime = time.time() - t
        success = trajectory is not None
        plan_length = len(trajectory.actions) if success else 0
        if not success:
            trajectory = Trajectory(mover.seed, mover.seed)
        trajectory.states = None
        trajectory.metrics = {
            'n_objs_pack': config.n_objs_pack,
            'tottime': tottime,
            'success': success,
            'plan_length': plan_length,
            'num_nodes': num_nodes,
        }

        with open(solution_file_name, 'wb') as f:
            pickle.dump(trajectory, f)
    print 'Time: %.2f Success: %d Plan length: %d Num nodes: %d' % (tottime, success, trajectory.metrics['plan_length'],
                                                                    trajectory.metrics['num_nodes'])

    """
    print("time: {}".format(','.join(str(trajectory.metrics[m]) for m in [
        'n_objs_pack',
        'tottime',
        'success',
        'plan_length',
        'num_nodes',
    ])))
    """

    print('\n'.join(str(a.discrete_parameters.values()) for a in trajectory.actions))

    return


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Greedy planner')
    parser.add_argument('-pidx', type=int, default=0)
    parser.add_argument('-train_seed', type=int, default=0)
    parser.add_argument('-planner_seed', type=int, default=0)
    parser.add_argument('-n_objs_pack', type=int, default=1)
    parser.add_argument('-num_train', type=int, default=5000)
    parser.add_argument('-timelimit', type=float, default=600)
    parser.add_argument('-visualize_plan', action='store_true', default=False)
    parser.add_argument('-visualize_sim', action='store_true', default=False)
    parser.add_argument('-dontsimulate', action='store_true', default=False)
    parser.add_argument('-plan', action='store_true', default=False)
    parser.add_argument('-dont_use_gnn', action='store_true', default=False)
    parser.add_argument('-dont_use_h', action='store_true', default=False)
    parser.add_argument('-loss', type=str, default='largemargin')
    parser.add_argument('-domain', type=str, default='two_arm_mover')

    config = parser.parse_args()
    generate_training_data_single()
