import os
import time
import pickle
import random
import argparse
import Queue

import cProfile
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


def find_path(graph, start, goal, heuristic=lambda x: 0, collision=lambda x: False):
    visited = {s for s in start}
    queue = Queue.PriorityQueue()
    for s in start:
        if not collision(s):
            queue.put((heuristic(s), 0, np.random.rand(), s, [s]))
    while not queue.empty():
        _, dist, _, vertex, path = queue.get()

        for next in graph[vertex] - visited:
            visited.add(next)

            if collision(next):
                continue

            if goal(next):
                return path + [next]
            else:
                newdist = dist + np.linalg.norm(prm_vertices[vertex] - prm_vertices[next])
                queue.put((newdist + heuristic(next), newdist, np.random.rand(), next, path + [next]))
    return None


def get_problem(mover):
    tt = time.time()

    obj_names = [obj.GetName() for obj in mover.objects]
    obj_poses = [get_body_xytheta(mover.env.GetKinBody(obj_name)).squeeze() for obj_name in obj_names]

    initial_robot_conf = get_body_xytheta(mover.robot).squeeze()

    def dfs(start, goal, collide):
        path = find_path(prm_edges, list(start), goal, collision=collide)
        if path is None:
            return path
        return path  # [-1]

    def inregion(i, r):
        q = prm_vertices[i]
        if r == 'home_region':
            return q[1] > -3
        elif r == 'loading_region':
            return q[1] < -5
        else:
            assert False

    n_objs_pack = config.n_objs_pack

    if config.domain == 'two_arm_mover':
        statecls = ShortestPathPaPState
        goal = ['home_region'] + [obj.GetName() for obj in mover.objects[:n_objs_pack]]
    elif config.domain == 'one_arm_mover':
        statecls = OneArmPaPState
        goal = ['rectangular_packing_box1_region'] + [obj.GetName() for obj in mover.objects[:n_objs_pack]]
    else:
        raise NotImplementedError

    """
    pr = cProfile.Profile()
    pr.enable()
    """
    state = statecls(mover, goal)
    """
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(30)
    pstats.Stats(pr).sort_stats('cumtime').print_stats(30)
    """

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
    elif config.domain == 'one_arm_mover':
        num_entities = 16
    else:
        raise NotImplementedError
    num_node_features = 10
    num_edge_features = 44
    entity_names = mover.entity_names

    with tf.variable_scope('pap'):
        pap_model = PaPGNN(num_entities, num_node_features, num_edge_features, pap_mconfig, entity_names)
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
            #number_in_goal = 0
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

    ps = PickWithBaseUnif(mover)
    pls = PlaceUnif(mover)

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
            elif config.domain == 'one_arm_mover':
                action = Operator('one_arm_pick_one_arm_place',
                                  {'object': mover.env.GetKinBody(o), 'region': mover.regions[r]})
            else:
                raise NotImplementedError

            action_queue.put((heuristic(state, action), float('nan'), action, initnode)) # initial q
    iter = 0
    while True:
        if time.time() - tt > config.timelimit:
            return None, iter

        iter += 1

        if iter > 3000:
            print('failed to find plan: iteration limit')
            return None, iter

        if action_queue.empty():
            print('failed to find plan: ran out of actions')
            return None, iter

        _, _, action, node = action_queue.get()
        state = node.state

        print('\n'.join([str(parent.action.discrete_parameters.values()) for parent in list(node.backtrack())[-2::-1]]))
        print("{}".format(action.discrete_parameters.values()))

        def collide(i, ignore=set(), target='robot', holding=None):
            q = prm_vertices[i]
            if True:
                set_robot_config(q, mover.robot)
                if mover.env.CheckCollision(mover.robot) or holding is not None and mover.env.CheckCollision(holding):
                    return True
            return False

        success = False
        if node.depth >= 2 and action.type == 'two_arm_pick' and node.parent.action.discrete_parameters['object'] == \
                action.discrete_parameters['object']:  # and plan[-1][1] == r:
            print('skipping because repeat', action.discrete_parameters['object'])
            continue

        if node.depth > depth_limit:
            print('skipping because depth limit', node.action.discrete_parameters.values())

        # reset to state
        for obj_name, obj_pose in state.object_poses.items():
            set_obj_xytheta(obj_pose, mover.env.GetKinBody(obj_name))
        set_robot_config(state.robot_pose, mover.robot)

        if action.type == 'two_arm_pick_two_arm_place':
            o = action.discrete_parameters['two_arm_place_object']
            obj = mover.env.GetKinBody(o)

            old_q = get_body_xytheta(mover.robot)
            old_p = get_body_xytheta(obj)

            success = False
            for _ in range(10):
                pick_action = None
                mover.enable_objects_in_region('entire_region')
                for _ in range(10):
                    a = ps.predict(obj, mover.regions['entire_region'], 10)
                    if a['base_pose'] is not None:
                        pick_action = a
                        break

                if pick_action is None:
                    print('pick_action is None')
                    set_robot_config(old_q, mover.robot)
                    set_obj_xytheta(old_p, obj)
                    continue

                start_neighbors = {
                    i for i, q in enumerate(prm_vertices)
                    if np.linalg.norm((q - old_q)[:2]) < .8
                }
                pick_neighbors = {
                    i for i, q in enumerate(prm_vertices)
                    if np.linalg.norm((q - pick_action['base_pose'])[:2]) < .8
                }
                if np.linalg.norm((pick_action['base_pose'] - old_q)[:2]) < .8:
                    pick_traj = []
                else:
                    pick_traj = dfs(start_neighbors, lambda i: i in pick_neighbors, lambda i: collide(i))

                if pick_traj is None:
                    print('pick_traj is None')
                    set_robot_config(old_q, mover.robot)
                    set_obj_xytheta(old_p, obj)
                    continue

                pick_action['path'] = [old_q] + [prm_vertices[i] for i in pick_traj] + [pick_action['base_pose']]

                success = True

                break

            if not success:
                continue

            r = action.discrete_parameters['two_arm_place_region']
            o = action.discrete_parameters['two_arm_place_object']
            obj = mover.env.GetKinBody(o)

            # old_q = get_body_xytheta(mover.robot)
            # old_p = get_body_xytheta(obj)

            success = False

            def reset_for_place():
                if len(mover.robot.GetGrabbed()) > 0:
                    release_obj()
                fold_arms()
                set_robot_config(old_q, mover.robot)
                set_obj_xytheta(old_p, obj)
                two_arm_pick_object(obj, pick_action)

            for _ in range(10):
                place_action = None
                mover.enable_objects_in_region('entire_region')
                for _ in range(10):
                    reset_for_place()
                    a = pls.predict(obj, mover.regions[r], 10)
                    q = a['base_pose']
                    p = a['object_pose']
                    if q is not None and p is not None:
                        # set_robot_config(q, mover.robot)
                        # set_obj_xytheta(p, obj)
                        # if not mover.env.CheckCollision(mover.robot) and not mover.env.CheckCollision(obj):
                        if True:
                            place_action = a
                            break

                # set_robot_config(old_q, mover.robot)

                if place_action is None:
                    print('place_action is None')
                    # set_robot_config(old_q, mover.robot)
                    # set_obj_xytheta(old_p, obj)
                    continue

                # grab_obj(mover.robot, obj)
                # set_obj_xytheta([1000,1000,0], obj)
                pick_neighbors = {
                    i for i, q in enumerate(prm_vertices)
                    if np.linalg.norm((q - pick_action['base_pose'])[:2]) < .8
                }
                place_neighbors = {
                    i for i, q in enumerate(prm_vertices)
                    if np.linalg.norm((q - place_action['base_pose'])[:2]) < .8
                }
                if np.linalg.norm((place_action['base_pose'] - pick_action['base_pose'])[:2]) < .8:
                    place_traj = []
                else:
                    set_robot_config(old_q, mover.robot)
                    set_obj_xytheta(old_p, obj)
                    two_arm_pick_object(obj, pick_action)
                    if mover.env.CheckCollision(mover.robot) or mover.env.CheckCollision(obj):
                        print('pick in collision')
                        two_arm_place_object(place_action)
                        continue
                    place_traj = dfs(list(pick_neighbors), lambda i: i in place_neighbors,
                                     lambda i: collide(i, holding=obj))
                    assert len(mover.robot.GetGrabbed()) > 0
                    if place_traj is not None:
                        for i in place_traj:
                            if collide(i, holding=obj):
                                print('in collision: {}'.format(i))
                                import pdb;
                                pdb.set_trace()

                            set_robot_config(prm_vertices[i], mover.robot)

                            if mover.env.CheckCollision(mover.robot):
                                import pdb;
                                pdb.set_trace()
                            for objj in mover.objects:
                                if mover.env.CheckCollision(objj):
                                    import pdb;
                                    pdb.set_trace()
                    two_arm_place_object(place_action)
                    if mover.env.CheckCollision(mover.robot) or mover.env.CheckCollision(obj):
                        print('place in collision')
                        continue
                # release_obj(mover.robot, obj)

                if place_traj is None:
                    print('place_traj is None')
                    # set_robot_config(old_q, mover.robot)
                    # set_obj_xytheta(old_p, obj)
                    continue


                success = True

                place_action['path'] = [pick_action['base_pose']] + [prm_vertices[i] for i in place_traj] + [
                    place_action['base_pose']]
                action.set_continuous_parameters((pick_action, place_action))

                set_robot_config(old_q, mover.robot)
                set_obj_xytheta(old_p, obj)
                two_arm_pick_object(obj, pick_action)
                for q in place_action['path']:
                    set_robot_config(q, mover.robot)

                    if mover.env.CheckCollision(mover.robot):
                        import pdb;
                        pdb.set_trace()
                    for objj in mover.objects:
                        if mover.env.CheckCollision(objj):
                            import pdb;
                            pdb.set_trace()
                two_arm_place_object(place_action)

                newstate = statecls(mover, goal, node.state, action)
                newstate.make_pklable()
                newnode = Node(node, action, newstate)

                if all(mover.regions['home_region'].contains_point(
                        get_body_xytheta(mover.env.GetKinBody(o))[0].tolist()[:2] + [1]) for o in
                       obj_names[:n_objs_pack]):
                    print("found successful plan: {}".format(n_objs_pack))
                    trajectory = Trajectory(mover.seed, mover.seed)
                    plan = list(newnode.backtrack())[::-1]
                    trajectory.states = [nd.state for nd in plan]
                    trajectory.actions = [nd.action for nd in plan[1:]]
                    trajectory.rewards = [nd.reward for nd in plan[1:]]
                    trajectory.state_prime = [nd.state for nd in plan[1:]]
                    trajectory.seed = mover.seed
                    print(trajectory)
                    # if len(mover.robot.GetGrabbed()) > 0:
                    #	release_obj()
                    return trajectory, iter

                for o in obj_names:
                    if o == action.discrete_parameters['two_arm_place_object']:
                        continue
                    for r in ('home_region', 'loading_region'):
                        newaction = Operator('two_arm_pick_two_arm_place',
                                             {'two_arm_place_object': o, 'two_arm_place_region': r})
                        if o not in goal and r in goal:
                            # you cannot place non-goal object in the goal region
                            continue
                        action_queue.put(
                            (heuristic(newstate, newaction) - 1. * newnode.depth, float('nan'), newaction, newnode))

                break

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
        else:
            raise NotImplementedError

        if not success:
            print('failed to execute action')
        else:
            print('action successful')


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
