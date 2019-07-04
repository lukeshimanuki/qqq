from learn.data_traj import extract_individual_example as make_raw_format

from trajectory_representation.operator import Operator
from trajectory_representation.shortest_path_pick_and_place_state import ShortestPathPaPState
from trajectory_representation.one_arm_pap_state import OneArmPaPState
from trajectory_representation.trajectory import Trajectory

from mover_library import utils
from generators.uniform import PaPUniformGenerator

import time
import Queue
import numpy as np


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


class GreedySearch:
    def __init__(self, problem_env, pap_model, config):
        self.problem_env = problem_env
        self.config = config
        self.pap_model = pap_model
        self.n_objs_pack = config.n_objs_pack
        self.goal_objs = [obj.GetName() for obj in problem_env.objects]
        if config.domain == 'two_arm_mover':
            self.statecls = ShortestPathPaPState
            self.goal = ['home_region'] + [obj.GetName() for obj in self.problem_env.objects[:self.n_objs_pack]]
        elif config.domain == 'one_arm_mover':
            self.statecls = OneArmPaPState
            self.goal = ['rectangular_packing_box1_region'] + [obj.GetName() for obj in
                                                               self.problem_env.objects[:self.n_objs_pack]]
        else:
            raise NotImplementedError

        pass

    def compute_heuristic(self, state, action):
        o = action.discrete_parameters['two_arm_place_object']
        r = action.discrete_parameters['two_arm_place_region']
        nodes, edges, actions, _ = make_raw_format(state, action)
        nodes = nodes[..., 6:]

        object_is_goal = o in state.goal_entities
        region_is_goal = r in state.goal_entities
        number_in_goal1 = sum(state.binary_edges[(i, r)][0] for i in state.nodes for r in self.problem_env.regions if
                             i != o and state.nodes[r][8]) + int(region_is_goal)

        number_in_goal = 0
        for i in state.nodes:
            for r in self.problem_env.regions:
                is_i_in_r = state.binary_edges[(i, r)][0]
                is_r_goal_region = state.nodes[r][8]
                if i != o and is_r_goal_region:
                    number_in_goal += is_i_in_r
        number_in_goal += int(region_is_goal)
        assert number_in_goal == number_in_goal1

        if self.config.dont_use_gnn:
            return -number_in_goal
        elif self.config.dont_use_h:
            gnn_pred = -self.pap_model.predict_with_raw_input_format(nodes[None, ...], edges[None, ...],
                                                                     actions[None, ...])
            return gnn_pred
        else:
            gnn_pred = -self.pap_model.predict_with_raw_input_format(nodes[None, ...], edges[None, ...],
                                                                     actions[None, ...])
            return -number_in_goal + gnn_pred

    def get_actions(self):
        actions = []
        for o in self.problem_env.entity_names:
            if 'region' in o:
                continue
            for r in self.problem_env.entity_names:
                if 'region' not in r or 'entire' in r:
                    continue
                if o not in self.goal and r in self.goal:
                    # you cannot place non-goal object in the goal region
                    continue
                if self.config.domain == 'two_arm_mover':
                    action = Operator('two_arm_pick_two_arm_place',
                                      {'two_arm_place_object': o, 'two_arm_place_region': r})
                    # following two lines are for legacy reasons, will fix later
                    action.discrete_parameters['object'] = action.discrete_parameters['two_arm_place_object']
                    action.discrete_parameters['region'] = action.discrete_parameters['two_arm_place_region']
                elif self.config.domain == 'one_arm_mover':
                    action = Operator('one_arm_pick_one_arm_place',
                                      {'object': self.problem_env.env.GetKinBody(o),
                                       'region': self.problem_env.regions[r]})
                else:
                    raise NotImplementedError
                actions.append(action)

        return actions

    def search(self):
        tt = time.time()
        depth_limit = 60

        state = self.statecls(self.problem_env, self.goal)
        action_queue = Queue.PriorityQueue()  # (self.compute_heuristic, nan, operator skeleton, state. trajectory)
        initnode = Node(None, None, state)
        initial_state = state
        actions = self.get_actions()
        for a in actions:
            action_queue.put((self.compute_heuristic(state, a), float('nan'), a, initnode))  # initial q

        iter = 0
        # beginning of the planner
        while True:
            if time.time() - tt > self.config.timelimit:
                return None, iter

            iter += 1

            if iter > 3000:
                print('failed to find plan: iteration limit')
                return None, iter

            if action_queue.empty():
                actions = self.get_actions()
                for a in actions:
                    action_queue.put((self.compute_heuristic(initial_state, a), float('nan'), a, initnode))  # initial q

            curr_hval, _, action, node = action_queue.get()
            state = node.state

            print('\n'.join(
                [str(parent.action.discrete_parameters.values()) for parent in list(node.backtrack())[-2::-1]]))
            print("{}".format(action.discrete_parameters.values()))

            if node.depth >= 2 and action.type == 'two_arm_pick' and node.parent.action.discrete_parameters['object'] == \
                    action.discrete_parameters['object']:  # and plan[-1][1] == r:
                print('skipping because repeat', action.discrete_parameters['object'])
                continue

            if node.depth > depth_limit:
                print('skipping because depth limit', node.action.discrete_parameters.values())

            # reset to state
            state.restore(self.problem_env)
            utils.set_color(action.discrete_parameters['object'], [1, 0, 0])  # visualization purpose

            if action.type == 'two_arm_pick_two_arm_place':
                smpler = PaPUniformGenerator(action, self.problem_env, None)
                smpled_param = smpler.sample_next_point(action, n_iter=200, n_parameters_to_try_motion_planning=3,
                                                        cached_collisions=state.collides,
                                                        cached_holding_collisions=None)
                if smpled_param['is_feasible']:
                    action.continuous_parameters = smpled_param
                    action.execute()
                    print "Action executed"
                else:
                    print "Failed to sample an action"
                    utils.set_color(action.discrete_parameters['object'], [0, 1, 0])  # visualization purpose
                    continue

                is_goal_achieved = \
                    np.all([self.problem_env.regions['home_region'].contains(
                        self.problem_env.env.GetKinBody(o).ComputeAABB())
                            for o in self.goal_objs])
                if is_goal_achieved:
                    print("found successful plan: {}".format(self.n_objs_pack))
                    trajectory = Trajectory(self.problem_env.seed, self.problem_env.seed)
                    plan = list(newnode.backtrack())[::-1]
                    trajectory.states = [nd.state for nd in plan]
                    trajectory.actions = [nd.action for nd in plan[1:]]
                    trajectory.rewards = [nd.reward for nd in plan[1:]]
                    trajectory.state_prime = [nd.state for nd in plan[1:]]
                    trajectory.seed = self.problem_env.seed
                    print(trajectory)
                    return trajectory, iter
                else:
                    newstate = self.statecls(self.problem_env, self.goal, node.state, action)
                    print "New state computed"
                    newstate.make_pklable()
                    newnode = Node(node, action, newstate)
                    newactions = self.get_actions()
                    print "Old h value", curr_hval
                    for newaction in newactions:
                        hval = self.compute_heuristic(newstate, newaction) - 1. * newnode.depth
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
                            'region': self.problem_env.regions[r],
                        },
                        continuous_parameters={
                            'pick': pick_op.continuous_parameters,
                            'place': place_op.continuous_parameters,
                        }
                    )
                    action.execute()

                    success = True

                    newstate = self.statecls(self.problem_env, self.goal, node.state, action)
                    newstate.make_pklable()
                    newnode = Node(node, action, newstate)
                    is_goal_achieved = \
                        np.all([self.problem_env.regions['rectangular_packing_box1_region'].contains(
                            self.problem_env.env.GetKinBody(o).ComputeAABB())
                            for o in self.goal_objs])

                    if is_goal_achieved:
                        print("found successful plan: {}".format(self.n_objs_pack))
                        trajectory = Trajectory(self.problem_env.seed, self.problem_env.seed)
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
                        trajectory.seed = self.problem_env.seed
                        print(trajectory)
                        return trajectory, iter
                    else:
                        newactions = self.get_actions()
                        for newaction in newactions:
                            action_queue.put(
                                (self.compute_heuristic(newstate, newaction) - 1. * newnode.depth, float('nan'),
                                 newaction, newnode))

                    if not success:
                        print('failed to execute action')
                    else:
                        print('action successful')
            else:
                raise NotImplementedError
