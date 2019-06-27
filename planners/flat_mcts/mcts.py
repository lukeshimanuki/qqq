import sys
import socket
import pickle

from mcts_tree_continuous_node import ContinuousTreeNode
from mcts_tree_discrete_node import DiscreteTreeNode, DiscreteTreeNodeWithLearnedQ
from mcts_tree_discrete_pap_node import PaPDiscreteTreeNodeWithLearnedQ
from mcts_tree import MCTSTree
# from mcts_utils import make_action_hashable, is_action_hashable
from generators.uniform import UniformGenerator, PaPUniformGenerator
from trajectory_representation.state import State, StateForPrediction
from trajectory_representation.shortest_path_pick_and_place_state import ShortestPathPaPState

## openrave helper libraries
from mover_library.utils import *

import time
import os

sys.setrecursionlimit(15000)
DEBUG = False

hostname = socket.gethostname()
if hostname == 'dell-XPS-15-9560':
    from mcts_graphics import write_dot_file


class MCTS:
    def __init__(self, widening_parameter, ucb_parameter, n_feasibility_checks, environment,
                 n_parameters_to_test_each_sample_time,
                 depth_limit, discount_rate, check_reachability, use_progressive_widening, use_ucb,
                 learned_q_function, n_motion_plan_trials, goal_entities):
        self.widening_parameter = widening_parameter
        self.ucb_parameter = ucb_parameter
        self.time_limit = np.inf
        self.check_reachability = check_reachability
        self.discount_rate = discount_rate
        self.n_parameters_to_test_each_sample_time = n_parameters_to_test_each_sample_time
        self.n_motion_plan_trials = n_motion_plan_trials

        self.environment = environment
        self.depth_limit = depth_limit
        self.env = self.environment.env
        self.robot = self.environment.robot
        self.s0_node = None
        self.tree = MCTSTree(self.ucb_parameter)
        self.best_leaf_node = None
        self.use_ucb = use_ucb
        self.use_progressive_widening = use_progressive_widening
        self.goal_entities = goal_entities

        # logging purpose
        self.search_time_to_reward = []
        self.reward_lists = []
        self.progress_list = []

        self.found_solution = False
        self.goal_reward = 2
        self.n_feasibility_checks = n_feasibility_checks
        self.swept_volume_constraint = None

        self.use_learned_q = False
        self.learned_q_function = learned_q_function
        if learned_q_function is not None:
            self.use_learned_q = True
            self.learned_q_function = learned_q_function

    def load_pickled_tree(self, fname=None):
        if fname is None:
            fname = 'tmp_tree.pkl'
        self.tree = pickle.load(open(fname, 'r'))

    def visit_all_nodes(self, curr_node):
        children = curr_node.children.values()
        print curr_node in self.tree.nodes
        for c in children:
            self.visit_all_nodes(c)

    def save_tree(self, fname=None):
        if fname is None:
            fname = 'tmp_tree.pkl'
        self.tree.make_tree_picklable()
        pickle.dump(self.tree, open(fname, 'wb'))

    def load_tree(self, fname=None):
        if fname is None:
            fname = 'tmp_tree.pkl'
        self.tree = pickle.load(open(fname, 'r'))

    def get_node_at_idx(self, idx):
        for n in self.tree.nodes:
            if n.idx == idx:
                return n
        return None

    def set_place_operator_swept_volume_constraint(self, constraint):
        self.swept_volume_constraint = constraint  # place operation should place object outside swept_volume

    def create_sampling_agent(self, operator_skeleton):
        is_pap = operator_skeleton.type.find('pick') != -1 and operator_skeleton.type.find('place') != -1
        if is_pap:
            return PaPUniformGenerator(operator_skeleton, self.environment, None)
        else:
            if operator_skeleton.type.find('pick') != -1:
                return UniformGenerator(operator_skeleton, self.environment, None)
            elif operator_skeleton.type.find('place') != -1:
                return UniformGenerator(operator_skeleton, self.environment, self.swept_volume_constraint)

    def get_current_state(self, parent_node, parent_action, reward):
        is_operator_skeleton_node = (parent_node is None) or (not parent_node.is_operator_skeleton_node)
        is_parent_action_infeasible = reward == -2
        if self.use_learned_q:
            if is_parent_action_infeasible:
                state = None
            elif is_operator_skeleton_node:
                if self.environment.is_goal_reached():
                    state = parent_node.state
                else:
                    if parent_node is None:
                        parent_state = None
                    else:
                        parent_state = parent_node.state
                    # where is the parent state?
                    state = ShortestPathPaPState(self.environment,
                                                 parent_state=parent_state,
                                                 parent_action=parent_action,
                                                 goal_entities=self.goal_entities)

                """
                fstate = './cached_states/n_objs_pack_%d_%s_pidx_%d_node_idx_%d_state.pkl' % (
                    len(self.goal_entities) - 1,
                    self.environment.name,
                    self.environment.problem_idx,
                    len(self.tree.nodes))
                previously_stored_state_file_exists = os.path.isfile(fstate)
                if previously_stored_state_file_exists:
                    print "Loading the cached state file", fstate
                    state = pickle.load(open(fstate, 'r'))
                else:
                    if self.environment.is_goal_reached():
                        state = parent_node.state
                    elif parent_node is not None:
                        state = PaPState(self.environment, parent_state=parent_node.state, parent_action=parent_action,
                                         goal_entities=self.goal_entities)
                    else:
                        state = PaPState(self.environment, goal_entities=self.goal_entities)
                    state.make_pklable()  # removing openrave files to pkl
                    pickle.dump(state, open(fstate, 'wb'))
                    state.make_plannable(self.environment)
                """
            else:
                state = parent_node.state
        else:
            state = None
        return state

    def create_node(self, parent_action, depth, reward, parent_node, is_init_node=False):
        state_saver = CustomStateSaver(self.environment.env)

        is_operator_skeleton_node = (parent_node is None) or (not parent_node.is_operator_skeleton_node)
        state = self.get_current_state(parent_node, parent_action, reward)

        if is_operator_skeleton_node:
            applicable_op_skeletons = self.environment.get_applicable_ops(parent_action)
            if self.use_learned_q:
                # where is the parent_node?
                node = PaPDiscreteTreeNodeWithLearnedQ(state, self.ucb_parameter, depth, state_saver,
                                                       is_operator_skeleton_node,
                                                       is_init_node, self.learned_q_function, applicable_op_skeletons,
                                                       is_goal_reached=self.environment.is_goal_reached())
            else:
                node = DiscreteTreeNode(state, self.ucb_parameter, depth, state_saver, is_operator_skeleton_node,
                                        is_init_node, applicable_op_skeletons)
        else:
            node = ContinuousTreeNode(state, parent_action, self.ucb_parameter, depth, state_saver,
                                      is_operator_skeleton_node, is_init_node)
            node.sampling_agent = self.create_sampling_agent(node.operator_skeleton)

            # apply constraint on continuous parameters if exists; used for hierarchical mcts
            if parent_action.type == 'two_arm_pick' and \
                    self.environment.two_arm_pick_continuous_constraint is not None:
                node.add_actions(self.environment.two_arm_pick_continuous_constraint)  # why do I have this?
            elif parent_action.type == 'two_arm_place' and \
                    self.environment.two_arm_place_continuous_constraint is not None:
                node.add_actions(self.environment.two_arm_place_continuous_constraint)

        node.parent = parent_node
        node.parent_action_reward = reward
        node.parent_action = parent_action
        return node

    @staticmethod
    def get_best_child_node(node):
        if len(node.children) == 0:
            return None
        else:
            best_child_action_idx = np.argmax(node.Q.values())
            best_child_action = node.Q.keys()[best_child_action_idx]
            return node.children[best_child_action]

    def retrace_best_plan(self):
        plan = []
        _, _, best_leaf_node = self.tree.get_best_trajectory_sum_rewards_and_node(self.discount_rate)
        curr_node = best_leaf_node

        while not curr_node.parent is None:
            plan.append(curr_node.parent_action)
            curr_node = curr_node.parent

        plan = plan[::-1]
        return plan, best_leaf_node

    def get_best_goal_node(self):
        leaves = self.tree.get_leaf_nodes()
        goal_nodes = [leaf for leaf in leaves if leaf.is_goal_node]
        if len(goal_nodes) > 1:
            best_traj_reward, curr_node, _ = self.tree.get_best_trajectory_sum_rewards_and_node(self.discount_rate)
        else:
            curr_node = goal_nodes[0]
        return curr_node

    def switch_init_node(self, node):
        self.s0_node.is_init_node = False
        self.s0_node = node
        self.s0_node.is_init_node = True
        self.environment.reset_to_init_state(node)
        self.found_solution = False

    @staticmethod
    def choose_child_node_to_descend_to(node):
        # todo: implement the one with highest visitation
        if node.is_operator_skeleton_node and len(node.A) == 1:
            # descend to grand-child
            only_child_node = node.children.values()[0]
            best_action = only_child_node.Q.keys()[np.argmax(only_child_node.Q.values())]
            best_node = only_child_node.children[best_action]
        else:
            best_action = node.Q.keys()[np.argmax(node.Q.values())]
            best_node = node.children[best_action]
        return best_node

    def log_current_tree_to_dot_file(self, iteration, node_to_search_from):
        if socket.gethostname() == 'dell-XPS-15-9560':
            write_dot_file(self.tree, iteration, '', node_to_search_from)

    def log_performance(self, time_to_search, iteration, ):
        best_traj_rwd, progress, best_node = self.tree.get_best_trajectory_sum_rewards_and_node(self.discount_rate)
        self.search_time_to_reward.append([time_to_search, iteration, best_traj_rwd, self.found_solution])
        self.progress_list.append(progress)
        self.best_leaf_node = best_node

    def is_optimal_solution_found(self):
        best_traj_rwd, best_node, reward_list = self.tree.get_best_trajectory_sum_rewards_and_node(self.discount_rate)
        if self.found_solution:
            if self.environment.reward_function.is_optimal_plan_found(best_traj_rwd):
                print "Optimal score found"
                return True
            else:
                return False
        else:
            return False

    def search(self, n_iter=np.inf, iteration_for_tree_logging=0, node_to_search_from=None, max_time=np.inf):
        depth = 0
        time_to_search = 0

        if node_to_search_from is None:
            self.s0_node = self.create_node(None, depth=0, reward=0, parent_node=None, is_init_node=True)
            self.tree.set_root_node(self.s0_node)
            node_to_search_from = self.s0_node

        new_trajs = []
        plan = []
        if n_iter == np.inf:
            n_iter = 999999
        for iteration in range(1, n_iter):
            print '*****SIMULATION ITERATION %d' % iteration
            self.environment.reset_to_init_state(node_to_search_from)

            new_traj = []
            stime = time.time()
            self.simulate(node_to_search_from, node_to_search_from, depth, new_traj)
            time_to_search += time.time() - stime
            new_trajs.append(new_traj)

            is_time_to_switch_node = iteration % 10 == 0
            # I have to have a feasible action to switch if this is an instance node
            if is_time_to_switch_node:
                if node_to_search_from.is_operator_skeleton_node:
                    node_to_search_from = node_to_search_from.get_child_with_max_value()
                else:
                    max_child = node_to_search_from.get_child_with_max_value()
                    if np.max(node_to_search_from.reward_history[max_child.parent_action]) != -2:
                        node_to_search_from = node_to_search_from.get_child_with_max_value()

            # self.log_current_tree_to_dot_file(iteration_for_tree_logging+iteration, node_to_search_from)
            self.log_performance(time_to_search, iteration)
            print self.search_time_to_reward[iteration_for_tree_logging:]

            # break if the solution is found
            if self.is_optimal_solution_found():
                print "Optimal score found"
                plan, _ = self.retrace_best_plan()
                break

            if time_to_search > max_time:
                print "Time is up"
                break

        self.environment.reset_to_init_state(node_to_search_from)
        return self.search_time_to_reward, plan

    def get_best_trajectory(self, node_to_search_from, trajectories):
        traj_rewards = []
        curr_node = node_to_search_from
        for trj in trajectories:
            traj_sum_reward = 0
            for aidx, a in enumerate(trj):
                traj_sum_reward += np.power(self.discount_rate, aidx) * curr_node.reward_history[a][0]
                curr_node = curr_node.children[a]
            traj_rewards.append(traj_sum_reward)
        return trajectories[np.argmax(traj_rewards)], curr_node

    def choose_action(self, curr_node):
        if curr_node.is_operator_skeleton_node:
            print "Skeleton node"

            if curr_node.state is None:
                import pdb; pdb.set_trace()

            action = curr_node.perform_ucb_over_actions(self.learned_q_function)
        else:
            print 'Instance node'
            if curr_node.sampling_agent is None:  # this happens if the tree has been pickled
                curr_node.sampling_agent = self.create_sampling_agent(curr_node.operator_skeleton)
            if not curr_node.is_reevaluation_step(self.widening_parameter,
                                                  self.environment.reward_function.infeasible_reward,
                                                  self.use_progressive_widening,
                                                  self.use_ucb):
                print "Sampling new action"
                new_continuous_parameters = self.sample_continuous_parameters(curr_node)
                curr_node.add_actions(new_continuous_parameters)
                action = curr_node.A[-1]
            else:
                print "Re-evaluation of actions"
                if self.use_ucb:
                    action = curr_node.perform_ucb_over_actions()
                else:
                    action = curr_node.choose_new_arm()
        return action

    @staticmethod
    def update_goal_node_statistics(curr_node, reward):
        # todo rewrite this function
        curr_node.Nvisited += 1
        curr_node.reward = reward

    def simulate(self, curr_node, node_to_search_from, depth, new_traj):
        if self.environment.is_goal_reached():
            if not curr_node.is_goal_and_already_visited:
                self.found_solution = True
                curr_node.is_goal_node = True
                print "Solution found, returning the goal reward", self.goal_reward
                self.update_goal_node_statistics(curr_node, self.goal_reward)
            return self.goal_reward

        if depth == self.depth_limit:
            # would it ever get here? why does it not satisfy the goal?
            print "Depth limit reached"
            return 0

        if DEBUG:
            print "At depth ", depth
            print "Is it time to pick?", self.environment.is_pick_time()

        if curr_node.state is None:
            # is it none because it was infeasible before?
            # it should never get here
            sys.exit(-1)

        action = self.choose_action(curr_node)
        reward = self.apply_action(curr_node, action)
        print "Reward is", reward

        if not curr_node.is_action_tried(action):
            next_node = self.create_node(action, depth + 1, reward, curr_node)
            self.tree.add_node(next_node, action, curr_node)
            next_node.sum_ancestor_action_rewards = next_node.parent.sum_ancestor_action_rewards + reward
        else:
            next_node = curr_node.children[action]

        is_infeasible_action = reward == self.environment.reward_function.infeasible_reward
        if is_infeasible_action:
            # this (s,a) is a dead-end
            print "Infeasible action"
            # todo use the average of Q values here, instead of termination
            sum_rewards = reward + curr_node.parent.learned_q[curr_node.parent_action]
        else:
            sum_rewards = reward + self.discount_rate * self.simulate(next_node, node_to_search_from, depth + 1,
                                                                      new_traj)

        curr_node.update_node_statistics(action, sum_rewards, reward)
        if curr_node == node_to_search_from and curr_node.parent is not None:
            self.update_ancestor_node_statistics(curr_node.parent, curr_node.parent_action, sum_rewards)

        # todo return a plan
        return sum_rewards

    def update_ancestor_node_statistics(self, node, action, child_sum_rewards):
        if node is None:
            return
        parent_reward_to_node = node.reward_history[action][0]
        parent_sum_rewards = parent_reward_to_node + self.discount_rate * child_sum_rewards
        node.update_node_statistics(action, parent_sum_rewards, parent_reward_to_node)
        self.update_ancestor_node_statistics(node.parent, node.parent_action, parent_sum_rewards)

    def apply_action(self, node, action):
        if node.is_operator_skeleton_node:
            print "Applying skeleton", action.type, action.discrete_parameters['object'], action.discrete_parameters['region']
            reward = self.environment.apply_operator_skeleton(action)
        else:
            print "Applying instance", action.type, action.discrete_parameters['object'], action.discrete_parameters['region']
            reward = self.environment.apply_operator_instance(action, self.check_reachability)
        return reward

    def sample_continuous_parameters(self, node):
        if self.environment.name.find('one_arm') != -1:
            feasible_param = node.sampling_agent.sample_next_point(node.operator_skeleton,
                                                                   self.n_feasibility_checks,
                                                                   n_parameters_to_try_motion_planning=1,
                                                                   no_motion_plan=True)
        else:
            current_collides = node.state.current_collides if node.state is not None else None
            current_holding_collides = node.state.current_holding_collides if node.state is not None else None
            feasible_param = node.sampling_agent.sample_next_point(node.operator_skeleton,
                                                                   self.n_feasibility_checks,
                                                                   self.n_motion_plan_trials,
                                                                   current_collides,
                                                                   current_holding_collides)
        return feasible_param
