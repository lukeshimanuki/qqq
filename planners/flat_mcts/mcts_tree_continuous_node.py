import numpy as np
from trajectory_representation.operator import Operator
from planners.flat_mcts.mcts_tree_node import TreeNode


def upper_confidence_bound(n, n_sa):
    return 2 * np.sqrt(np.log(n) / float(n_sa))


class ContinuousTreeNode(TreeNode):
    def __init__(self, state, operator_skeleton, ucb_parameter, depth, state_saver, is_operator_skeleton_node,
                 is_init_node):
        TreeNode.__init__(self, state, ucb_parameter, depth, state_saver, is_operator_skeleton_node, is_init_node)
        self.operator_skeleton = operator_skeleton
        # todo initialize actions with the state evaluation

    def add_actions(self, action):
        new_action = Operator(operator_type=self.operator_skeleton.type,
                              discrete_parameters=self.operator_skeleton.discrete_parameters,
                              continuous_parameters=action)
        self.A.append(new_action)
        self.N[new_action] = 0

    def is_reevaluation_step(self, widening_parameter, infeasible_rwd, use_progressive_widening, use_ucb):
        n_arms = len(self.A)
        if n_arms < 1:
            return False

        max_reward_of_each_action = np.array([np.max(rlist) for rlist in self.reward_history.values()])
        n_feasible_actions = np.sum(max_reward_of_each_action > infeasible_rwd)

        if n_feasible_actions < 1:
            return False

        if not use_ucb:
            new_action = self.A[-1]
            is_new_action_infeasible = np.max(self.reward_history[new_action]) <= -2
            if is_new_action_infeasible:
                return False

        if use_progressive_widening:
            n_actions = len(self.A)
            is_time_to_sample = n_actions <= widening_parameter * self.Nvisited
            return is_time_to_sample
        else:
            if self.n_ucb_iterations < widening_parameter:
                self.n_ucb_iterations += 1
                return True
            else:
                self.n_ucb_iterations = 0
                return False

    def get_never_evaluated_action(self):
        # get list of actions that do not have an associated Q values
        no_evaled = [a for a in self.A if a not in self.Q.keys()]
        no_evaled_feasible = [a for a in no_evaled if a.continuous_parameters['base_pose'] is not None]
        if len(no_evaled_feasible) == 0:
            return np.random.choice(no_evaled)
        else:
            return np.random.choice(no_evaled_feasible)

    def perform_ucb_over_actions(self, qinit=None):
        best_value = -np.inf
        never_executed_actions_exist = len(self.Q) != len(self.A)

        if never_executed_actions_exist:
            best_action = self.get_never_evaluated_action()
        else:
            best_action = self.Q.keys()[0]
            for action, value in zip(self.Q.keys(), self.Q.values()):
                if action.continuous_parameters['base_pose'] is None:
                    continue

                ucb_value = value + self.ucb_parameter * upper_confidence_bound(self.Nvisited, self.N[action])

                # todo randomized tie-break
                if ucb_value > best_value:
                    best_action = action
                    best_value = ucb_value

        return best_action
