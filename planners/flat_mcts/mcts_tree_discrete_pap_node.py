from mcts_tree_discrete_node import DiscreteTreeNode
import numpy as np
import openravepy
from manipulation.bodies.bodies import set_color
from mover_library.utils import visualize_path, set_color, viewer


class PaPDiscreteTreeNodeWithLearnedQ(DiscreteTreeNode):
    def __init__(self, state, ucb_parameter, depth, state_saver, is_operator_skeleton_node, is_init_node, learned_q,
                 actions):
        DiscreteTreeNode.__init__(self, state, ucb_parameter, depth, state_saver, is_operator_skeleton_node,
                                  is_init_node, actions)
        self.learned_q = learned_q
        self.q_function = self.learned_q

        is_infeasible_state = state is None
        self.learned_q = {}
        if not is_infeasible_state:
            self.initialize_mixed_q_values()
        self.mix_weight = 0.99

    def visualize_values_in_two_arm_domains(self, entity_values, entity_names):
        is_place_node = entity_names[0].find('region') != -1
        if is_place_node:
            return

        max_val = np.max(entity_values)
        min_val = np.min(entity_values)
        for entity_name, entity_value in zip(entity_names, entity_values):
            entity = openravepy.RaveGetEnvironments()[0].GetKinBody(entity_name)
            set_color(entity, [0, (entity_value - min_val) / (max_val - min_val), 0])

    def initialize_mixed_q_values(self):
        entity_values = []
        entity_names = []
        for a in self.A:
            self.Q[a] = self.q_function.predict(self.state, a)[0]
            self.learned_q[a] = self.q_function.predict(self.state, a)[0]
            # self.learnedQ[a] = self.mixedQ[a]
        """
            print a.discrete_parameters, self.learned_q[a]
            entity_names.append(a.discrete_parameters['two_arm_place_object'])
            entity_values.append(self.learned_q[a])
        self.visualize_values_in_two_arm_domains(entity_values, entity_names)
        print self.state.get_entities_in_pick_way('square_packing_box1')
        print self.state.get_entities_in_place_way('square_packing_box1', 'home_region')
        import pdb; pdb.set_trace()
        """

    def update_node_statistics(self, action, sum_rewards, reward):
        DiscreteTreeNode.update_node_statistics(self, action, sum_rewards, reward)
        # self.update_mixed_q_value(action)

    def perform_ucb_over_actions(self, learned_q_functions=None):
        # why does this get called before initializing mixed q values
        # todo why does it ever have key error here?
        # it performs ucb_over_actions in an infeasible state?
        q_vals = [self.Q[a] for a in self.A]
        best_action = self.get_action_with_highest_ucb_value(self.A, q_vals)  # but your Nsa are all zero?
        return best_action
