prm_vertices = prm_edges = None

from pick_and_place_state import PaPState


class OneArmPaPState(PaPState):
    def __init__(self, problem_env, goal_entities, parent_state=None, parent_action=None, paps_used_in_data=None):
        PaPState.__init__(problem_env, goal_entities, parent_state=None, parent_action=None, paps_used_in_data=None)

    def get_nodes(self):
        nodes = {}
        for entity in self.problem_env.entity_names:
            nodes[entity] = self.get_node_features(entity, self.goal_entities)
        return nodes

    def get_binary_edges(self):
        raise NotImplementedError

    def get_ternary_edges(self):
        raise NotImplementedError

    def get_node_features(self, entity, goal_entities):
        raise NotImplementedError

    def get_ternary_edge_features(self, a, b, r):
        raise NotImplementedError

    def get_binary_edge_features(self, a, b):
        raise NotImplementedError
