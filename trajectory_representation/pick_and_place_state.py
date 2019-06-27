from trajectory_representation.state import State
from trajectory_representation.operator import Operator
from generators.uniform import UniformGenerator
from mover_library.utils import CustomStateSaver, get_body_xytheta, set_robot_config, set_obj_xytheta

from predicates.is_reachable import IsReachable
from predicates.is_holding_goal_entity import IsHoldingGoalEntity
from predicates.place_in_way import PlaceInWay
from predicates.pick_in_way import PickInWay
from predicates.in_region import InRegion

from planners.subplanners.motion_planner import BaseMotionPlanner

import copy
from mover_library.utils import visualize_path, two_arm_pick_object
from manipulation.bodies.bodies import set_color


class PaPState(State):
    def __init__(self, problem_env, goal_entities, parent_state=None, parent_action=None, paps_used_in_data=None):
        self.state_saver = CustomStateSaver(problem_env.env)
        self.problem_env = problem_env
        self.parent_state = parent_state  # used to update the node features
        self.goal_entities = goal_entities

        # raw variables
        self.robot_pose = get_body_xytheta(problem_env.robot)
        self.object_poses = {
            obj.GetName(): get_body_xytheta(obj)
            for obj in problem_env.objects
        }

        # cached info
        self.use_prm = problem_env.name.find('two_arm') != -1
        if self.use_prm:
            self.collides, self.current_collides = self.update_collisions_at_prm_vertices(parent_state)
        else:
            self.collides = None
            self.current_collides = None

        if paps_used_in_data is not None:
            self.pick_used = paps_used_in_data[0]
            self.place_used = paps_used_in_data[1]

        self.mc_pick_path = {}
        self.mc_place_path = {}
        self.reachable_entities = []

        self.pick_in_way = None
        self.place_in_way = None
        self.in_region = None
        self.is_holding_goal_entity = None

        self.ternary_edges = None
        self.binary_edges = None
        self.nodes = None

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

    def update_cached_data_after_binary(self):
        self.mc_pick_path = self.pick_in_way.mc_path_to_entity
        self.reachable_entities = self.pick_in_way.reachable_entities
        self.pick_used = self.pick_in_way.pick_used

    def update_cached_data_after_ternary(self):
        self.place_used = self.place_in_way.place_used
        self.mc_place_path = self.place_in_way.mc_path_to_entity
        self.pick_used = self.place_in_way.pick_used

    def is_entity_reachable(self, entity):
        return self.nodes[entity][-2]

    def pick_entities_occluded_by(self, entity):
        inway = []
        entity_names = self.problem_env.object_names + self.problem_env.region_names
        for obj_name in entity_names:
            if self.binary_edges[(entity, obj_name)][1]:
                inway.append(obj_name)
        return inway

    def place_entities_occluded_by(self, entity):
        inway = []
        for region_name in self.problem_env.region_names:
            if region_name == 'entire_region':
                continue
            for obj_name in self.problem_env.object_names:
                if self.ternary_edges[(obj_name, entity, region_name)][0]:
                    inway.append(obj_name, region_name)
        return inway

    def get_entities_in_place_way(self, entity, region):
        inway = []
        for obj_name in self.problem_env.object_names:
            if self.ternary_edges[(entity, obj_name, region)][0]:
                inway.append(obj_name)
        return inway

    def get_entities_in_pick_way(self, entity):
        inway = []
        for obj_name in self.problem_env.object_names:
            if self.binary_edges[(obj_name, entity)][1]:
                inway.append(obj_name)
        return inway

    def visualize_place_inways(self):
        self.problem_env.env.SetViewer('qtcoin')
        for key, val in self.place_in_way.mc_path_to_entity.items():
            hold_obj_name = key[0]
            region_name = key[1]

            objs_in_way = self.place_in_way.mc_to_entity[key]
            if len(objs_in_way) > 0:
                saver = CustomStateSaver(self.problem_env.env)
                self.pick_used[hold_obj_name].execute()
                for tmp in objs_in_way:
                    set_color(self.problem_env.env.GetKinBody(tmp), [0, 0, 0])
                visualize_path(val)
                import pdb;pdb.set_trace()

                for tmp in objs_in_way:
                    set_color(self.problem_env.env.GetKinBody(tmp), [0, 1, 0])
                saver.Restore()

    def make_pklable(self):
        self.problem_env = None
        # self.is_reachable.problem_env = None
        self.in_region.problem_env = None
        self.pick_in_way.problem_env = None
        self.pick_in_way.robot = None
        self.is_holding_goal_entity.problem_env = None
        self.place_in_way.problem_env = None
        self.place_in_way.robot = None

        for operator in self.pick_used.values():
            operator.make_pklable()

        for operator in self.place_used.values():
            operator.make_pklable()

        if self.parent_state is not None:
            self.parent_state.make_pklable()

    def make_plannable(self, problem_env):
        self.problem_env = problem_env
        # self.is_reachable.problem_env = problem_env
        self.in_region.problem_env = problem_env
        self.pick_in_way.problem_env = problem_env
        self.pick_in_way.robot = problem_env.robot
        self.place_in_way.problem_env = problem_env
        self.place_in_way.robot = problem_env.robot
        self.is_holding_goal_entity.problem_env = problem_env
        if self.parent_state is not None:
            self.parent_state.make_plannable(problem_env)

