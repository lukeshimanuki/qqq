from generators.uniform import UniformGenerator
from manipulation.bodies.bodies import set_color
from mover_library import utils
from mover_library.utils import CustomStateSaver, get_body_xytheta, set_robot_config, set_obj_xytheta, visualize_path, one_arm_pick_object
from pick_and_place_state import PaPState
from planners.subplanners.motion_planner import BaseMotionPlanner
from predicates.in_region import InRegion
from predicates.is_holding_goal_entity import IsHoldingGoalEntity
from predicates.pick_in_way import PickInWay
from predicates.place_in_way import PlaceInWay
from trajectory_representation.operator import Operator
from trajectory_representation.state import State
import pickle
import copy

class OneArmPaPState(PaPState):
    def __init__(self, problem_env, goal_entities, parent_state=None, parent_action=None, paps_used_in_data=None):
        PaPState.__init__(self, problem_env, goal_entities, parent_state=None, parent_action=None)

        self.parent_state = parent_state
        self.parent_ternary_predicates = {}
        self.parent_binary_predicates = {}
        if parent_state is not None:
            moved_obj_type = type(parent_action.discrete_parameters['object'])
            if moved_obj_type == str or moved_obj_type == unicode:
                moved_obj = parent_action.discrete_parameters['object']
            else:
                moved_obj = parent_action.discrete_parameters['object'].GetName()
            self.initialize_parent_predicates(moved_obj, parent_state, parent_action)
        else:
            moved_obj = None
        problem_env.enable_objects_in_region('entire_region')

        object_names = {obj.GetName() for obj in problem_env.objects}

        problem_env.disable_objects()
        self.pick_params = {}
        for obj in object_names:
            if parent_state is not None and obj != moved_obj:
                self.pick_params[obj] = parent_state.pick_params[obj]
            else:
                op = Operator(operator_type='one_arm_pick', discrete_parameters={'object': problem_env.env.GetKinBody(obj)})
                params, status = UniformGenerator(op, problem_env).sample_feasible_op_parameters(op, 300, 20)
                if status == 'HasSolution':
                    self.pick_params[obj] = params
                else:
                    self.pick_params[obj] = []
        problem_env.enable_objects()

        self.nocollision_pick_op = {}
        self.collision_pick_op = {}
        for obj, params in self.pick_params.items():
            for pp in params:
                pick_op = Operator(
                    operator_type='one_arm_pick',
                    discrete_parameters={'object': problem_env.env.GetKinBody(obj)},
                    continuous_parameters=pp
                )
                before = CustomStateSaver(problem_env.env)
                pick_op.execute()
                if not problem_env.env.CheckCollision(problem_env.robot):
                    self.nocollision_pick_op[obj] = pick_op
                    before.Restore()
                    break

                collisions = {
                    o for o in object_names
                    if self.problem_env.env.CheckCollision(self.problem_env.env.GetKinBody(o))
                }
                if obj not in self.collision_pick_op or len(collisions) < len(self.collision_pick_op[obj][1]):
                    self.collision_pick_op[obj] = pick_op, collisions

                before.Restore()

        # assume that pick params don't affect placement feasibility much
        self.place_params = {}
        for obj in object_names:
            for r in problem_env.regions:
                if obj not in self.nocollision_pick_op:
                    self.place_params[(obj, r)] = []
                    continue
                pick = self.nocollision_pick_op[obj]
                before = CustomStateSaver(problem_env.env)
                pick.execute()
                place_op = Operator(
                    operator_type='one_arm_place',
                    discrete_parameters={'object': problem_env.env.GetKinBody(obj), 'region': problem_env.regions[r]},
                    continuous_parameters=pick.continuous_parameters
                )
                params, status = UniformGenerator(op, problem_env).sample_feasible_op_parameters(op, 300, 20)
                if status == 'HasSolution':
                    self.place_params[(obj, r)] = params
                else:
                    self.place_params[(obj, r)] = []
                before.Restore()

        self.nocollision_place_op = {}
        self.collision_place_op = {}
        prepick = CustomStateSaver(problem_env.env)
        for obj in object_names:
            if obj in self.nocollision_pick_op:
                self.nocollision_pick_op[obj].execute()
                preplace = CustomStateSaver(problem_env.env)
                for r in self.problem_env.regions:
                    for pp in self.place_params[(obj,r)]:
                        place_op = Operator(
                            operator_type='one_arm_place',
                            discrete_parameters={'object': problem_env.env.GetKinBody(obj), 'region': problem_env.regions[r]},
                            continuous_parameters=pp
                        )
                        place_op.execute()

                        if not self.problem_env.env.CheckCollision(self.problem_env.robot) and not self.problem_env.env.CheckCollision(self.problem_env.env.GetKinBody(obj)):
                            self.nocollision_place_op[(obj,r)] = place_op
                            preplace.Restore()
                            break

                        collisions = {
                            o for o in object_names
                            if self.problem_env.env.CheckCollision(self.problem_env.env.GetKinBody(o))
                        }
                        if (obj,r) not in self.collision_place_op or len(collisions) < len(self.collision_place_op[(obj,r)][1]):
                            self.collision_place_op[(obj,r)] = place_op, collisions

                        preplace.Restore()
                    preplace.Restore()
                prepick.Restore()

        # predicates
        self.pick_in_way = PickInWay(self.problem_env)
        self.place_in_way = PlaceInWay(self.problem_env)
        self.in_region = InRegion(self.problem_env)
        self.is_holding_goal_entity = IsHoldingGoalEntity(self.problem_env, goal_entities)

        self.nodes = self.get_nodes()
        self.binary_edges = self.get_binary_edges()
        self.ternary_edges = self.get_ternary_edges()

    def get_nodes(self):
        nodes = {}
        for entity in self.problem_env.entity_names:
            nodes[entity] = self.get_node_features(entity, self.goal_entities)
        return nodes

    def get_binary_edges(self):
        edges = {}
        for a in self.problem_env.entity_names:
            for b in self.problem_env.entity_names:
                key = (a, b)
                if key not in edges.keys():
                    pick_edge_features = self.get_binary_edge_features(a, b)  # a = src, b = dest
                    edges[key] = pick_edge_features
        return edges


    def get_ternary_edges(self):
        edges = {}
        for a in self.problem_env.entity_names:
            for b in self.problem_env.entity_names:
                for r in self.problem_env.regions:
                    key = (a, b, r)
                    if r.find('region') == -1 or r.find('entire') != -1:
                        continue
                    if key not in edges.keys():
                        place_edge_features = self.get_ternary_edge_features(a, b, r)
                        edges[key] = place_edge_features
        return edges


    def get_node_features(self, entity, goal_entities):
        isobj = entity not in self.problem_env.regions
        obj = self.problem_env.env.GetKinBody(entity) if isobj else None
        pose = get_body_xytheta(obj)[0] if isobj else None

        if isobj:
            is_entity_reachable = entity in self.nocollision_pick_op
        else:
            is_entity_reachable = False

        return [
            0,  # l
            0,  # w
            0,  # h
            pose[0] if isobj else 0,  # x
            pose[1] if isobj else 0,  # y
            pose[2] if isobj else 0,  # theta
            entity not in self.problem_env.regions,  # IsObj
            entity in self.problem_env.regions,  # IsRoom
            entity in self.goal_entities,  # IsGoal
            is_entity_reachable,
            self.is_holding_goal_entity(),
        ]

    def get_ternary_edge_features(self, a, b, r):
        if a in self.problem_env.regions or b in self.problem_env.regions or r not in self.problem_env.regions:
            is_b_in_way_of_reaching_r_while_holding_a = False
        else:
            is_b_in_way_of_reaching_r_while_holding_a = a in self.nocollision_pick_op and (a,r) not in self.nocollision_place_op and (a,r) in self.collision_place_op and b in self.collision_place_op[(a,r)][1]

        return [is_b_in_way_of_reaching_r_while_holding_a]

    def get_binary_edge_features(self, a, b):
        if a in self.problem_env.regions or b not in self.problem_env.regions:
            is_place_in_b_reachable_while_holding_a = False
        else:
            is_place_in_b_reachable_while_holding_a = (a,b) in self.nocollision_place_op

        if a in self.problem_env.regions or b in self.problem_env.regions:
            is_a_in_pick_path_of_b = False
        else:
            is_a_in_pick_path_of_b = b not in self.nocollision_pick_op and b in self.collision_pick_op and a in self.collision_pick_op[b][1]

        return [
            self.in_region(a, b),
            is_a_in_pick_path_of_b,
            is_place_in_b_reachable_while_holding_a
        ]
