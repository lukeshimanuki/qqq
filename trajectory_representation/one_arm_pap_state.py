from generators.uniform import UniformGenerator
from manipulation.bodies.bodies import set_color
from mover_library import utils
from mover_library.utils import CustomStateSaver, get_body_xytheta, set_robot_config, set_obj_xytheta, visualize_path, \
    one_arm_pick_object
from pick_and_place_state import PaPState
from planners.subplanners.motion_planner import BaseMotionPlanner
from predicates.in_region import InRegion
from predicates.is_holding_goal_entity import IsHoldingGoalEntity
from predicates.pick_in_way import PickInWay
from predicates.place_in_way import PlaceInWay
from trajectory_representation.operator import Operator
from generators.one_arm_pap_uniform_generator import OneArmPaPUniformGenerator
from trajectory_representation.state import State
import pickle
import copy


class OneArmPaPState(PaPState):
    def __init__(self, problem_env, goal_entities, parent_state=None, parent_action=None, paps_used_in_data=None):
        PaPState.__init__(self, problem_env, goal_entities, parent_state=None, parent_action=None)

        self.objects = objects = {
            o: problem_env.env.GetKinBody(o)
            for o in problem_env.entity_names
            if 'region' not in o
        }
        self.regions = regions = {
            r: problem_env.regions[r]
            for r in problem_env.entity_names
            if 'region' in r
        }

        self.pick_used = {}
        self.place_used = {}

        self.parent_state = parent_state
        self.parent_ternary_predicates = {}
        self.parent_binary_predicates = {}
        if parent_state is not None:
            moved_obj_type = type(parent_action.discrete_parameters['object'])
            if moved_obj_type == str or moved_obj_type == unicode:
                moved_obj = parent_action.discrete_parameters['object']
            else:
                moved_obj = parent_action.discrete_parameters['object'].GetName()
        else:
            moved_obj = None

        self.pap_params = {}
        self.pick_params = {}
        self.place_params = {}
        problem_env.disable_objects()
        for obj in objects:
            self.pick_params[obj] = []
            for r in regions:
                print(obj, r)
                if obj in goal_entities and r in goal_entities:
                    num_tries = 1
                    num_iters = 30
                elif obj not in goal_entities and r in goal_entities:
                    num_iters = 0
                else:
                    num_tries = 1
                    num_iters = 10

                if parent_state is not None and obj != moved_obj:
                    status = 'HasSolution'
                    self.pap_params[(obj,r)]  = parent_state.pap_params[(obj, r)]
                else:
                    self.pap_params[(obj, r)] = []

                papg = OneArmPaPUniformGenerator(Operator(operator_type='one_arm_pick_one_arm_place',
                                                          discrete_parameters={
                                                              'object': problem_env.env.GetKinBody(obj),
                                                              'region': problem_env.regions[r]}), problem_env)
                for _ in range(num_iters - len(self.pap_params[(obj, r)])):
                    pick_params, place_params, status = papg.sample_next_point(num_tries)
                    if status == 'HasSolution':
                        self.pap_params[(obj, r)].append((pick_params, place_params))
                        self.pick_params[obj].append(pick_params)
                        print('success')

                    self.place_params[(obj, r)] = []
        print([o.IsEnabled() for o in problem_env.objects])
        problem_env.enable_objects()

        # problem_env.disable_objects()
        # for obj in objects:
        #    if parent_state is not None and obj != moved_obj:
        #        self.pick_params[obj] = parent_state.pick_params[obj]
        #    else:
        #        op = Operator(operator_type='one_arm_pick', discrete_parameters={'object': problem_env.env.GetKinBody(obj)})
        #        params, status = UniformGenerator(op, problem_env).sample_feasible_op_parameters(op, 300, 20)
        #        if status == 'HasSolution':
        #            self.pick_params[obj] = params
        #        else:
        #            self.pick_params[obj] = []
        # problem_env.enable_objects()

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
                    o for o in objects
                    if self.problem_env.env.CheckCollision(self.problem_env.env.GetKinBody(o))
                }
                if obj not in self.collision_pick_op or len(collisions) < len(self.collision_pick_op[obj][1]):
                    self.collision_pick_op[obj] = pick_op, collisions

                before.Restore()

        # for obj in objects:
        #    for r in regions:
        #        if obj not in self.nocollision_pick_op:
        #            self.place_params[(obj, r)] = []
        #            continue
        #        pick = self.nocollision_pick_op[obj]
        #        before = CustomStateSaver(problem_env.env)
        #        pick.execute()
        #        place_op = Operator(
        #            operator_type='one_arm_place',
        #            discrete_parameters={'object': problem_env.env.GetKinBody(obj), 'region': problem_env.regions[r]},
        #            continuous_parameters=pick.continuous_parameters
        #        )
        #        params, status = UniformGenerator(op, problem_env).sample_feasible_op_parameters(op, 300, 20)
        #        if status == 'HasSolution':
        #            self.place_params[(obj, r)] = params
        #        else:
        #            self.place_params[(obj, r)] = []
        #        before.Restore()

        self.nocollision_place_op = {}
        self.collision_place_op = {}
        before = CustomStateSaver(problem_env.env)
        for obj in objects:
            for r in regions:
                if len(self.pap_params[(obj, r)]) > 0:
                    def count_collides(papp):
                        before = CustomStateSaver(problem_env.env)
                        pickp, placep = papp

                        pick_op = Operator(
                            operator_type='one_arm_pick',
                            discrete_parameters={'object': problem_env.env.GetKinBody(obj)},
                            continuous_parameters=pickp
                        )
                        place_op = Operator(
                            operator_type='one_arm_place',
                            discrete_parameters={'object': problem_env.env.GetKinBody(obj),
                                                 'region': problem_env.regions[r]},
                            continuous_parameters=placep
                        )

                        pick_op.execute()
                        pick_collisions = sum(problem_env.env.CheckCollision(o) for o in problem_env.objects)
                        place_op.execute()
                        place_collisions = sum(problem_env.env.CheckCollision(o) for o in problem_env.objects)

                        before.Restore()
                        return pick_collisions + place_collisions
                    papp = min(self.pap_params[(obj, r)], key=lambda papp: count_collides(papp))
                    pickp, placep = papp
                    pick_op = Operator(
                        operator_type='one_arm_pick',
                        discrete_parameters={'object': problem_env.env.GetKinBody(obj)},
                        continuous_parameters=pickp
                    )
                    pick_op.execute()
                    if self.problem_env.env.CheckCollision(
                            self.problem_env.robot) or self.problem_env.env.CheckCollision(
                            self.problem_env.env.GetKinBody(obj)):
                        before.Restore()
                        continue
                    place_op = Operator(
                        operator_type='one_arm_place',
                        discrete_parameters={'object': problem_env.env.GetKinBody(obj),
                                             'region': problem_env.regions[r]},
                        continuous_parameters=placep
                    )
                    place_op.execute()

                    if not self.problem_env.env.CheckCollision(
                            self.problem_env.robot) and not self.problem_env.env.CheckCollision(
                            self.problem_env.env.GetKinBody(obj)):
                        self.nocollision_place_op[(obj, r)] = pick_op, place_op
                        before.Restore()
                        continue

                    collisions = {
                        o for o in objects
                        if self.problem_env.env.CheckCollision(self.problem_env.env.GetKinBody(o))
                    }
                    if (obj, r) not in self.collision_place_op or len(collisions) < len(
                            self.collision_place_op[(obj, r)][1]):
                        self.collision_place_op[(obj, r)] = place_op, collisions

                    before.Restore()

        print(
        sum([o in self.collision_pick_op for o in objects]), sum([o in self.nocollision_pick_op for o in objects]),
        sum([(o, r) in self.collision_place_op for o in objects for r in regions]),
        sum([(o, r) in self.nocollision_place_op for o in objects for r in regions]))

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
                for r in self.problem_env.entity_names:
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
            is_b_in_way_of_reaching_r_while_holding_a = a in self.nocollision_pick_op and (
            a, r) not in self.nocollision_place_op and (a, r) in self.collision_place_op and b in \
                                                        self.collision_place_op[(a, r)][1]

        return [is_b_in_way_of_reaching_r_while_holding_a]

    def get_binary_edge_features(self, a, b):
        if a in self.problem_env.regions or b not in self.problem_env.regions:
            is_place_in_b_reachable_while_holding_a = False
        else:
            is_place_in_b_reachable_while_holding_a = (a, b) in self.nocollision_place_op

        if a in self.problem_env.regions or b in self.problem_env.regions:
            is_a_in_pick_path_of_b = False
        else:
            is_a_in_pick_path_of_b = b not in self.nocollision_pick_op and b in self.collision_pick_op and a in \
                                     self.collision_pick_op[b][1]

        return [
            self.in_region(a, b),
            is_a_in_pick_path_of_b,
            is_place_in_b_reachable_while_holding_a
        ]

    def make_pklable(self):
        PaPState.make_pklable(self)
        self.objects = None

    def make_plannable(self, problem_env):
        PaPState.make_plannable(self, problem_env)
        self.objects = {
            o: problem_env.env.GetKinBody(o)
            for o in problem_env.entity_names
            if 'region' not in o
        }

