from generators.uniform import UniformGenerator
from manipulation.bodies.bodies import set_color
from mover_library import utils
from mover_library.utils import CustomStateSaver, get_body_xytheta, set_robot_config, set_obj_xytheta, visualize_path, \
    one_arm_pick_object, set_point
from pick_and_place_state import PaPState
from planners.subplanners.motion_planner import BaseMotionPlanner
from predicates.in_region import InRegion
from predicates.is_holding_goal_entity import IsHoldingGoalEntity
from predicates.pick_in_way import PickInWay
from predicates.place_in_way import PlaceInWay
from trajectory_representation.operator import Operator
from generators.one_arm_pap_uniform_generator import OneArmPaPUniformGenerator
from generators.feasibility_checkers.one_arm_pick_feasibility_checker import OneArmPickFeasibilityChecker
from trajectory_representation.state import State
import numpy as np
import pickle
import copy
import os


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
        self.object_names = [o for o in problem_env.entity_names if 'region' not in o]
        self.region_names = [o for o in problem_env.entity_names if 'region' in o]
        # cache ik solutions
        ikcachename = './ikcache.pkl'
        self.iksolutions = {}
        if parent_state is not None:
            self.iksolutions = parent_state.iksolutions
        elif os.path.isfile(ikcachename):
            self.iksolutions = pickle.load(open(ikcachename, 'r'))
        else:
            self.compute_and_cache_ik_solutions(ikcachename)

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
        self.initialize_pap_pick_place_params(moved_obj, parent_state)

        self.nocollision_pick_op = {}
        self.collision_pick_op = {}
        self.determine_collision_and_collision_free_picks()

        self.nocollision_place_op = {}
        self.collision_place_op = {}
        self.determine_collision_and_collision_free_places()

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

    def compute_and_cache_ik_solutions(self, ikcachename):
        before = CustomStateSaver(self.problem_env.env)
        utils.open_gripper()

        for o in self.problem_env.env.GetBodies()[2:]:
            o.Enable(False)
        self.iksolutions = {}
        for o, obj in self.objects.items():
            print(o)
            self.iksolutions[o] = {r: [] for r in self.regions}
            pick_op = Operator(operator_type='one_arm_pick', discrete_parameters={'object': obj})
            pick_generator = UniformGenerator(pick_op, self.problem_env)
            pick_feasibility_checker = OneArmPickFeasibilityChecker(self.problem_env)
            for _ in range(10000):
                pick_params = pick_generator.sample_from_uniform()
                iks = []
                for r, region in self.regions.items():
                    place_op = Operator(operator_type='one_arm_place',
                                        discrete_parameters={
                                            'object': obj,
                                            'region': region})
                    place_generator = UniformGenerator(place_op, self.problem_env)

                    obj_pose = place_generator.sample_from_uniform()
                    set_obj_xytheta(obj_pose, obj)
                    set_point(obj, np.hstack([obj_pose[0:2], region.z + 0.001]))

                    params, status = pick_feasibility_checker.check_feasibility(pick_op, pick_params)

                    if status == 'HasSolution':
                        iks.append((obj.GetTransform(), params))
                    else:
                        break

                if len(iks) == len(self.regions):
                    for r, ik in zip(self.regions, iks):
                        self.iksolutions[o][r].append(ik)

                if all(len(self.iksolutions[o][r]) >= 1000 for r in self.regions):
                    break

            print([len(self.iksolutions[o][r]) for r in self.regions])

        for o in self.problem_env.env.GetBodies()[2:]:
            o.Enable(True)

        before.Restore()

        pickle.dump(self.iksolutions, open(ikcachename, 'w'))

        import pdb;
        pdb.set_trace()

    def determine_collision_and_collision_free_places(self):
        before = CustomStateSaver(self.problem_env.env)
        for obj in self.objects:
            for r in self.regions:
                if len(self.pap_params[(obj, r)]) > 0:
                    def count_collides(papp):
                        before = CustomStateSaver(self.problem_env.env)
                        pickp, placep = papp

                        pick_op = Operator(
                            operator_type='one_arm_pick',
                            discrete_parameters={'object': self.problem_env.env.GetKinBody(obj)},
                            continuous_parameters=pickp
                        )
                        place_op = Operator(
                            operator_type='one_arm_place',
                            discrete_parameters={'object': self.problem_env.env.GetKinBody(obj),
                                                 'region': self.problem_env.regions[r]},
                            continuous_parameters=placep
                        )

                        pick_op.execute()
                        pick_collisions = sum(self.problem_env.env.CheckCollision(o) for o in self.problem_env.objects)
                        place_op.execute()
                        place_collisions = sum(self.problem_env.env.CheckCollision(o) for o in self.problem_env.objects)

                        before.Restore()
                        return pick_collisions + place_collisions

                    # chooses the one with minimal collisions
                    papp = min(self.pap_params[(obj, r)], key=lambda papp: count_collides(papp))
                    pickp, placep = papp
                    pick_op = Operator(
                        operator_type='one_arm_pick',
                        discrete_parameters={'object': self.problem_env.env.GetKinBody(obj)},
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
                        discrete_parameters={'object': self.problem_env.env.GetKinBody(obj),
                                             'region': self.problem_env.regions[r]},
                        continuous_parameters=placep
                    )
                    place_op.execute()

                    if not self.problem_env.env.CheckCollision(self.problem_env.robot) \
                            and not self.problem_env.env.CheckCollision(self.problem_env.env.GetKinBody(obj)):
                        self.nocollision_place_op[(obj, r)] = pick_op, place_op
                        if obj in self.goal_entities and r in self.goal_entities:
                            print('successful goal pap')
                        before.Restore()
                        continue

                    collisions = {
                        o for o in self.objects
                        if self.problem_env.env.CheckCollision(self.problem_env.env.GetKinBody(o))
                    }
                    if (obj, r) not in self.collision_place_op or len(collisions) < len(
                            self.collision_place_op[(obj, r)][1]):
                        self.collision_place_op[(obj, r)] = place_op, collisions

                    before.Restore()

    def determine_collision_and_collision_free_picks(self):
        for obj, params in self.pick_params.items():
            for pp in params:
                pick_op = Operator(
                    operator_type='one_arm_pick',
                    discrete_parameters={'object': self.problem_env.env.GetKinBody(obj)},
                    continuous_parameters=pp
                )
                before = CustomStateSaver(self.problem_env.env)
                pick_op.execute()
                if not self.problem_env.env.CheckCollision(self.problem_env.robot):
                    self.nocollision_pick_op[obj] = pick_op
                    before.Restore()
                    break

                collisions = {
                    o for o in self.objects
                    if self.problem_env.env.CheckCollision(self.problem_env.env.GetKinBody(o))
                }
                if obj not in self.collision_pick_op or len(collisions) < len(self.collision_pick_op[obj][1]):
                    self.collision_pick_op[obj] = pick_op, collisions

                before.Restore()

    def initialize_pap_pick_place_params(self, moved_obj, parent_state):
        self.problem_env.disable_objects()
        for obj in self.objects:
            self.pick_params[obj] = []
            for r in self.regions:
                print(obj, r)

                current_region = self.problem_env.get_region_containing(obj).name

                if obj in self.goal_entities and r in self.goal_entities:
                    num_tries = 3
                    num_iters = 300
                elif obj not in self.goal_entities and r in self.goal_entities:
                    num_iters = 0
                else:
                    num_tries = 10
                    num_iters = 30

                if self.parent_state is not None and obj != moved_obj:
                    self.pap_params[(obj, r)] = parent_state.pap_params[(obj, r)]
                else:
                    self.pap_params[(obj, r)] = []

                op_skel = Operator(operator_type='one_arm_pick_one_arm_place',
                                   discrete_parameters={'object': self.problem_env.env.GetKinBody(obj), 'region': self.problem_env.regions[r]})
                papg = OneArmPaPUniformGenerator(op_skel,
                                                 self.problem_env,
                                                 cached_picks=(self.iksolutions[obj][current_region], self.iksolutions[obj][r]))

                # I think num_iters is the number of paps for each object
                for _ in range(num_iters - len(self.pap_params[(obj, r)])):
                    pick_params, place_params, status = papg.sample_next_point(num_tries)
                    if status == 'HasSolution':
                        self.pap_params[(obj, r)].append((pick_params, place_params))
                        self.pick_params[obj].append(pick_params)
                        print('success')

                    self.place_params[(obj, r)] = []
                #if obj in self.goal_entities and r in self.goal_entities:
                #    print self.pap_params[(obj, r)]
        self.problem_env.enable_objects()

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

        is_entity_reachable = True
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
            """
            is_b_in_way_of_reaching_r_while_holding_a = False
                                                        a in self.nocollision_pick_op \
                                                        and (a, r) not in self.nocollision_place_op \
                                                        and (a, r) in self.collision_place_op and \
                                                        b in self.collision_place_op[(a, r)][1]
            """
        is_b_in_way_of_reaching_r_while_holding_a = False

        return [is_b_in_way_of_reaching_r_while_holding_a]

    def get_binary_edge_features(self, a, b):
        if a in self.problem_env.regions or b not in self.problem_env.regions:
            is_place_in_b_reachable_while_holding_a = False
        else:
            if 'region' in b and 'region' not in a:
                obj_a = self.problem_env.env.GetKinBody(a)
                if self.problem_env.regions[b].contains(obj_a.ComputeAABB()):
                    is_place_in_b_reachable_while_holding_a = True
                else:
                    is_place_in_b_reachable_while_holding_a = True  #(a, b) in self.nocollision_place_op

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
        for k in self.collision_pick_op.values(): k[0].make_pklable()
        for k in self.nocollision_pick_op.values(): k.make_pklable()
        for k in self.collision_place_op.values(): k[0].make_pklable()
        for k in self.nocollision_place_op.values():
            k[0].make_pklable()
            k[1].make_pklable()


        self.objects = None

    def make_plannable(self, problem_env):
        PaPState.make_plannable(self, problem_env)
        self.objects = {
            o: problem_env.env.GetKinBody(o)
            for o in problem_env.entity_names
            if 'region' not in o
        }
