from trajectory_representation.operator import Operator
from generators.uniform import UniformGenerator
from planners.subplanners.motion_planner import BaseMotionPlanner
from mover_library.utils import CustomStateSaver

from mover_library import utils

import numpy as np


class PlannerWithoutReachability:
    def __init__(self, problem_env, goal_object_names, goal_region):
        self.problem_env = problem_env
        self.goal_objects = [problem_env.env.GetKinBody(o) for o in goal_object_names]
        self.goal_region = self.problem_env.regions[goal_region]

    def generate_potential_pick_configs(self, operator_skeleton, n_pick_configs):
        target_object = operator_skeleton.discrete_parameters['object']
        generator = UniformGenerator(operator_skeleton, self.problem_env, None)
        print "Generating goals for ", target_object
        op_cont_params = []
        for _ in range(n_pick_configs):
            param = generator.sample_next_point(operator_skeleton,
                                                n_iter=1000,
                                                n_parameters_to_try_motion_planning=1,
                                                dont_check_motion_existence=True)
            op_cont_params.append(param)
        print "Done"
        potential_motion_plan_goals = [op['q_goal'] for op in op_cont_params if op['q_goal'] is not None]
        is_op_skel_infeasible = len(potential_motion_plan_goals) == 0
        if is_op_skel_infeasible:
            return None
        else:
            return potential_motion_plan_goals

    def generate_potential_place_configs(self, operator_skeleton, n_pick_configs):
        target_object = operator_skeleton.discrete_parameters['object']
        self.problem_env.disable_objects_in_region('entire_region')
        target_object.Enable(True)
        generator = UniformGenerator(operator_skeleton, self.problem_env, swept_volume_constraint=None)
        print "Generating goals for ", target_object
        op_cont_params = []
        for _ in range(n_pick_configs):
            param = generator.sample_next_point(operator_skeleton,
                                                n_iter=1000,
                                                n_parameters_to_try_motion_planning=1,
                                                dont_check_motion_existence=True)
            op_cont_params.append(param)
        print "Done"
        self.problem_env.enable_objects_in_region('entire_region')
        potential_motion_plan_goals = [op['q_goal'] for op in op_cont_params if op['q_goal'] is not None]
        is_op_skel_infeasible = len(potential_motion_plan_goals) == 0
        if is_op_skel_infeasible:
            return None
        else:
            return potential_motion_plan_goals

    def get_goal_config_used(self, motion_plan, potential_goal_configs):
        which_goal = np.argmin(np.linalg.norm(motion_plan[-1] - potential_goal_configs, axis=-1))
        return potential_goal_configs[which_goal]

    def find_pick(self, curr_obj):
        pick_op = Operator(operator_type='two_arm_pick', discrete_parameters={'object': curr_obj})
        potential_motion_plan_goals = self.generate_potential_pick_configs(pick_op, n_pick_configs=1)
        if potential_motion_plan_goals is None:
            return None

        pick_base_config = potential_motion_plan_goals[-1]
        """
        motion_planner = BaseMotionPlanner(problem_env=self.problem_env, algorithm='prm')
        motion, status = motion_planner.get_motion_plan(potential_motion_plan_goals)
        if status != 'HasSolution':
            return None, status

        pick_base_config = self.get_goal_config_used(motion, potential_motion_plan_goals)
        pick_op.low_level_motion = motion
        """
        pick_op.continuous_parameters = {'q_goal': pick_base_config}
        return pick_op

    def find_place(self, curr_obj):
        place_op = Operator(operator_type='two_arm_place', discrete_parameters={'object': curr_obj,
                                                                                'region': self.goal_region})
        potential_motion_plan_goals = self.generate_potential_place_configs(place_op, n_pick_configs=1)
        if potential_motion_plan_goals is None:
            return None

        place_base_config = potential_motion_plan_goals[-1]
        """
        motion_planner = BaseMotionPlanner(problem_env=self.problem_env, algorithm='prm')
        motion, status = motion_planner.get_motion_plan(potential_motion_plan_goals)
        if status != 'HasSolution':
            return None, status

        place_base_config = self.get_goal_config_used(motion, potential_motion_plan_goals)
        place_op.continuous_parameters = {'q_goal': place_base_config}
        place_op.low_level_motion = motion
        """
        place_op.continuous_parameters = {'q_goal': place_base_config}
        return place_op

    def search(self):
        # returns the order of objects that respects collision at placements

        init_state = CustomStateSaver(self.problem_env.env)
        # self.problem_env.set_exception_objs_when_disabling_objects_in_region(self.goal_objects)
        idx = 0
        plan = []
        goal_obj_move_plan = []

        while True:
            curr_obj = self.goal_objects[idx]

            self.problem_env.disable_objects_in_region('entire_region')
            print [o.IsEnabled() for o in self.problem_env.objects]
            curr_obj.Enable(True)
            pick = self.find_pick(curr_obj)
            if pick is None:
                plan = []
                goal_obj_move_plan = []
                idx += 1
                idx = idx % len(self.goal_objects)
                init_state.Restore()
                print "Pick sampling failed"
                continue
            pick.execute()

            self.problem_env.enable_objects_in_region('entire_region')
            place = self.find_place(curr_obj)
            if place is None:
                plan = []
                goal_obj_move_plan = []
                idx += 1
                idx = idx % len(self.goal_objects)
                init_state.Restore()
                print "Place sampling failed"
                continue
            place.execute()

            plan.append(pick)
            plan.append(place)
            goal_obj_move_plan.append(curr_obj)

            idx += 1
            idx = idx % len(self.goal_objects)
            print "Plan length: ", len(plan)
            if len(plan) / 2.0 == len(self.goal_objects):
                break

        init_state.Restore()
        return goal_obj_move_plan
