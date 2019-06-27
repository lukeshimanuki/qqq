from problem_environments.reward_functions.reward_function import RewardFunction
from mover_library.utils import two_arm_pick_object, two_arm_place_object, one_arm_place_object, one_arm_pick_object
from manipulation.bodies.bodies import set_color
import numpy as np


class ObjectPackingRewardFunction(RewardFunction):
    def __init__(self, problem_env, goal_objects, goal_region):
        RewardFunction.__init__(self, problem_env)
        self.goal_objects = [self.problem_env.env.GetKinBody(obj_name) for obj_name in goal_objects]
        self.goal_region = self.problem_env.regions[goal_region]
        #set_color(self.goal_object, [1, 0, 0])

    def apply_operator_instance_and_get_reward(self, operator_instance, is_op_feasible):
        if not is_op_feasible:
            return self.infeasible_reward
        else:
            obj = operator_instance.discrete_parameters['object']
            if isinstance(obj, str) or isinstance(obj, unicode):
                obj = self.problem_env.env.GetKinBody(obj)
            prev_region = self.problem_env.get_region_containing(obj)
            operator_instance.execute()
            return self.is_one_of_entities_in_goal_region(obj, prev_region) * 1

    def is_one_of_entities_in_goal_region(self, entity, prev_region=None):
        is_goal_entity = entity in self.goal_objects
        if is_goal_entity and self.goal_region.contains(entity.ComputeAABB()):
            if prev_region is not None and prev_region != self.goal_region:
                return True
            elif prev_region is None:
                return True
        return False

    def apply_operator_skeleton_and_get_reward(self, operator_instance):
        return 0

    def is_goal_reached(self):
        return np.all([self.is_one_of_entities_in_goal_region(obj) for obj in self.goal_objects])

    def is_optimal_plan_found(self, best_traj_rwd):
        return True # satisficing problem
