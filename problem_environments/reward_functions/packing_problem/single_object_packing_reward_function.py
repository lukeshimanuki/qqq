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
            operator_instance.execute()
            """
            if operator_instance.type == 'two_arm_pick':
                two_arm_pick_object(operator_instance.discrete_parameters['object'], operator_instance.continuous_parameters)
            elif operator_instance.type == 'two_arm_place':
                object_held = self.problem_env.robot.GetGrabbed()[0]
                two_arm_place_object(operator_instance.continuous_parameters)
            elif operator_instance.type == 'one_arm_pick':
                one_arm_pick_object(operator_instance.discrete_parameters['object'],
                                    operator_instance.continuous_parameters)
            elif operator_instance.type == 'one_arm_place':
                one_arm_place_object(operator_instance.continuous_parameters)
            elif operator_instance.type == 'two_arm_pick_two_arm_place':
                operator_instance.execute()
            else:
                raise NotImplementedError
            """
            return self.is_one_of_entities_in_goal_region(operator_instance.discrete_parameters['object']) * 1

    def is_one_of_entities_in_goal_region(self, entity):
        is_goal_entity = entity in self.goal_objects
        if is_goal_entity and self.goal_region.contains(entity.ComputeAABB()):
            return True
        else:
            return False

    def apply_operator_skeleton_and_get_reward(self, operator_instance):
        return 0

    def is_goal_reached(self):
        return np.all([self.is_one_of_entities_in_goal_region(obj) for obj in self.goal_objects])

    def is_optimal_plan_found(self, best_traj_rwd):
        return True # satisficing problem
