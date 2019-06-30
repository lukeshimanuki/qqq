from problem_environments.reward_functions.reward_function import RewardFunction
from mover_library.utils import two_arm_pick_object, two_arm_place_object, one_arm_place_object, one_arm_pick_object
from manipulation.bodies.bodies import set_color
import numpy as np


class ObjectPackingRewardFunction(RewardFunction):
    def __init__(self, problem_env, goal_objects, goal_region):
        RewardFunction.__init__(self, problem_env)
        self.goal_objects = [self.problem_env.env.GetKinBody(obj_name) for obj_name in goal_objects]
        self.goal_region = self.problem_env.regions[goal_region]
        self.achieved = []
        self.infeasible_reward = -1
        # set_color(self.goal_object, [1, 0, 0])

    def apply_operator_instance_and_get_reward(self, state, operator_instance, is_op_feasible):
        if not is_op_feasible:
            return self.infeasible_reward
        else:
            obj = operator_instance.discrete_parameters['object']
            if isinstance(obj, str) or isinstance(obj, unicode):
                obj = self.problem_env.env.GetKinBody(obj)
            operator_instance.execute()
            return self.get_reward_of_curr_scene(obj)

    def n_cleared_obstacles_to_goal(self, state):
        if state is None:
            return 0
        if state.parent_state is not None:
            objs_in_way = state.get_entities_in_way_to_goal_entities()
            parent_objs_in_way = state.parent_state.get_entities_in_way_to_goal_entities()
            if len(parent_objs_in_way) - len(objs_in_way) > 0:
                return len(parent_objs_in_way) - len(objs_in_way)
        return 0

    def apply_operator_skeleton_and_get_reward(self, state, operator_instance):
        return self.n_cleared_obstacles_to_goal(state)

    def get_reward_of_curr_scene(self, entity):
        is_goal_entity = entity in self.goal_objects
        if is_goal_entity and self.goal_region.contains(entity.ComputeAABB()):
            if (entity, self.goal_region) in self.achieved:
                return 0
            else:
                self.achieved.append((entity, self.goal_region))
                return 10

        return 0

    def is_goal_reached(self):
        for obj in self.goal_objects:
            if not (obj, self.goal_region) in self.achieved:
                return False
        return True
        #return np.all([self.is_one_of_entities_in_goal_region(obj) for obj in self.goal_objects])

    def is_optimal_plan_found(self, best_traj_rwd):
        return True  # satisficing problem
