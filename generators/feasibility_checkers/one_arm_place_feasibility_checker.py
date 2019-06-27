from mover_library.samplers import *
from mover_library.utils import set_robot_config, grab_obj, release_obj, set_config
from generators.feasibility_checkers.place_feasibility_checker import PlaceFeasibilityChecker
from generators.feasibility_checkers.one_arm_pick_feasibility_checker import OneArmPickFeasibilityChecker

from manipulation.primitives.utils import mirror_arm_config
from manipulation.constants import PARALLEL_LEFT_ARM, REST_LEFT_ARM, HOLDING_LEFT_ARM, FOLDED_LEFT_ARM, \
    FAR_HOLDING_LEFT_ARM, LOWER_TOP_HOLDING_LEFT_ARM, REGION_Z_OFFSET


class OneArmPlaceFeasibilityChecker(PlaceFeasibilityChecker, OneArmPickFeasibilityChecker):
    def __init__(self, problem_env):
        PlaceFeasibilityChecker.__init__(self, problem_env)
        """
        self.problem_env = problem_env
        self.env = problem_env.env
        self.robot = self.env.GetRobots()[0]
        self.robot_region = self.problem_env.regions['entire_region']
        self.objects_to_check_collision = []
        """

    def place_object_and_robot_at_new_pose(self, obj, obj_pose, obj_region):
        T_r_wrt_o = np.dot(np.linalg.inv(obj.GetTransform()), self.robot.GetTransform())
        release_obj()
        set_obj_xytheta(obj_pose, obj)
        new_T_robot = np.dot(obj.GetTransform(), T_r_wrt_o)
        self.robot.SetTransform(new_T_robot)
        new_base_pose = get_body_xytheta(self.robot)
        set_robot_config(new_base_pose, self.robot)
        fold_arms()
        set_point(obj, np.hstack([obj_pose[0:2], obj_region.z]))
        return new_base_pose

    def check_feasibility(self, operator_skeleton, place_parameters, swept_volume_to_avoid=None):
        obj = self.robot.GetGrabbed()[0]
        obj_pose = place_parameters
        before = CustomStateSaver(self.problem_env.env)
        grasp_params = operator_skeleton.continuous_parameters['grasp_params']

        obj_region = operator_skeleton.discrete_parameters['region']
        if type(obj_region) == str:
            obj_region = self.problem_env.regions[obj_region]
        new_base_pose = self.place_object_and_robot_at_new_pose(obj, obj_pose, obj_region)

        target_robot_region = self.problem_env.regions['entire_region']
        target_obj_region = obj_region
        is_base_pose_infeasible = self.env.CheckCollision(self.robot) or \
                                  (not target_robot_region.contains(self.robot.ComputeAABB()))
        is_object_pose_infeasible = self.env.CheckCollision(obj) or \
                                    (not target_obj_region.contains(obj.ComputeAABB()))

        if is_base_pose_infeasible or is_object_pose_infeasible:
            action = {'operator_name': 'one_arm_place', 'q_goal': None, 'base_pose': None, 'object_pose': None,
                      'action_parameters': place_parameters, 'grasp_params': grasp_params}
            before.Restore()
            return action, 'NoSolution'

        grasp_config = OneArmPickFeasibilityChecker.compute_grasp_config(self, obj, new_base_pose, grasp_params)
        if grasp_config is None:
            action = {'operator_name': 'one_arm_place', 'base_pose': None, 'object_pose': None,
                      'q_goal': None,
                      'action_parameters': place_parameters, 'g_config': grasp_config, 'grasp_params': grasp_params}
            before.Restore()
            return action, 'NoSolution'
        else:
            before.Restore()
            grasp_config = grasp_config.squeeze()
            new_base_pose = new_base_pose.squeeze()
            action = {'operator_name': 'one_arm_place', 'q_goal': np.hstack([grasp_config, new_base_pose]),
                      'base_pose': new_base_pose, 'object_pose': place_parameters,
                      'action_parameters': place_parameters, 'g_config': grasp_config, 'grasp_params': grasp_params}
            return action, 'HasSolution'
