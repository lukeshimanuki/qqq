from generator import Generator
import numpy as np
import time

from mover_library.utils import get_pick_base_pose_and_grasp_from_pick_parameters
from mover_library import utils
from generators.uniform import UniformGenerator
from trajectory_representation.operator import Operator
from mover_library.operator_utils import grasp_utils

from feasibility_checkers.one_arm_pick_feasibility_checker import OneArmPickFeasibilityChecker
from feasibility_checkers.one_arm_place_feasibility_checker import OneArmPlaceFeasibilityChecker


class OneArmPaPUniformGenerator:
    def __init__(self, operator_skeleton, problem_env, swept_volume_constraint=None):
        self.problem_env = problem_env
        target_region = None
        if 'region' in operator_skeleton.discrete_parameters:
            target_region = operator_skeleton.discrete_parameters['region']
            if type(target_region) == str:
                target_region = self.problem_env.regions[target_region]
        target_obj = operator_skeleton.discrete_parameters['object']
        self.robot = problem_env.robot
        self.target_region = target_region
        self.target_obj = target_obj
        self.swept_volume_constraint = swept_volume_constraint

        self.pick_op = Operator(operator_type='one_arm_pick',
                                discrete_parameters={'object': target_obj})
        self.pick_generator = UniformGenerator(self.pick_op, problem_env)

        self.place_op = Operator(operator_type='one_arm_place',
                                 discrete_parameters={'object': target_obj, 'region': target_region},
                                 continuous_parameters={})
        self.place_generator = UniformGenerator(self.place_op, problem_env)

        self.pick_feasibility_checker = OneArmPickFeasibilityChecker(problem_env)
        self.place_feasibility_checker = OneArmPlaceFeasibilityChecker(problem_env)
        self.operator_skeleton = operator_skeleton

    def sample_next_point(self, max_ik_attempts):
        # n_iter refers to the max number of IK attempts on pick
        n_ik_attempts = 0
        while True:
            pick_cont_params, place_cont_params, status = self.sample_cont_params()
            if status == 'InfeasibleIK':
                n_ik_attempts += 1
                if n_ik_attempts == max_ik_attempts:
                    break
            elif status == 'InfeasibleBase':
                return None, None, "NoSolution"
            elif status == 'HasSolution':
                return pick_cont_params, place_cont_params, 'HasSolution'
        return None, None, 'NoSolution'

    def is_base_feasible(self, base_pose):
        utils.set_robot_config(base_pose, self.robot)
        inside_region = self.problem_env.regions['home_region'].contains(self.robot.ComputeAABB()) or \
                        self.problem_env.regions['loading_region'].contains(self.robot.ComputeAABB())
        no_collision = not self.problem_env.env.CheckCollision(self.robot)
        if (not inside_region) or (not no_collision):
            return False
        else:
            return True

    def sample_pick_cont_parameters(self):
        op_parameters = self.pick_generator.sample_from_uniform()
        grasp_params, pick_base_pose = get_pick_base_pose_and_grasp_from_pick_parameters(self.target_obj, op_parameters)
        if not self.is_base_feasible(pick_base_pose):
            return None, 'InfeasibleBase'

        utils.open_gripper()
        grasps = grasp_utils.compute_one_arm_grasp(depth_portion=grasp_params[2],
                                                   height_portion=grasp_params[1],
                                                   theta=grasp_params[0],
                                                   obj=self.target_obj,
                                                   robot=self.robot)
        grasp_config, grasp = grasp_utils.solveIKs(self.problem_env.env, self.robot, grasps)

        param = {'q_goal': np.hstack([grasp_config, pick_base_pose]),
                 'grasp_params': grasp_params,
                 'g_config': grasp_config,
                 'action_parameters': op_parameters}

        if grasp_config is None:
            return None, 'InfeasibleIK'
        else:
            return param, 'HasSolution'

    def sample_place_cont_parameters(self, pick_params):
        obj_place_pose = self.place_generator.sample_from_uniform()
        self.place_op.continuous_parameters['grasp_params'] = pick_params['grasp_params']
        cont_params, status = self.place_feasibility_checker.check_feasibility(self.place_op, obj_place_pose,
                                                                               self.swept_volume_constraint)
        if status != 'HasSolution':
            return None, status
        else:
            return cont_params, status

    def sample_cont_params(self):
        # todo refactor this function
        robot_pose = utils.get_body_xytheta(self.robot)
        robot_config = self.robot.GetDOFValues()

        assert len(self.robot.GetGrabbed()) == 0

        # sample pick
        pick_cont_params = None
        n_ik_attempts = 0
        n_base_attempts = 0
        status = "NoSolution"
        while pick_cont_params is None:
            pick_cont_params, status = self.sample_pick_cont_parameters()
            if status == 'InfeasibleBase':
                n_base_attempts += 1
            if status == 'InfeasibleIK':
                n_ik_attempts += 1
            if status == 'HasSolution':
                n_ik_attempts += 1
                break
            if n_ik_attempts == 1 or n_base_attempts == 500:
                break
        if status != 'HasSolution':
            utils.set_robot_config(robot_pose)
            return None, None, status

        print "Pick succeeded, sampling place..."
        self.pick_op.continuous_parameters = pick_cont_params
        self.pick_op.execute()

        # sample place
        n_ik_attempts = 0
        n_base_attempts = 0
        status = "NoSolution"
        place_cont_params = None
        while place_cont_params is None:
            place_cont_params, status = self.sample_place_cont_parameters(pick_cont_params)
            if status == 'InfeasibleBase':
                n_base_attempts += 1
            if status == 'InfeasibleIK':
                n_ik_attempts += 1

            if status == 'HasSolution':
                n_ik_attempts += 1
                break
            if n_ik_attempts == 1 or n_base_attempts == 500:
                break
        utils.one_arm_place_object(pick_cont_params)
        self.robot.SetDOFValues(robot_config)
        utils.set_robot_config(robot_pose)
        if status != 'HasSolution':
            print "place sampling failed"
            return None, None, status
        else:
            self.place_op.continuous_parameters = place_cont_params
            return pick_cont_params, place_cont_params, status
