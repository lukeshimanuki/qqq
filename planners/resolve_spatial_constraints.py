from trajectory_representation.operator import Operator
from generators.uniform import UniformGenerator
from planners.subplanners.minimum_constraint_planner import MinimumConstraintPlanner
from planners.subplanners.minimum_constraint_goal import MinimumConstraintGoalSampler
from trajectory_representation.swept_volume import PickAndPlaceSweptVolume
from manipulation.bodies.bodies import set_color, get_color

from mover_library import utils

import numpy as np
import pdb
import time


class ResolveSpatialConstraints:
    def __init__(self, problem_env, goal_object_name, goal_region_name, misc_region_name, collides=None):
        self.objects_moved_before = []
        self.plan = []
        self.objects_in_collision = []
        self.problem_env = problem_env
        if self.problem_env.name.find('two_arm') == -1:
            raise NotImplementedError
        self.goal_object = self.problem_env.env.GetKinBody(goal_object_name)
        self.goal_region = self.problem_env.regions[goal_region_name]
        self.misc_region = self.problem_env.regions[misc_region_name]

        self.robot = self.problem_env.robot
        self.sampled_pick_configs_for_objects = {}
        self.collides = collides
        self.env = problem_env.env
        self.recursion_level = 0
        self.number_of_picks = 0
        self.number_of_places = 0
        self.number_of_nodes = 0

    def get_num_nodes(self):
        return self.number_of_nodes

    def generate_potential_pick_configs(self, operator_skeleton, n_pick_configs):
        target_object = operator_skeleton.discrete_parameters['object']
        we_already_have_pick_config = target_object.GetName() in self.sampled_pick_configs_for_objects.keys()
        if we_already_have_pick_config:
            return self.sampled_pick_configs_for_objects[target_object.GetName()]


        self.problem_env.disable_objects_in_region('entire_region')
        target_object.Enable(True)
        generator = UniformGenerator(operator_skeleton, self.problem_env, None)
        print "Generating goals for ", target_object
        param = generator.sample_next_point(operator_skeleton,
                                            n_iter=1000,
                                            n_parameters_to_try_motion_planning=n_pick_configs,
                                            dont_check_motion_existence=True,
                                            cached_collisions=self.collides)
        import pdb;pdb.set_trace()
        print "Done"
        self.problem_env.enable_objects_in_region('entire_region')
        potential_motion_plan_goals = [op['q_goal'] for op in op_cont_params if op['q_goal'] is not None]
        is_op_skel_infeasible = len(potential_motion_plan_goals) == 0
        if is_op_skel_infeasible:
            return None
        else:
            return potential_motion_plan_goals

    def generate_potential_place_configs(self, operator_skeleton, n_configs, swept_volumes):
        target_object = operator_skeleton.discrete_parameters['object']
        self.problem_env.disable_objects_in_region('entire_region')
        target_object.Enable(True)
        generator = UniformGenerator(operator_skeleton, self.problem_env,
                                     swept_volume_constraint=swept_volumes)
        print "Generating goals for ", target_object
        op_cont_params = []
        for _ in range(n_configs): # but I am allowed to return fewer than this
            param = generator.sample_next_point(operator_skeleton,
                                                n_iter=50,
                                                n_parameters_to_try_motion_planning=1,
                                                dont_check_motion_existence=True,
                                                cached_collisions=self.collides)
            op_cont_params.append(param)
        print "Done"
        self.problem_env.enable_objects_in_region('entire_region')

        potential_motion_plan_goals = [op['q_goal'] for op in op_cont_params if op['is_feasible']]
        is_op_skel_infeasible = len(potential_motion_plan_goals) == 0
        if is_op_skel_infeasible:
            return None
        else:
            return potential_motion_plan_goals

    def get_goal_config_used(self, motion_plan, potential_goal_configs):
        which_goal = np.argmin(np.linalg.norm(motion_plan[-1] - potential_goal_configs, axis=-1))
        return potential_goal_configs[which_goal]

    def plan_pick_motion_for(self, object_to_move, pick_op_instance):
        pick_op = Operator(operator_type='two_arm_pick', discrete_parameters={'object': object_to_move})
        motion_planner = MinimumConstraintPlanner(self.problem_env, object_to_move, 'rrt')
        motion, status = motion_planner.get_motion_plan(pick_op_instance.continuous_parameters['q_goal'], self.collides)
        if motion is None:
            return None, "NoSolution", None
        motion.append(pick_op_instance.continuous_parameters['q_goal'])
        pick_op.low_level_motion = motion

        pick_op.continuous_parameters = {'q_goal': motion[-1]}
        return motion, status, pick_op

    def plan_place(self, target_region, swept_volumes):
        obj_holding = self.robot.GetGrabbed()[0]
        place_op = Operator(operator_type='two_arm_place', discrete_parameters={'object': obj_holding,
                                                                                'region': target_region})
        stime = time.time()
        potential_motion_plan_goals = self.generate_potential_place_configs(place_op,
                                                                            n_pick_configs=5,
                                                                            swept_volumes=swept_volumes)
        print time.time() - stime
        import pdb;pdb.set_trace()

        if potential_motion_plan_goals is None:
            return None, "NoSolution", None
        motion, status = self.get_minimum_constraint_path_to(potential_motion_plan_goals, obj_holding)
        if motion is None:
            return None, "NoSolution", None
        place_op.low_level_motion = motion
        place_op.continuous_parameters = {'q_goal': motion[-1]}
        return motion, status, place_op

    def get_pick_from_initial_config(self, obj):
        utils.set_robot_config(self.problem_env.initial_robot_base_pose)
        pick_op = Operator(operator_type='two_arm_pick', discrete_parameters={'object': obj})
        potential_motion_plan_goals = self.generate_potential_pick_configs(pick_op, n_pick_configs=5)

        if potential_motion_plan_goals is None:
            return None, "NoSolution", None
        motion, status = self.get_minimum_constraint_path_to(potential_motion_plan_goals, obj)
        if motion is None:
            return None, "NoSolution", None
        if motion is None:
            return None, "NoSolution", None

        pick_op.low_level_motion = motion
        pick_op.continuous_parameters = {'q_goal': motion[-1]}
        return motion, status, pick_op

    def get_potential_pick_op_instance_for_retired(self, obj):
        pick_op = Operator(operator_type='two_arm_pick', discrete_parameters={'object': obj})
        pick_config_smpler = MinimumConstraintGoalSampler(self.problem_env, pick_op)
        pick_config, status = pick_config_smpler.get_motion_plan()
        pick_op.continuous_parameters = {'q_goal': pick_config}
        pick_op.low_level_motion = [pick_config]

        if status == "HasSolution":
            return pick_op
        else:
            return None

    def get_minimum_constraint_path_to(self, goal_config, target_obj):
        motion_planner = MinimumConstraintPlanner(self.problem_env, target_obj, 'rrt')
        print "Planning to goal config:", goal_config
        motion, status = motion_planner.get_motion_plan(goal_config, self.collides)
        if motion is None:
            return None, 'NoSolution'
        goal_used = self.get_goal_config_used(motion, goal_config)
        motion.append(goal_used)
        return motion, status

    def search(self, object_to_move, parent_swept_volumes, obstacles_to_remove, objects_moved_before, plan,
               parent_pick=None, parent_obj=None, stime=None, time_limit=None):
        print time.time() - stime
        if time.time() - stime > time_limit:
            return False, 'NoSolution'
        swept_volumes = PickAndPlaceSweptVolume(self.problem_env, parent_swept_volumes)
        objects_moved_before = [o for o in objects_moved_before]
        plan = [p for p in plan]

        self.problem_env.set_exception_objs_when_disabling_objects_in_region(objects_moved_before)

        self.number_of_nodes += 1
        if isinstance(object_to_move, unicode):
            object_to_move = self.problem_env.env.GetKinBody(object_to_move)
        if object_to_move == self.goal_object:
            target_region = self.goal_region
        else:
            if self.misc_region.contains(object_to_move.ComputeAABB()):
                target_region = self.misc_region
            else:
                target_region = self.goal_region

        # Debugging purpose
        color_before = get_color(object_to_move)
        set_color(object_to_move, [1, 0, 0])
        # End of debugging purpose

        # PlanGrasp
        saver = utils.CustomStateSaver(self.problem_env.env)
        stime = time.time()
        _, _, pick_operator_instance_for_curr_object = self.get_pick_from_initial_config(object_to_move) # this contains mc-path from initial config to the target obj
        print 'Time pick', time.time()-stime

        if pick_operator_instance_for_curr_object is None:
            saver.Restore()
            self.reset()
            print "Infeasible branch"
            return False, 'NoSolution'
        utils.two_arm_pick_object(object_to_move, pick_operator_instance_for_curr_object.continuous_parameters)

        # FindPlacements
        import pdb;pdb.set_trace()
        stime = time.time()
        _, _, place_operator_instance = self.plan_place(target_region, swept_volumes)
        print "Place time", time.time() - stime
        import pdb;pdb.set_trace()
        if place_operator_instance is None:
            saver.Restore()
            self.reset()
            print "Infeasible branch"
            return False, 'NoSolution'

        # O_{FUC} update
        objects_moved_before.append(object_to_move)
        self.problem_env.set_exception_objs_when_disabling_objects_in_region(objects_moved_before)

        if parent_pick is not None:
            utils.two_arm_place_object(place_operator_instance.continuous_parameters)
            stime = time.time()
            _, _, pick_operator_instance_for_parent_object = self.plan_pick_motion_for(parent_obj, parent_pick) # PlanNavigation
            print "Parent pick time", time.time()-stime
            import pdb;pdb.set_trace()
            if pick_operator_instance_for_parent_object is None:
                print "Infeasible branch"
                saver.Restore()
                return False, 'NoSolution'

            swept_volumes.add_pick_swept_volume(pick_operator_instance_for_parent_object)
            swept_volumes.add_place_swept_volume(place_operator_instance, pick_operator_instance_for_curr_object)

            plan.insert(0, pick_operator_instance_for_parent_object)
            plan.insert(0, place_operator_instance)
        else:
            pick_operator_instance_for_parent_object = None
            swept_volumes.add_place_swept_volume(place_operator_instance, pick_operator_instance_for_curr_object)
            plan.insert(0, place_operator_instance)
        saver.Restore()

        # O_{PAST}
        self.problem_env.enable_objects_in_region('entire_region')
        objs_in_collision_for_pap             \
            = swept_volumes.get_objects_in_collision_with_pick_and_place(pick_operator_instance_for_parent_object,
                                                                         place_operator_instance)

        obstacles_to_remove = objs_in_collision_for_pap + obstacles_to_remove
        # Note:
        #  For this code to be precisely HPN, I need to keep all objects that have not been moved so far in obstacles
        #  to remove. I am making the assumption that, because we are in a continuous domain, we always keep the
        #  tried-actions in the queue, and because the swept-volume heuristic tells us to move the ones in collision
        #  first, we will always try to move the first-colliding object.

        if len(obstacles_to_remove) == 0:
            objs_in_collision_for_picking_curr_obj \
                = swept_volumes.pick_swept_volume.get_objects_in_collision_with_given_op_inst(pick_operator_instance_for_curr_object)
            if len(objs_in_collision_for_picking_curr_obj) == 0:
                plan.insert(0, pick_operator_instance_for_curr_object)
                return plan, 'HasSolution'
            else:
                obstacles_to_remove += objs_in_collision_for_picking_curr_obj

        # enumerate through all object orderings
        print "Obstacles to remove", obstacles_to_remove
        """
        cbefore = []
        for oidx, o in enumerate(obstacles_to_remove):
            cbefore.append(get_color(o))
            set_color(o, [0, 0, float(oidx) / len(obstacles_to_remove)])
        [set_color(o, c) for c, o in zip(cbefore, obstacles_to_remove)]
        """

        for new_obj_to_move in obstacles_to_remove:
            set_color(object_to_move, color_before)
            tmp_obstacles_to_remove = set(obstacles_to_remove).difference(set([new_obj_to_move]))
            tmp_obstacles_to_remove = list(tmp_obstacles_to_remove)
            print "tmp obstacles to remove:", tmp_obstacles_to_remove
            branch_plan, status = self.search(new_obj_to_move,
                                              swept_volumes,
                                              tmp_obstacles_to_remove,
                                              objects_moved_before,
                                              plan,
                                              pick_operator_instance_for_curr_object,
                                              parent_obj=object_to_move, stime=stime, time_limit=time_limit)
            is_branch_success = status == 'HasSolution'
            if is_branch_success:
                return branch_plan, status

        # It should never come down here, as long as the number of nodes have not exceeded the limit
        # but to which level do I back track? To the root node. If this is a root node and
        # the number of nodes have not reached the maximum, keep searching.
        return False, 'NoSolution'

    def reset(self):
        self.problem_env.objects_to_check_collision = None
