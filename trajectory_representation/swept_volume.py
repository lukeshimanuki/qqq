from mover_library.utils import two_arm_pick_object, two_arm_place_object, grab_obj, release_obj, set_robot_config, \
    fold_arms, CustomStateSaver
from mover_library import utils
from manipulation.bodies.bodies import get_color, set_color
import numpy as np


class SweptVolume:
    def __init__(self, problem_env, parent_swept_volume=None):
        self.problem_env = problem_env
        self.robot = self.problem_env.robot
        if parent_swept_volume is None:
            self.objects = []
            self.op_instances = []
            self.swept_volumes = []
        else:
            self.objects = [o for o in parent_swept_volume.objects]
            self.op_instances = [o for o in parent_swept_volume.op_instances]
            self.swept_volumes = [v for v in parent_swept_volume.swept_volumes]

    def add_swept_volume(self, operator_instance):
        target_object = operator_instance.discrete_parameters['object']
        if type(target_object) == str:
            target_object = self.problem_env.env.GetKinBody(target_object)
        self.objects.append(target_object)
        self.op_instances.append(operator_instance)
        self.swept_volumes.append(operator_instance.low_level_motion)

    def is_collision_in_single_volume(self, vol, obj):
        # this is asking if the new object placement will collide with previous swept volumes
        before = utils.get_body_xytheta(self.problem_env.robot)
        for config in vol:
            set_robot_config(config, self.problem_env.robot)
            if self.problem_env.env.CheckCollision(self.problem_env.robot):
                if self.problem_env.env.CheckCollision(self.problem_env.robot, obj):
                    # I don't know why, but checking collision with obj directly sometimes
                    # does not generate proper collision check result; it has to do with whether the robot is holding the
                    # object when the object is enabled.
                    utils.set_robot_config(before)
                    return True
        utils.set_robot_config(before)
        return False

    def get_objects_in_collision(self):
        raise NotImplementedError

    def reset(self):
        self.objects = []
        self.op_instances = []
        self.swept_volumes = []


class PickSweptVolume(SweptVolume):
    def __init__(self, problem_env, parent_swept_volume):
        SweptVolume.__init__(self, problem_env, parent_swept_volume)

    def is_collision_in_all_volumes(self, object_being_moved):
        state_saver = CustomStateSaver(self.problem_env.env)
        fold_arms()
        for vol in self.swept_volumes:
            if self.is_collision_in_single_volume(vol, object_being_moved):
                state_saver.Restore()
                return True
        state_saver.Restore()
        return False

    def get_objects_in_collision_with_given_op_inst(self, op_inst):
        saver = CustomStateSaver(self.problem_env.env)
        if len(self.problem_env.robot.GetGrabbed()) > 0:
            held = self.problem_env.robot.GetGrabbed()[0]
            release_obj()
        else:
            held = None

        fold_arms()
        new_cols = self.problem_env.get_objs_in_collision(op_inst.low_level_motion, 'entire_region')

        saver.Restore()
        if held is not None:
            grab_obj(held)

        return new_cols


class PlaceSweptVolume(SweptVolume):
    def __init__(self, problem_env, parent_swept_volume):
        SweptVolume.__init__(self, problem_env, parent_swept_volume)
        if parent_swept_volume is not None:
            self.pick_used = {}
            for key, val in zip(parent_swept_volume.pick_used.keys(), parent_swept_volume.pick_used.values()):
                self.pick_used[key] = val
        else:
            self.pick_used = {}

    def add_swept_volume(self, operator_instance, associated_pick):
        SweptVolume.add_swept_volume(self, operator_instance)
        self.pick_used[operator_instance] = associated_pick

    def is_collision_in_all_volumes(self, obj_being_moved):
        for op_instance in self.op_instances:
            print "Checking place collisions"
            assert len(self.problem_env.robot.GetGrabbed()) == 0
            obj_touched_before = op_instance.discrete_parameters['object']

            associated_pick = self.pick_used[op_instance]
            if type(obj_touched_before) == str:
                obj_touched_before = self.problem_env.env.GetKinBody(obj_touched_before)
            obj_touched_before.Enable(True)

            pick_param = associated_pick.continuous_parameters
            two_arm_pick_object(obj_touched_before, pick_param)
            print "Number of place swept volumes = ", len(self.op_instances)

            if self.is_collision_in_single_volume(op_instance.low_level_motion, obj_being_moved):
                two_arm_place_object(pick_param)
                return True

            two_arm_place_object(pick_param)
        return False

    def get_objects_in_collision_with_given_op_inst(self, op_inst):
        saver = CustomStateSaver(self.problem_env.env)
        if len(self.problem_env.robot.GetGrabbed()) > 0:
            held = self.problem_env.robot.GetGrabbed()[0]
            release_obj()
        else:
            held = None

        associated_pick = self.pick_used[op_inst]
        two_arm_pick_object(associated_pick.discrete_parameters['object'], associated_pick.continuous_parameters)
        new_cols = self.problem_env.get_objs_in_collision(op_inst.low_level_motion, 'entire_region')

        saver.Restore()
        if held is not None:
            grab_obj(held)

        return new_cols

    def reset(self):
        SweptVolume.reset(self)
        self.pick_used = {}


class PickAndPlaceSweptVolume:
    def __init__(self, problem_env, parent_swept_volume):
        self.problem_env = problem_env
        if parent_swept_volume is None:
            self.place_swept_volume = PlaceSweptVolume(problem_env, None)
            self.pick_swept_volume = PickSweptVolume(problem_env, None)
        else:
            self.place_swept_volume = PlaceSweptVolume(problem_env, parent_swept_volume.place_swept_volume)
            self.pick_swept_volume = PickSweptVolume(problem_env, parent_swept_volume.pick_swept_volume)

    def add_pick_swept_volume(self, pick_operator_instance):
        self.pick_swept_volume.add_swept_volume(pick_operator_instance)

    def add_place_swept_volume(self, place_operator_instance, associated_pick_operator_instance):
        self.place_swept_volume.add_swept_volume(place_operator_instance, associated_pick_operator_instance)

    def is_swept_volume_cleared(self, obj):
        saver = CustomStateSaver(self.problem_env.env)

        if len(self.problem_env.robot.GetGrabbed()) > 0:
            held = self.problem_env.robot.GetGrabbed()[0]
            release_obj()
        else:
            held = None

        collision_occurred = self.pick_swept_volume.is_collision_in_all_volumes(obj) \
                             or self.place_swept_volume.is_collision_in_all_volumes(obj)

        saver.Restore()
        if held is not None:
            grab_obj(held)

        if collision_occurred:
            return False
        else:
            return True

    def get_objects_in_collision_with_pick_and_place(self, parent_obj_pick, place):
        saver = utils.CustomStateSaver(self.problem_env.env)
        place_collisions \
            = self.place_swept_volume.get_objects_in_collision_with_given_op_inst(place)

        # place the object at the desired place
        associated_pick = self.place_swept_volume.pick_used[place]
        utils.two_arm_pick_object(associated_pick.discrete_parameters['object'], associated_pick.continuous_parameters)
        utils.two_arm_place_object(place.continuous_parameters)

        # now check the collision to the parent object
        if parent_obj_pick is not None:
            # how could I collide with obj to pick here?
            pick_collisions \
                = self.pick_swept_volume.get_objects_in_collision_with_given_op_inst(parent_obj_pick)
        else:
            pick_collisions = []
        saver.Restore()

        objs_in_collision = place_collisions + [o for o in pick_collisions if not (o in place_collisions)]
        return objs_in_collision

    def reset(self):
        self.pick_swept_volume.reset()
        self.place_swept_volume.reset()
