# This class describes an operator, in terms of:
#   type, discrete parameters (represented with entity class instance), continuous parameteres,
#   and the associated low-level motions

from mover_library.utils import two_arm_pick_object, two_arm_place_object
import openravepy


class Operator:
    def __init__(self, operator_type, discrete_parameters, continuous_parameters=None):
        self.type = operator_type
        assert type(discrete_parameters) is dict, "Discrete parameters of an operator must be a dictionary"
        if self.type == 'one_arm_place':
            assert continuous_parameters is not None, "One arm place must have a grasp used to pick the object"
        self.discrete_parameters = discrete_parameters
        if continuous_parameters is None:
            self.continuous_parameters = {'is_feasible': False}
        else:
            self.continuous_parameters = continuous_parameters
        self.low_level_motion = None

    def update_low_level_motion(self, low_level_motion):
        self.low_level_motion = low_level_motion

    def set_continuous_parameters(self, continuous_parameters):
        self.continuous_parameters = continuous_parameters

    def execute(self):
        env = openravepy.RaveGetEnvironments()[0]
        if self.type == 'two_arm_pick':
            if isinstance(self.discrete_parameters['object'], openravepy.KinBody):
                obj_to_pick = self.discrete_parameters['object']
            else:
                obj_to_pick = env.GetKinBody(self.discrete_parameters['object'])
            two_arm_pick_object(obj_to_pick, self.continuous_parameters)
        elif self.type == 'two_arm_place':
            two_arm_place_object(self.continuous_parameters)
        elif self.type == 'two_arm_pick_two_arm_place':
            if isinstance(self.discrete_parameters['object'], openravepy.KinBody):
                obj_to_pick = self.discrete_parameters['object']
            else:
                obj_to_pick = env.GetKinBody(self.discrete_parameters['object'])
            two_arm_pick_object(obj_to_pick, self.continuous_parameters['pick'])
            two_arm_place_object(self.continuous_parameters['place'])
        else:
            raise NotImplementedError

    def execute_pick(self): # todo better way?
        assert self.type == 'two_arm_pick_two_arm_place'
        env = openravepy.RaveGetEnvironments()[0]
        if isinstance(self.discrete_parameters['object'], openravepy.KinBody):
            obj_to_pick = self.discrete_parameters['object']
        else:
            obj_to_pick = env.GetKinBody(self.discrete_parameters['object'])

        two_arm_pick_object(obj_to_pick, self.continuous_parameters['pick'])

    def is_discrete_parameters_eq_to(self, param):
        if self.type == 'two_arm_pick':
            if type(param) != str:
                param = param.GetName()

            my_obj = self.discrete_parameters['object']
            if type(my_obj) != str:
                my_obj = my_obj.GetName()

            return param == my_obj
        else:
            raise NotImplemented

    def merge_operators(self, operator):
        curr_op_type = self.type
        other_op_type = operator.type
        self.type = curr_op_type + "_" + other_op_type
        for k, v in zip(operator.discrete_parameters.keys(), operator.discrete_parameters.values()):
            self.discrete_parameters[other_op_type+'_'+k] = v

        for k, v in zip(operator.continuous_parameters.keys(), operator.continuous_parameters.values()):
            self.continuous_parameters[other_op_type+'_'+k] = v

        return self

    def make_pklable(self):
        if 'object' in self.discrete_parameters.keys():
            if isinstance(self.discrete_parameters['object'], openravepy.KinBody):
                self.discrete_parameters['object'] = self.discrete_parameters['object'].GetName()

        if 'region' in self.discrete_parameters.keys():
            if not isinstance(self.discrete_parameters['region'], str):
                self.discrete_parameters['region'] = self.discrete_parameters['region'].name



