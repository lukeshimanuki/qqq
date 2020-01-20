import copy
import numpy as np
import openravepy
from openravepy import DOFAffine, Environment
from mover_library.utils import grab_obj, release_obj, set_robot_config, check_collision_except, set_active_config
import numpy as np

from openravepy import DOFAffine
from problem_environments.problem_environment import ProblemEnvironment
from problem_environments.mover_problem import MoverProblem
from trajectory_representation.operator import Operator

from mover_library.utils import two_arm_pick_object, two_arm_place_object, set_robot_config, get_body_xytheta, \
    visualize_path, CustomStateSaver, set_obj_xytheta
from mover_library.operator_utils.grasp_utils import solveTwoArmIKs, compute_two_arm_grasp
from problem_environments.mover_env import Mover
from mover_library.utils import set_robot_config, set_obj_xytheta
from trajectory_representation.operator import Operator
import manipulation
from manipulation.regions import AARegion
from mover_library import utils

from manipulation.primitives.transforms import point_from_trans, quat_from_trans, trans_from_quat_point

from examples.pybullet.utils.pybullet_tools.pr2_problems import *
from examples.pybullet.utils.pybullet_tools.pr2_utils import *
from problem_environments.one_arm_mover_env import OneArmMover
import pybullet as pb
import pdb

global_bodies_dict = {}

def create_kitchen(w=.5, h=.91003, ww=.5, hh=1.136, www=.07, hhh=.2):
    floor = create_floor()

    table = create_box(w, w, h, color=(.75, .75, .75, 1))
    set_point(table, (2, 0, h/2))

    mass = 1
    #mass = 0.01
    #mass = 1e-6
    cabbage = create_box(www, www, hhh, mass=mass, color=(0, 1, 0, 1))
    #cabbage = load_model(BLOCK_URDF, fixed_base=False)
    set_point(cabbage, (2, 0, h + .10101))

    sink = create_box(w, w, h, color=(.25, .25, .75, 1))
    set_point(sink, (0, 2, h/2))

    stove = create_box(ww, ww, hh, color=(.75, .25, .25, 1))
    set_point(stove, (0, -2, hh/2))

    return table, cabbage, sink, stove

def one_arm_pb(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    table, cabbage, sink, stove = create_kitchen()

    return pr2, [stove, table, sink], [cabbage]

    #return Problem(robot=pr2, movable=[cabbage], arms=[arm], grasp_types=[grasp_type],
    #               surfaces=[table, sink, stove], sinks=[sink], stoves=[stove],
    #               goal_on=[(cabbage, stove)])

class WEnvironment:
    def __init__(self, problem_env=None):
        self.problem_env = problem_env

    def CheckCollision(self, a, b=None):
        global global_bodies_dict
        aabb = a.ComputeAABB()
        collisions = pb.getOverlappingObjects(aabb.low, aabb.high)
        if collisions is None:
            return False
        elif b is None:
            return any(global_bodies_dict[c].enabled for c in collisions if c in global_bodies_dict)
        else:
            return b.pb in collisions

    def GetRobots(self):
        return [self.problem_env.robot]

    def GetBodies(self):
        return self.problem_env.objects

    def GetRobot(self, name):
        return self.problem_env.robot

    def GetKinBody(self, name):
        if hasattr(name, 'GetName'):
            name = name.GetName()
        return self.problem_env.dobjects[name]

    def Remove(self, body):
        pass

    def drawtrimesh(self, *args, **kwargs):
        pass

global_env = WEnvironment()
openravepy.RaveGetEnvironment = lambda id: global_env
openravepy.RaveGetEnvironments = lambda: [global_env]
manipulation.primitives.utils.RaveGetEnvironments = openravepy.RaveGetEnvironments

class WAABB:
    def __init__(self, low, high):
        self.low = np.array(low)
        self.high = np.array(high)
        #self.low[2] += manipulation.constants.BODY_Z_OFFSET

    def pos(self):
        return .5 * (self.low + self.high)# + manipulation.constants.BODY_Z_OFFSET

    def extents(self):
        return .5 * (self.high - self.low)

class BodyStateSaver:
    def __init__(self, body):
        self.body = body
        self.state = pb.getBasePositionAndOrientation(body.pb)

    def Restore(self):
        pb.resetBasePositionAndOrientation(self.body.pb, *self.state)

class WBody:
    def __init__(self, body, name=None, origin=(0,0,0)):
        global global_bodies_dict
        global_bodies_dict[body] = self
        self.pb = body

        self.origin = np.array(origin)
        if name is None:
            self.name = str(body)
        else:
            self.name = name

        self.enabled = True

        self.grabbed = None

    def GetTransform(self):
        xyz, quat = pb.getBasePositionAndOrientation(self.pb)

        transform = trans_from_quat_point(quat, xyz + self.origin)
        #transform = np.zeros((4,4))
        #transform[:3,:3] = np.reshape(pb.getMatrixFromQuaternion(quat), (3,3))
        #transform[:3,3] = xyz
        #transform[3,3] = 1

        return transform

    def SetTransform(self, transform):
        if self.grabbed is not None:
            #curr_tf = self.GetTransform()
            #tf = np.multiply(np.linalg.pinv(curr_tf), transform)
            #self.grabbed.SetTransform(np.multiply(tf, self.grabbed.GetTransform()))
            pass

        xyz = point_from_trans(transform) - self.origin
        quat = quat_from_trans(transform)

        pb.resetBasePositionAndOrientation(self.pb, xyz, quat)

    def ComputeAABB(self):
        lowest, highest = pb.getAABB(self.pb)
        for idx in range(pb.getNumJoints(self.pb)):
            low, high = pb.getAABB(self.pb, idx)
            lowest = np.minimum(lowest, low)
            highest = np.maximum(highest, high)
        return WAABB(lowest, highest)

    def GetName(self):
        return self.name

    def CreateKinBodyStateSaver(self):
        return BodyStateSaver(self)

    def Enable(self, value=True):
        self.enabled = value

    def CheckSelfCollision(self):
        return False

class RobotStateSaver:
    def __init__(self, robot):
        self.robot = robot
        self.state = robot.GetDOFValues()

    def Restore(self):
        self.robot.SetDOFValues(self.state)

class WRobot(WBody):
    def __init__(self, robot):
        WBody.__init__(self, robot)

        self.active_dofs = []
        self.active_base_dofs = 0

    def __enter__(self):
        pass

    def __exit__(self, *args):
        pass

    def GetDOFValues(self, indices=None):
        if indices is None:
            indices = list(range(pb.getNumJoints(self.pb)))

        return [
            pb.getJointState(self.pb, joint)[0]
            for joint in indices
        ]

    def SetDOFValues(self, values, indices=None):
        if self.grabbed is not None:
            #curr_tf = get_manipulator_transform
            pass

        if indices is None:
            indices = list(range(pb.getNumJoints(self.pb)))

        for joint,value in zip(indices, values):
            pb.resetJointState(self.pb, joint, value)

        if self.grabbed is not None:
            #tf = np.multiply(np.linalg.pinv(curr_tf), get_manipulator_transform)
            #self.grabbed.SetTransform(np.multiply(tf, self.grabbed.GetTransform()))
            pass

    def GetGrabbed(self):
        return [self.grabbed] if self.grabbed is not None else []

    def Grab(self, body):
        if self.grabbed is not None:
            raise Exception('already holding something')

        self.grabbed = body

    def Release(self, body):
        if self.grabbed is None:
            raise Exception('not holding anything')
        elif self.grabbed.pb != body.pb:
            raise Exception('holdig other object')
        else:
            self.grabbed = None

    def CreateRobotStateSaver(self):
        return RobotStateSaver(self)

    def SetActiveDOFs(self, dofs, base_dofs, what):
        assert what == [0,0,1]

        self.active_dofs = dofs
        self.base_dofs = base_dofs

    def SetActiveDOFValues(self, values):
        dof_values = values[:len(self.active_dofs)]
        base_values = values[len(self.active_dofs):]

        self.SetDOFValues(dof_values, self.active_dofs)

        if self.base_dofs > 0:
            assert self.base_dofs == DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis
            assert len(base_values) == 3
            set_obj_xytheta(base_values, self)

    def GetActiveDOFValues(self):
        q = get_body_xytheta(self)
        return np.array(self.GetDOFValues(self.active_dofs)
            + ([q[0]] if self.base_dofs & DOFAffine.X else [])
            + ([q[1]] if self.base_dofs & DOFAffine.Y else [])
            + ([q[2]] if self.base_dofs & DOFAffine.RotationAxis else [])
        )

    def GetManipulator(self, name):
        return WManip(self, name)

    def GetActiveManipulator(self):
        return self.GetManipulator('leftarm')

    def GetDOFLimits(self, indices):
        return zip(*[pb.getJointInfo(self.pb, joint)[8:10] for joint in indices])

class WManip:
    def __init__(self, robot, name):
        self.robot = robot
        self.name = name
        if name == 'leftarm':
            self.arm_joints = get_arm_joints(robot.pb, 'left')
            self.gripper_joints = get_gripper_joints(robot.pb, 'left')
            self.tool_link = get_gripper_link(robot.pb, 'left')
        elif name == 'rightarm':
            self.arm_joints = get_arm_joints(robot.pb, 'right')
            self.gripper_joints = get_gripper_joints(robot.pb, 'right')
            self.tool_link = get_gripper_link(robot.pb, 'right')
        elif name == 'leftarm_torso':
            self.arm_joints = get_torso_arm_joints(robot.pb, 'left')
            self.gripper_joints = get_gripper_joints(robot.pb, 'left')
            self.tool_link = get_gripper_link(robot.pb, 'left')
        elif name == 'rightarm_torso':
            self.arm_joints = get_torso_arm_joints(robot.pb, 'right')
            self.gripper_joints = get_gripper_joints(robot.pb, 'right')
            self.tool_link = get_gripper_link(robot.pb, 'right')
        else:
            raise NotImplementedError

    def GetArmIndices(self):
        return self.arm_joints

    def GetGripperIndices(self):
        return self.gripper_joints

    def FindIKSolutions(self, manip_trans, filteroptions):
        return [self.FindIKSolution(manip_trans, filteroptions)]

    def FindIKSolution(self, manip_trans, filteroptions):
        pdb.set_trace()

        for i in range(20):
            iksolution = pb.calculateInverseKinematics(self.robot.pb, self.tool_link, point_from_trans(manip_trans), quat_from_trans(manip_trans))
            if filteroptions != 0:
                # check collisions
                self.robot.SetDOFValues(iksolution, indices)
                if global_env.CheckCollision(self.robot):
                    continue

            return iksolution
        return None

class PBStateSaver:
    def __init__(self):
        self.state = pb.saveState()

    def Restore(self):
        pb.restoreState(self.state)

class OneArmPB(OneArmMover):
    def __init__(self, problem_idx):
        pb.connect(pb.DIRECT)

        self.env = global_env
        self.env.problem_env = self
        #collisionChecker = openravepy.RaveCreateCollisionChecker(self.env, 'fcl_')
        #self.env.SetCollisionChecker(collisionChecker)
        self.problem_idx = problem_idx

        self.initial_placements = []
        self.placements = []
        self.robot = None
        self.objects = None
        self.curr_state = None
        self.curr_obj = None
        self.init_saver = None
        self.init_which_opreator = None
        self.v = False
        self.robot_region = None
        self.obj_region = None
        self.objs_to_move = None
        self.problem_config = None
        self.init_objs_to_move = None
        self.optimal_score = None
        self.name = None

        self.is_solving_packing = False
        self.is_solving_namo = False
        self.is_solving_fetching = False

        self.high_level_planner = None
        self.namo_planner = None
        self.fetch_planner = None
        #self.env.StopSimulation()  # openrave crashes with physics engine on
        self.motion_planner = None

        ##################

        #problem = MoverProblem(self.env)
        #self.problem_config = problem.get_problem_config()
        #self.robot = self.env.GetRobots()[0]
        #self.objects = self.problem_config['packing_boxes']
        robot, tables, objects = one_arm_pb()
        self.robot = WRobot(robot)
        self.objects = [WBody(o, origin=(0,0,-.1)) for o in objects]

        #self.set_problem_type(problem_type)
        self.set_problem_type('one_arm_mover')

        self.object_init_poses = {o.GetName(): get_body_xytheta(o).squeeze() for o in self.objects}
        self.initial_robot_base_pose = get_body_xytheta(self.robot)
        #self.regions = {'entire_region': self.problem_config['entire_region'],
        #                'home_region': self.problem_config['home_region'],
        #                'loading_region': self.problem_config['loading_region']}
        #self.region_names = ['entire_region', 'home_region', 'loading_region']
        self.regions = {}
        self.region_names = []
        self.object_names = [obj.GetName() for obj in self.objects]
        #self.placement_regions = {'home_region': self.problem_config['home_region'],
        #                          'loading_region': self.problem_config['loading_region']}
        self.placement_regions = {}

        self.entity_names = self.object_names + self.region_names
        self.entity_idx = {name: idx for idx, name in enumerate(self.entity_names)}

        self.is_init_pick_node = True
        #self.name = 'two_arm_mover'
        #self.init_saver = CustomStateSaver(self.env)
        #self.problem_config['env'] = self.env
        #self.operator_names = ['two_arm_pick', 'two_arm_place']
        self.reward_function = None
        self.applicable_op_constraint = None
        self.two_arm_pick_continuous_constraint = None
        self.two_arm_place_continuous_constraint = None
        self.objects_to_check_collision = None

        ##################

        self.operator_names = ['one_arm_pick', 'one_arm_place']
        #set_robot_config([4.19855789, 2.3236321, 5.2933337], self.robot)
        #set_obj_xytheta([3.35744004, 2.19644156, 3.52741118], self.objects[1])
        self.boxes = [WBody(b) for b in tables]
        #self.objects = self.problem_config['shelf_objects']
        #self.objects = [k for v in self.objects.values() for k in v]
        #self.objects[0], self.objects[1] = self.objects[1], self.objects[0]

        #self.target_box = self.env.GetKinBody('rectangular_packing_box1')
        self.target_box = self.boxes[0]
        #utils.randomly_place_region(self.target_box, self.regions['home_region'])
        self.target_box.name = 'rectangular_packing_box1'
        self.boxes[1].name = 'center_shelf'
        self.regions['rectangular_packing_box1_region'] = self.compute_box_region(self.target_box)
        #self.shelf_regions = self.problem_config['shelf_regions']
        self.shelf_regions = {box.GetName(): self.compute_box_region(box) for box in self.boxes[1:]}
        self.target_box_region = self.regions['rectangular_packing_box1_region']
        self.regions.update(self.shelf_regions)
        self.entity_names = [obj.GetName() for obj in self.objects] + ['rectangular_packing_box1_region',
                                                                       'center_shelf_region']
        self.name = 'one_arm_mover'
        #self.init_saver = utils.CustomStateSaver(self.env)
        self.init_saver = PBStateSaver()

        self.object_names = self.entity_names

        # fix incorrectly named regions
        self.regions = {
            region.name: region
            for region in self.regions.values()
        }

        #######

        self.dobjects = {o.GetName(): o for o in self.objects}

        margin = 3.
        minx = min(self.regions.values(), lambda region: region.box[0][0] - margin)
        maxx = min(self.regions.values(), lambda region: region.box[0][1] + margin)
        miny = min(self.regions.values(), lambda region: region.box[1][0] - margin)
        maxy = min(self.regions.values(), lambda region: region.box[1][1] + margin)
        self.regions['entire_region'] = AARegion('entire_region', ((minx, maxx), (miny, maxy)), 0, color=(0,0,0,0))
        self.regions['loading_region'] = AARegion('loading_region', ((minx, maxx), (miny, maxy)), 0, color=(0,0,0,0))
        self.regions['home_region'] = AARegion('home_region', ((minx, maxx), (miny, maxy)), 0, color=(0,0,0,0))

def get_pap_gen(problem):
	sample_pose = pr2_primitives.get_stable_gen(problem)
	sample_grasp_list = pr2_primitives.get_grasp_gen(problem)
	def sample_grasp(body):
		while True:
			grasps = sample_grasp_list(body)
			for grasp in grasps:
				yield grasp
	sample_ik = pr2_primitives.get_ik_ir_gen(problem, teleport=True)
	sample_motion = pr2_primitives.get_motion_gen(problem, teleport=True)

	def fn():
		pick_q = sample_pose(problem.robot)
		grasp = sample_grasp(body)
		ik = sample_ik(a, body, getpose(body), grasp)

	return fn

