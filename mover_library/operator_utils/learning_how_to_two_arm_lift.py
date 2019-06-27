from manipulation.constants import PARALLEL_LEFT_ARM, REST_LEFT_ARM, HOLDING_LEFT_ARM, FOLDED_LEFT_ARM, FAR_HOLDING_LEFT_ARM, LOWER_TOP_HOLDING_LEFT_ARM,REGION_Z_OFFSET
from manipulation.regions import create_region, AARegion
from manipulation.bodies.bodies import randomly_place_region, place_body,sphere_body
from manipulation.inverse_reachability.inverse_reachability import ir_base_trans
from manipulation.primitives.inverse_kinematics import inverse_kinematics_helper
                                                        
from manipulation.primitives.savers import DynamicEnvironmentStateSaver
from itertools import product
from manipulation.primitives.transforms import trans_from_base_values, set_pose, set_quat, \
  point_from_pose, axis_angle_from_rot, rot_from_quat, quat_from_pose, quat_from_z_rot,\
  get_pose,base_values_from_pose,pose_from_base_values
from math import pi
#from tools.utils import randomize
from time import time
import numpy as np
import copy

from manipulation.bodies.bodies import geometry_hash
from manipulation.bodies.bounding_volumes import aabb_from_body
from manipulation.grasps.grasps import save_grasp_database, Grasp

from manipulation.primitives.transforms import trans_from_base_values, set_pose, set_quat, \
                                               point_from_pose, axis_angle_from_rot, \
                                               rot_from_quat, quat_from_pose, quat_from_z_rot,\
                                               get_pose,base_values_from_pose,\
                                               pose_from_base_values,get_point


from manipulation.bodies.bodies import *
from manipulation.bodies.robot import open_gripper, get_manip_trans

from math import pi,cos,sin
from openravepy import RaveSetDebugLevel, DebugLevel, Environment, RaveDestroy
from openravepy.misc import SetViewerUserThread
from collections import deque
from random import sample as random_sample
#from tamp.learning_tamp import *
from random import random
from manipulation.primitives.utils import get_env,mirror_arm_config
from manipulation.primitives.transforms import *
from openravepy import *
from manipulation.bodies.robot import open_gripper, get_manip_trans
from manipulation.bodies.robot import manip_from_pose_grasp
from manipulation.bodies.robot import get_active_arm_indices, get_manipulator


import copy

env=Environment()
ENV_FILENAME = './just_robot.xml'
env.SetViewer('qtcoin')
env.Load(ENV_FILENAME)
robot = env.GetRobots()[0]

# define object and place it (0,0,0)


# define robot 
robot.SetActiveManipulator('leftarm')
manip = robot.GetActiveManipulator()
ee    = manip.GetEndEffector()
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot, \
                              iktype=IkParameterization.Type.Transform6D, \
                              forceikfast=True, freeindices=None, \
                              freejoints=None, manip=None)
if not ikmodel.load():
  ikmodel.autogenerate()

robot.SetActiveManipulator('rightarm_torso')
manip = robot.GetActiveManipulator()
ee    = manip.GetEndEffector()
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot, \
                              iktype=IkParameterization.Type.Transform6D, \
                              forceikfast=True, freeindices=None, \
                              freejoints=None, manip=None)
if not ikmodel.load():
  ikmodel.autogenerate()



#robot.SetDOFValues(np.array([0.54800022]), robot.GetActiveManipulator().GetGripperIndices())

### Notations
# w = world
# o = obj
# ee = end-effector

def compute_tool_trans_wrt_obj_trans(tool_trans_wrt_world,object_trans):
  return np.linalg.solve(object_trans, tool_trans_wrt_world)

def compute_Tee_at_given_Ttool( tool_trans_wrt_world,tool_trans_wrt_ee ):
  # computes the endeffector transform at the given tool transform
  return np.dot(tool_trans_wrt_world, np.linalg.inv(tool_trans_wrt_ee))  

def translate_point( target_transform,point):
  if len(point) == 3:
    point =  np.concatenate([point, [1]])
  elif len(point) != 4:
    print 'Invalid dimension'
    return
  transformed_point = trans_dot( target_transform, point) # equation 2.23 in Murray
  return transformed_point

def transform_tool_and_check_ee_position(roll,pitch,yaw,tool_point):
  manip = robot.GetActiveManipulator();
  ee    = manip.GetEndEffector()

  desired_roll = quat_from_angle_vector(roll,np.array([1,0,0]))
  desired_pitch = quat_from_angle_vector(pitch,np.array([0,1,0]))
  desired_yaw = quat_from_angle_vector(yaw,np.array([0,0,1]))
  tool_rot_wrt_w = quat_dot(desired_yaw,desired_pitch,desired_roll) 

  desired_tool_wrt_w = trans_from_quat_point(tool_rot_wrt_w, tool_point ) 
  desired_ee_world = compute_Tee_at_given_Ttool(desired_tool_wrt_w, \
                                                manip.GetLocalToolTransform())
  ee.SetTransform(desired_ee_world)
  return desired_tool_wrt_w
  

def compute_two_arm_grasp():
  # This function computes four different two-arm grasps 
  # for box shaped obejcts at each side
  # grasp is parametrized by four parameters:
  # theta, depth_portion, and height_portion
  # theta controls the angle with respect to the grasp axis, ranges from 0 to PI
  # depth_portion determines how deep grasp axis is on the obj
  # height_portion determines how how the grasp axis is on the obj

  width = 0.8
  length = 0.8
  height = 0.5
  obj = box_body(env,width,length,height,\
                      name='obj',\
                      color=(0, 1, 1))
  env.Add(obj)

  # compute the object bounding box
  aabb = obj.ComputeAABB()
  x_extent = aabb.extents()[0]
  y_extent = aabb.extents()[1]
  z_extent = aabb.extents()[2]
  
  # grasp parameters
  depth_portion   = 0.5 # How deep is your grasp
  height_portion  = 0.7 # how high is your grasp
  theta           = PI/2  # angle around grasp axis; 0 to PI

  set_point(robot,[-0.7,0,0])
  
  yaw_list =[PI/2,0,PI,3*PI/2] #iterate through four sides
  grasps = []
  for yaw in yaw_list:
    if yaw == PI/2 or yaw ==3*PI/2:
      roll = theta
      pitch = 0
      # these directions are so adhoc, computed using trial-and-error
      if yaw==PI/2: 
        grasp_axis = np.array([0,1,0])  
        non_grasp_axis = np.array([-1,0,0]) 
      else:
        grasp_axis = np.array([0,-1,0]) 
        non_grasp_axis = np.array([1,0,0]) 
      extent = y_extent
      depth  = x_extent
    else:
      roll = theta 
      pitch = 0
      # these directions are so adhoc, computed using trial-and-error
      if yaw==0:
        grasp_axis = np.array([1,0,0])
        non_grasp_axis = np.array([0,1,0]) 
      else:
        grasp_axis = np.array([-1,0,0])
        non_grasp_axis = np.array([0,-1,0]) 
      extent = x_extent
      depth  = y_extent
    
    # compute the grasp point on the object surface
    grasp_width  = grasp_axis*(extent+0.045)
    grasp_depth  = non_grasp_axis*(-depth+2*depth*depth_portion)
    grasp_height = np.array([0,0,1]) * (z_extent-2*z_extent*height_portion)

    # grasp point on the obj for the right arm
    tool_point = aabb.pos() - grasp_width -grasp_depth -grasp_height

    robot.SetActiveManipulator('rightarm_torso')
    manip = robot.GetManipulator('rightarm_torso');
    rightarm_tool = transform_tool_and_check_ee_position(roll,pitch,yaw,tool_point)
    right_g_config = get_manipulator(robot).FindIKSolution(rightarm_tool, 0)   # finds the solution without checking collision
    set_config(robot, right_g_config, robot.GetActiveManipulator().GetArmIndices()) 

    # grasp point on the obj for the left arm
    tool_point = aabb.pos() + grasp_width -grasp_depth -grasp_height
    robot.SetActiveManipulator('leftarm')
    manip = robot.GetManipulator('leftarm');
    leftarm_tool = transform_tool_and_check_ee_position(roll,pitch,yaw,tool_point)
    left_g_config = get_manipulator(robot).FindIKSolution(leftarm_tool, 0)   # finds the solution without checking collision
    set_config(robot, left_g_config, robot.GetActiveManipulator().GetArmIndices()) 
    import pdb;pdb.set_trace()

  manip=robot.GetManipulator('rightarm');new_config = manip.GetArmDOFValues(); new_config[1]=new_config[1]*0; set_config(robot,new_config,manip.GetArmIndices()) 
  manip=robot.GetManipulator('leftarm');new_config = manip.GetArmDOFValues(); new_config[1]=new_config[1]*0; set_config(robot,new_config,manip.GetArmIndices()) 

  """
  tool_point = tool_point - np.array([0,0,z_extent]) + \
               np.array([0,0,z_extent*2*z_portion])
  tool_point = tool_point - np.array([x_extent,0,0]) + \
               np.array([2*x_extent*1,0,0])
  """

  """

  robot.SetActiveManipulator('leftarm')
  manip = robot.GetManipulator('leftarm');
  ee    = manip.GetEndEffector()
  tool_point = aabb.pos() - np.array([0,y_extent,0]) + \
               np.array([0,2*y_extent+0.045,0])
  tool_point = tool_point - np.array([0,0,z_extent]) + \
               np.array([0,0,z_extent*2*z_portion])
  tool_point = tool_point - np.array([x_extent,0,0]) + \
               np.array([2*x_extent*1,0,0])

  # x_portion ranges from 0.5 to 1
  leftarm_tool = transform_tool_and_check_ee_position(roll,pitch,yaw,tool_point)
  print env.CheckCollision(ee)
  obj.Enable(True)
  #g_config = inverse_kinematics_helper(env, robot, leftarm_tool) # solve for ee pose
  left_g_config = get_manipulator(robot).FindIKSolution(leftarm_tool, 0)   # finds the solution without checking collision
  set_config(robot, left_g_config, robot.GetActiveManipulator().GetArmIndices()) 
  robot.Grab(obj) # attaches the object to the robot; no longer checks collision
  import pdb;pdb.set_trace()

  """
  import pdb;pdb.set_trace()

  
  #set_config(robot,mirror_arm_config(g_config[1:]),robot.GetManipulator('rightarm').GetArmIndices()) 
  #ee_right = robot.GetManipulator('rightarm').GetEndEffector()


  #g_config = get_manipulator(robot).FindIKSolution(leftarm_tool, openravepy_int.IkFilterOptions.CheckEnvCollisions)
  """
  set_config(robot,mirror_left_arm(g_config),robot.GetManipulator('rightarm_torso'))
  coll = [k for k in robot.GetLinks() if env.CheckCollision(k)]

  import pdb;pdb.set_trace()
  robot.Grab(obj) # attaches the object to the robot; no longer checks collision
  """
  """
  robot.SetActiveManipulator('rightarm_torso')
  manip = robot.GetManipulator('rightarm_torso');
  ee    = manip.GetEndEffector()
  tool_point = aabb.pos() - np.array([0,y_extent+0.02,0])
  tool_point = tool_point - np.array([0,0,z_extent]) + \
               np.array([0,0,z_extent*2*z_portion])
  rightarm_tool = transform_tool_and_check_ee_position(roll,pitch,yaw,tool_point)
  g_config = inverse_kinematics_helper(env, robot, rightarm_tool) # solve for ee pose
  set_config(robot, g_config, robot.GetActiveManipulator().GetArmIndices())
  import pdb;pdb.set_trace()
  """
  return 

def compute_grasp(pitch,z_rotation,z_portion,slide_portion,approach_portion,obj):
  # returns tool transform given grasp parameters, given a 
  # box-shaped obj

  ### Notations
  # w = world
  # o = obj
  # ee = end-effector

  # Place object at some location
  set_quat(obj,quat_from_z_rot(PI/2))
  set_point(obj,np.array([-0.8,0.188,1.01967739]))
  o_wrt_w = copy.deepcopy(get_trans(obj))
  
  ### First, compute the gtrans, the relative trans of tool wrt obj,
  ### by temporarily moving object to the origin
  # place it origin for convenience for computing rotation
  obj.SetTransform(np.eye(4))
  aabb = obj.ComputeAABB()
  x_extent = aabb.extents()[0]
  y_extent = aabb.extents()[1]
  z_extent = aabb.extents()[2] 
  yaws = [0,PI/2,PI,3*PI/2] # rotation about z axis

  for yaw in yaws:
    tool_point = aabb.pos() - np.array([0,0,z_extent]) + \
                 np.array([0,0,z_portion*2*z_extent])
    tool_point = tool_point - np.array([0,y_extent,0]) + \
                 np.array([0,slide_portion*2*y_extent,0])
    tool_point = tool_point - np.array([x_extent,0,0]) + \
                 np.array([2*x_extent*approach_portion,0,0])

    # compute the desired rotation of tool transform wrt world frame
    desired_pitch = quat_from_angle_vector(pitch,np.array([0,1,0]))
    desired_yaw   = quat_from_angle_vector(yaw,np.array([0,0,1]))
    # order of rotation matters; TODO study this later
    tool_rot_wrt_w = quat_dot(desired_yaw,desired_pitch) 
    
    desired_tool_wrt_w = trans_from_quat_point(tool_rot_wrt_w, tool_point ) 

    # Compute the tool transform wrt object frame. We call this grasp transform.
    gtrans = compute_tool_trans_wrt_obj_trans(desired_tool_wrt_w, get_trans(obj))

    # Compute the corresponding ee transform corresponding to the desired tool transform
    # Good for visualization of where the grasp will be
    desired_ee_world = compute_Tee_at_given_Ttool(desired_tool_wrt_w, \
                                                  manip.GetLocalToolTransform())
    ee.SetTransform(desired_ee_world)
    import pdb;pdb.set_trace()
    ##### End of computing relative transform of tool

  ### Now move object back to the original location, and then 
  ### use the relative tool trans, gtrans, to compute a new grasp
  # move object back to the origin
  obj.SetTransform(o_wrt_w)

  # compute the real desired tool_point
  # use relative tool trans to compute a new trans in the new object transform
  desired_tool_wrt_w= np.dot(get_trans(obj),gtrans)  
  desired_ee_world = compute_Tee_at_given_Ttool(desired_tool_wrt_w, manip.GetLocalToolTransform())
  ee.SetTransform(desired_ee_world) # visualize 
  ### Done! Now test IK

  obj.Enable(False)
  g_config = inverse_kinematics_helper(env, robot, desired_tool_wrt_w) # solve for ee pose
  set_config(robot, g_config, robot.GetActiveManipulator().GetArmIndices())
  
  # later

def get_four_grasps():
  # obj transform wrt world
  o_wrt_w = obj.GetTransform()

  #### Determine four different tool transforms that rotates around z-axis, and rotated about y-axis at angle theta
  gtrans_list = []
  theta = 150* PI/180. # theta in my grasp parametrization
  # desired rotation about y-axis
  ee_th_rot_wrt_o = quat_from_angle_vector( theta,np.array([0,1,0])) 

  for rotation in [0,PI/2,PI,3*PI/2]: # four rotations around z
    quat_z_rot = quat_from_angle_vector( rotation,np.array([0,0,1]))
    ee_rot_wrt_o = quat_dot(ee_th_rot_wrt_o,quat_z_rot) # apply two rotations to get the desired rotation of ee
    o_rot_wrt_w = quat_from_trans( o_wrt_w )            # obj rotation wrt w
    ee_rot_wrt_w = quat_dot(o_rot_wrt_w,ee_rot_wrt_o)   # ee rotation wrt w
    ee_wrt_w = trans_from_quat(ee_rot_wrt_w)            # ee transform wrt w

    # compute the point that we want to reach, and the angle in quaternion
    manip_point = np.array([0,0,0])
    manip_quat  = quat_from_trans( ee_wrt_w )
    desired_ee_wrt_w = trans_from_quat_point(manip_quat, manip_point)

    # compute the tool transform given an desired ee transform using
    # M*T  = P
    # where T is the returned tool transform, M is the ee transform, P is the point we want to reach (ex. center of obj)
    gtrans= compute_grasp(desired_ee_wrt_w, unit_trans()) 

    # see the result
    Tdelta = np.dot(gtrans,np.linalg.inv(manip.GetEndEffectorTransform())) # rotate by theta around [0,1,0]?
    for link in manip.GetChildLinks(): link.SetTransform(np.dot(Tdelta,link.GetTransform()))
    import pdb;pdb.set_trace()
  #####


def rotate_obj():
  #### rotate obj by theta
  theta=PI/4
  obj.SetTransform(np.eye(4))
  rot = quat_from_angle_vector( theta,np.array([0,0,1]))
  set_quat(obj,rot)
  o_wrt_w = obj.GetTransform()
  ##### 

def translate_ee():
  #### placing ee at 0.1 in x direction wrt obj
  ee_wrt_o = np.eye(4); ee_wrt_o[0,3] = 0.1  # desired ee location
  ee_pos_wrt_w = trans_dot( o_wrt_w, ee_wrt_o[:,3]) # equation 2.23 in Murray
  ee_rot_wrt_w = trans_dot( rot_from_trans(o_wrt_w),rot_from_trans(ee_wrt_o))
  ee_wrt_w     = np.eye(4); ee_wrt_w[:3,:3] =ee_rot_wrt_w; ee_wrt_w[:,3]=ee_pos_wrt_w
  ee.SetTransform(ee_wrt_w)
  #### 

def rotate_ee():
  #### rotating ee wrt x axis of obj
  theta=PI/4
  ee_rot_wrt_o = quat_from_angle_vector( PI,np.array([1,0,0]))
  o_rot_wrt_w = quat_from_trans( o_wrt_w ) 
  ee_rot_wrt_w = quat_dot(o_rot_wrt_w,ee_rot_wrt_o) 
  ee_wrt_w = trans_from_quat(ee_rot_wrt_w)
  ee.SetTransform(ee_wrt_w)
  #### 

#compute_grasp(pitch=PI/4,z_rotation=PI/2,z_portion=0.5,slide_portion=0.5,approach_portion=0.5,obj=obj)
compute_two_arm_grasp()
import pdb;pdb.set_trace()
