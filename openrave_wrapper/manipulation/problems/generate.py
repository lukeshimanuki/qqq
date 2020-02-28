from manipulation.problems.fixed import ENVIRONMENTS_DIR
from manipulation.bodies.bodies import box_body, randomly_place_body, place_xyz_body
from manipulation.problems.problem import *
from manipulation.bodies.bodies import get_name
from misc.functions import randomize
from misc.generators import take
from misc.numerical import INF
from manipulation.bodies.robot import set_default_robot_config
from manipulation.primitives.transforms import get_point, set_point, pose_from_quat_point, unit_quat
from misc.colors import get_color
from manipulation.constants import BODY_PLACEMENT_Z_OFFSET
from manipulation.primitives.utils import Pose

##TODO: Clean this
from manipulation.constants import PARALLEL_LEFT_ARM, REST_LEFT_ARM, HOLDING_LEFT_ARM, FOLDED_LEFT_ARM, FAR_HOLDING_LEFT_ARM, LOWER_TOP_HOLDING_LEFT_ARM,REGION_Z_OFFSET
from manipulation.regions import create_region, AARegion
from manipulation.bodies.bodies import randomly_place_region, place_body
from manipulation.inverse_reachability.inverse_reachability import ir_base_trans
from manipulation.primitives.utils import mirror_arm_config
from manipulation.primitives.transforms import trans_from_base_values, set_pose, set_quat, \
  point_from_pose, axis_angle_from_rot, rot_from_quat, quat_from_pose, quat_from_z_rot,\
  get_pose,base_values_from_pose,pose_from_base_values, set_xy
from itertools import product
import numpy as np
import copy
import math

from manipulation.bodies.bounding_volumes import aabb_extrema, aabb_from_body, aabb_union
from manipulation.inverse_reachability.inverse_reachability import get_custom_ir, get_base_generator
from manipulation.bodies.robot import manip_from_pose_grasp
from manipulation.bodies.robot import get_active_arm_indices
from manipulation.grasps.grasps import FILENAME as GRASP_FILENAME, load_grasp_database
from manipulation.grasps.grasp_options import positive_hash, get_grasp_options
from manipulation.constants import GRASP_APPROACHES, GRASP_TYPES

from manipulation.bodies.bodies import geometry_hash
from manipulation.bodies.bounding_volumes import aabb_from_body
from manipulation.grasps.grasps import save_grasp_database, Grasp
from openravepy import *

def separate(env, n=7): # Previously 4, 8
  env.Load(ENVIRONMENTS_DIR + 'tables.xml')
  set_default_robot_config(env.GetRobots()[0])
  table_names = filter(lambda name: 'table' in name, [get_name(body) for body in env.GetBodies() if not body.IsRobot()])

  objects = []
  goal_regions = {}
  for i in range(2*n):
    objects.append(box_body(env, .07, .07, .2, name='red'+str(i+1), color=RED))
  for i in range(n):
    name = 'green'+str(i+1)
    objects.append(box_body(env, .07, .07, .2, name=name, color=GREEN))
    goal_regions[name] = 'table1'
  for i in range(n):
    name = 'blue'+str(i+1)
    objects.append(box_body(env, .07, .07, .2, name=name, color=BLUE))
    goal_regions[name] = 'table3'
  objects.append(box_body(env, .07, .07, .2, name='black', color=BLACK))


  object_names = [get_name(body) for body in objects]
  robot = env.GetRobots()[0]
  robot.SetActiveManipulator('leftarm')
  print robot.GetActiveManipulator().GetLocalToolTransform()

  grasps = {}
  #for obj_name in object_names:
  obj_name = 'black'
  env.Add(objects[-1]) 
  obj = env.GetKinBody(obj_name)
  with obj:
    obj.SetTransform(np.eye(4))
    obj_grasps = get_grasps(env, robot, obj, GRASP_APPROACHES.SIDE, GRASP_TYPES.GRASP) 

  #obj_grasps = get_grasps(env, robot, obj, GRASP_APPROACHES.TOP, GRASP_TYPES.TOUCH) # TOP and SIDE are swapped
  grasps[get_name(obj)] = obj_grasps
  for obj in randomize(objects):
    randomly_place_body(env, obj, ['table2', 'table4'])
  return ManipulationProblem(None,
    object_names=object_names, table_names=table_names,
    goal_regions=goal_regions,grasps=grasps)

def srivastava_table(env, n=INF):
  env.Load(ENVIRONMENTS_DIR + '../srivastava/good_cluttered.dae')
  set_default_robot_config(env.GetRobots()[0])
  body_names = [get_name(body) for body in env.GetBodies() if not body.IsRobot()]
  table_names = [body_name for body_name in body_names if 'table' in body_name]

  dx = .5
  for body_name in body_names:
    body = env.GetKinBody(body_name)
    set_point(body, get_point(body) + np.array([dx, 0, 0]))

  objects = [env.GetKinBody(body_name) for body_name in body_names if body_name not in table_names]
  for obj in objects: env.Remove(obj)
  object_names = []
  for obj in take(objects, n):
    randomly_place_body(env, obj, table_names)
    object_names.append(get_name(obj))

  goal_holding = 'object1'
  goal_config = 'initial' # None

  return ManipulationProblem(None,
      object_names=object_names, table_names=table_names,
      goal_config=goal_config, goal_holding=goal_holding)

def grid_arrangement(env): # (Dealing with Difficult Instances of Object Rearrangment)
  env.Load(ENVIRONMENTS_DIR + 'empty.xml')
  #m, n = 2, 10
  m, n = 2, 8
  box_dims = (.12, .04, .08)
  #separation = (.08, .08)
  separation = (.12, .12)
  #separation = (.16, .16)

  length = m*(box_dims[0] + separation[0])
  width = n*(box_dims[1] + separation[1])
  height = .7
  table = box_body(env, length, width, height, name='table', color=get_color('tan1'))
  set_point(table, (1.75, 0, 0))
  env.Add(table)

  objects = []
  goal_poses = {}
  z =  get_point(table)[2] + height + BODY_PLACEMENT_Z_OFFSET
  for i in range(m):
    x = get_point(table)[0] - length/2 + (i+.5)*(box_dims[0] + separation[0])
    row_color = np.zeros(4)
    row_color[2-i] = 1.
    for j in range(n):
      y = get_point(table)[1] - width/2 + (j+.5)*(box_dims[1] + separation[1])
      name = 'block%d-%d'%(i, j)
      color = row_color + float(j)/(n-1)*np.array([1, 0, 0, 0])
      goal_poses[name] = Pose(pose_from_quat_point(unit_quat(), np.array([x, y, z])))
      objects.append(box_body(env, *box_dims, name=name, color=color))
  object_names = [get_name(body) for body in objects]

  for obj in randomize(objects):
    randomly_place_body(env, obj, [get_name(table)])

  return ManipulationProblem(None,
    object_names=object_names, table_names=[get_name(table)],
    goal_poses=goal_poses)

# TODO - use non-vaporize to try solving this one
def shelf_arrangement(env): # (Dealing with Difficult Instances of Object Rearrangment)
  env.Load(ENVIRONMENTS_DIR + 'empty.xml')
  #m, n = 2, 10
  m, n = 2, 4
  box_dims = (.07, .07, .2)
  #separation = (.08, .08)
  separation = (.15, .15)

  length = m*(box_dims[0] + separation[0])
  width = n*(box_dims[1] + separation[1])
  height = .7
  table = box_body(env, length, width, height, name='table', color=get_color('tan1'))
  set_point(table, (1.75, 0, 0))
  env.Add(table)
  # TODO - place walls and/or a roof to make more similar to pebble graph people

  objects = []
  goal_poses = {}
  z =  get_point(table)[2] + height + BODY_PLACEMENT_Z_OFFSET
  for i in range(m):
    x = get_point(table)[0] - length/2 + (i+.5)*(box_dims[0] + separation[0])
    row_color = np.zeros(4)
    row_color[2-i] = 1.
    for j in range(n):
      y = get_point(table)[1] - width/2 + (j+.5)*(box_dims[1] + separation[1])
      name = 'block%d-%d'%(i, j)
      color = row_color + float(j)/(n-1)*np.array([1, 0, 0, 0])
      goal_poses[name] = Pose(pose_from_quat_point(unit_quat(), np.array([x, y, z])))
      objects.append(box_body(env, *box_dims, name=name, color=color))
  object_names = [get_name(body) for body in objects]

  for obj in randomize(objects):
    randomly_place_body(env, obj, [get_name(table)])

  return ManipulationProblem(None,
    object_names=object_names, table_names=[get_name(table)],
    goal_poses=goal_poses)

class HackedOracle(object):
  def __init__(self, env):
    self.env = env
    self.robot = env.GetRobots()[0]
  def get_aabb(self, body_name):
    return aabb_from_body(self.env.GetKinBody(body_name))
  def get_body_name(self, geom_hash):
    for body in self.env.GetBodies():
      if geometry_hash(body) == geom_hash:
        return get_name(body)
    return None

def hacked_create_grasp_database(env, grasp_options):
  oracle = HackedOracle(env)
  grasps = []
  for grasp_tform, approach_vector in grasp_options.grasp_trans(oracle):
    gr = Grasp(grasp_tform, None, None, approach_vector)
    grasps.append(Grasp(grasp_tform, None, None, approach_vector)) # TODO - grasp incomplete
  return grasps

def get_grasps(env, robot, body, grasp_approach, grasp_type):
  Options = get_grasp_options(body, grasp_approach)
  grasp_options = Options.default(grasp_type, body)
  filename = GRASP_FILENAME%positive_hash(grasp_options) # TODO - get rid of the negative numbers
  """
  print 'Creating', get_name(body), GRASP_TYPES.names[grasp_type], 'database' # TODO - move this to create_grasp_database
  c_grasps = hacked_create_grasp_database(env, grasp_options)
  o_grasps = load_grasp_database(robot,filename)
  return c_grasps
  #import pdb;pdb.set_trace()
  """
  try:
    return load_grasp_database(robot, filename)
  except IOError:
    print 'Creating', get_name(body), GRASP_TYPES.names[grasp_type], 'database' # TODO - move this to create_grasp_database
    grasps = hacked_create_grasp_database(env, grasp_options)
    #save_grasp_database(filename, grasps) # TODO - not saving right not because incomplete
  return grasps



def rearrangement_problem(env,obj_poses=None):
#  import random 
#  seed = 50
#  random.seed(seed)
#  np.random.seed(seed)

  MIN_DELTA = .01 # .01 | .02
  ENV_FILENAME = ENVIRONMENTS_DIR+'/single_table.xml' 
  env.Load(ENV_FILENAME)
  robot = env.GetRobots()[0]

  TABLE_NAME = 'table1'

  NUM_OBJECTS = 8
  WIDTH = .05 # .07 | .1
  TARGET_WIDTH = 0.07
  OBJ_HEIGHT = 0.2
  objects = [box_body(env, WIDTH, WIDTH, .2, name='obj%s'%i, \
              color=(0, (i+.5)/NUM_OBJECTS, 0)) for i in range(NUM_OBJECTS)]
  target_obj =  box_body(env, TARGET_WIDTH, TARGET_WIDTH, .2, name='target_obj', \
                           color=(1, 1.5, 0)) 

  #TODO: Place obstacle that prevent you to reach from top
  
  OBST_X = 0
  OBST_WIDTH = 1.
  OBST_COLOR = (1, 0, 0)
  OBST_HEIGHT = 0.4
  OBST_TRANSPARENCY = .25
  place_body(env, box_body(env, .4, .05, OBST_HEIGHT, name='obst1', color=OBST_COLOR, transparency=OBST_TRANSPARENCY),
             (.0, OBST_X-(OBST_WIDTH-.05)/2, 0), TABLE_NAME)
  place_body(env, box_body(env, .4, .05, OBST_HEIGHT, name='obst2', color=OBST_COLOR, transparency=OBST_TRANSPARENCY),
             (.0, OBST_X+(OBST_WIDTH-.05)/2, 0), TABLE_NAME)
  place_body(env, box_body(env, .05, OBST_WIDTH, OBST_HEIGHT, name='obst3', color=OBST_COLOR, transparency=OBST_TRANSPARENCY),
             (.225, OBST_X, 0), TABLE_NAME)
  # roof
  OBST_Z = OBST_HEIGHT
  place_xyz_body(env, box_body(env, .4, OBST_WIDTH, .01, name='obst4', color=OBST_COLOR, transparency=OBST_TRANSPARENCY),
           (0, OBST_X, OBST_Z,0), TABLE_NAME)

  # I think this can be done once and for all
  REST_TORSO = [.15]
  robot.SetDOFValues(REST_TORSO, [robot.GetJointIndex('torso_lift_joint')])
  l_model = databases.linkstatistics.LinkStatisticsModel(robot)
  if not l_model.load(): l_model.autogenerate()
  l_model.setRobotWeights()
  min_delta = 0.01
  l_model.setRobotResolutions(xyzdelta=min_delta) # xyzdelta is the minimum Cartesian distance of an object
  extrema = aabb_extrema(aabb_union([aabb_from_body(body) for body in env.GetBodies()])).T
  robot.SetAffineTranslationLimits(*extrema)

  X_PERCENT = 0.0 

  # define intial region for movable objects
  init_region = create_region(env, 'init_region', ((-0.8, 0.8), (-0.7, 0.6)), TABLE_NAME, color=np.array((1, 0, 1, .5)))
  #init_region.draw(env)
  
  # define target object region
  target_obj_region = create_region(env, 'target_obj_region', ((-0.2, 0.6), (-0.5,0.5)), \
                        TABLE_NAME, color=np.array((0, 0, 0, 0.9)))
  target_obj_region.draw(env)

  # place object
  if obj_poses is not None:
    for obj in objects:
      set_pose(obj,obj_poses[get_name(obj)])
      set_quat(obj, quat_from_z_rot(0))
      if env.GetKinBody(get_name(obj)) is None: env.Add(obj)
    set_pose( target_obj, obj_poses[get_name(target_obj)] )
    set_quat( target_obj, quat_from_z_rot(0) ) 
    if env.GetKinBody(get_name(target_obj)) is None: env.Add(target_obj)
  else:
    for obj in objects:
      randomly_place_region(env, obj, init_region)
      set_quat(obj, quat_from_z_rot(0))
    randomly_place_region(env, target_obj, target_obj_region)
    set_quat(target_obj, quat_from_z_rot(0))

  object_names = [get_name(body) for body in objects]           # (tothefront,totheback), (to_the_right,to_the_left)


  set_xy(robot, -.75, 0)

#  LEFT_ARM_CONFIG = HOLDING_LEFT_ARM
  robot.SetDOFValues(REST_LEFT_ARM, robot.GetManipulator('leftarm').GetArmIndices())
  robot.SetDOFValues(mirror_arm_config(REST_LEFT_ARM), robot.GetManipulator('rightarm').GetArmIndices())
  
  grasps = {}
  for obj_name in object_names:
    obj = env.GetKinBody(obj_name)
    obj_grasps = get_grasps(env, robot, obj, GRASP_APPROACHES.SIDE, GRASP_TYPES.TOUCH) \
                + get_grasps(env, robot, obj, GRASP_APPROACHES.TOP, GRASP_TYPES.TOUCH)
    grasps[get_name(obj)] = obj_grasps

  target_obj = env.GetKinBody('target_obj')
  target_obj_grasps = get_grasps(env, robot, target_obj, GRASP_APPROACHES.TOP, GRASP_TYPES.TOUCH)+\
                      get_grasps(env, robot, target_obj, GRASP_APPROACHES.SIDE, GRASP_TYPES.TOUCH)
  grasps['target_obj'] = target_obj_grasps

  initial_poses={}
  for obj in objects:
    initial_poses[get_name(obj)] = get_pose(obj)
  initial_poses[get_name(target_obj)] = get_pose(target_obj)

  return ManipulationProblem(None,
      object_names=object_names, table_names='table1',
      goal_regions=init_region,grasps=grasps,
      initial_poses = initial_poses) 



def two_tables_through_door(env, obj_length, n=7): # Previously 4, 8
  env.Load(ENVIRONMENTS_DIR + 'two_tables'+str(obj_length)+'.xml')
  #env.Load(ENVIRONMENTS_DIR + 'two_tables'+'.xml')

  #set_default_robot_config(env.GetRobots()[0])
  table_names = filter(lambda name: 'table' in name, [get_name(body) for body in env.GetBodies() if not body.IsRobot()])

  objects = []
#  objects.append(env.GetKinBody("ObjToG"))
  goal_regions = {}
  for i in range(2*n):
    objects.append(box_body(env, .07, .07, .2, name='red'+str(i+1), color=RED))
  for i in range(n):
    name = 'green'+str(i+1)
    objects.append(box_body(env, .07, .07, .2, name=name, color=GREEN))
    goal_regions[name] = 'table1'
  for i in range(n):
    name = 'blue'+str(i+1)
    objects.append(box_body(env, .07, .07, .2, name=name, color=BLUE))
    goal_regions[name] = 'table3'
  object_names = [get_name(body) for body in objects]

  for obj in randomize(objects):
    randomly_place_body(env, obj, ['table2', 'table4'])
#    randomly_place_body(env, obj, ['table2'])

  return ManipulationProblem(None,
    object_names=object_names, table_names=table_names,
    goal_regions=goal_regions)
