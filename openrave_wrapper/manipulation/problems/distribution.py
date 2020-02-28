from manipulation.problems.fixed import ENVIRONMENTS_DIR
from manipulation.bodies.bodies import box_body, randomly_place_body
from manipulation.problems.problem import *
from manipulation.bodies.bodies import get_name
from misc.functions import randomize
from manipulation.primitives.transforms import get_point, set_point, pose_from_quat_point, unit_quat, set_pose, set_base_values
from misc.colors import get_color
from manipulation.constants import BODY_PLACEMENT_Z_OFFSET, REARRANGEMENT
from manipulation.primitives.utils import Pose
from manipulation.bodies.robot import set_default_robot_config
from misc.utils import function_name, flatten
from itertools import product
from random import sample
import math

def separate(env, n=7): # Previously 4, 8
  assert not REARRANGEMENT
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
  object_names = [get_name(body) for body in objects]

  for obj in randomize(objects):
    randomly_place_body(env, obj, ['table2', 'table4'])

  return ManipulationProblem(None,
    object_names=object_names, table_names=table_names,
    goal_regions=goal_regions)

separate_2 = lambda env: separate(env, n=2)
separate_4 = lambda env: separate(env, n=4)
separate_6 = lambda env: separate(env, n=6)
separate_7 = lambda env: separate(env, n=7)

#################################################################

def two_tables(env, n=2):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + '2tables.xml')
  set_default_robot_config(env.GetRobots()[0])
  table_names = filter(lambda name: 'table' in name, [get_name(body) for body in env.GetBodies() if not body.IsRobot()])

  #m = 4*n
  objects = []
  goal_regions = {}
  #for i in range(4*m):
  for i in range(10*n):
    objects.append(box_body(env, .07, .07, .2, name='red'+str(i+1), color=RED))
  #for i in range(n):
  for i in range(1):
    name = 'blue'+str(i+1)
    objects.append(box_body(env, .07, .07, .2, name=name, color=BLUE))
    goal_regions[name] = 'table2'
  object_names = [get_name(body) for body in objects]

  for obj in randomize(objects):
    randomly_place_body(env, obj, ['table1'])

  return ManipulationProblem(None,
    object_names=object_names, table_names=table_names,
    goal_regions=goal_regions)

two_tables_1 = lambda env: two_tables(env, n=1)
two_tables_2 = lambda env: two_tables(env, n=2)
two_tables_3 = lambda env: two_tables(env, n=3)
two_tables_4 = lambda env: two_tables(env, n=4)

#################################################################

def dantam(env): # (Incremental Task and Motion Planning: A Constraint-Based Approach)
  assert REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'empty.xml')
  set_default_robot_config(env.GetRobots()[0])
  set_point(env.GetRobots()[0], (-1.5, 0, 0))

  m, n = 3, 3
  #m, n = 5, 5
  n_obj = 8
  side_dim = .07
  height_dim = .1
  box_dims = (side_dim, side_dim, height_dim)
  separation = (side_dim, side_dim)

  length = m*(box_dims[0] + separation[0])
  width = n*(box_dims[1] + separation[1])
  height = .7
  table = box_body(env, length, width, height, name='table', color=get_color('tan1'))
  set_point(table, (0, 0, 0))
  env.Add(table)

  pose_indices = list(product(range(m), range(n)))
  colors = {}
  for r, c in pose_indices:
    color = np.zeros(4)
    color[2-r] = 1.
    colors[(r, c)] = color + float(c)/(n-1)*np.array([1, 0, 0, 0])

  poses = {}
  z =  get_point(table)[2] + height + BODY_PLACEMENT_Z_OFFSET
  for r, c in pose_indices:
    x = get_point(table)[0] - length/2 + (r+.5)*(box_dims[0] + separation[0])
    y = get_point(table)[1] - width/2 + (c+.5)*(box_dims[1] + separation[1])
    poses[(r, c)] = Pose(pose_from_quat_point(unit_quat(), np.array([x, y, z])))

  initial_indices = randomize(pose_indices[:])
  initial_poses = {}
  goal_poses = {}
  for i, indices in enumerate(pose_indices[:n_obj]):
    name = 'block%d-%d'%indices
    color = colors[indices]
    initial_poses[name] = poses[initial_indices[i]]
    goal_poses[name] = poses[indices]
    obj = box_body(env, *box_dims, name=name, color=color)
    set_pose(obj, initial_poses[name].value)
    env.Add(obj)

  #for obj in randomize(objects):
  #  randomly_place_body(env, obj, [get_name(table)])

  return ManipulationProblem(function_name(inspect.stack()),
    object_names=initial_poses.keys(), table_names=[get_name(table)],
    goal_poses=goal_poses,
    initial_poses=initial_poses, known_poses=poses.values())

def dantam_distract(env, n_obj): # (Incremental Task and Motion Planning: A Constraint-Based Approach)
  assert REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'empty.xml')

  m, n = 3, 3
  #m, n = 5, 5
  side_dim = .07 # .05 | .07
  height_dim = .1
  box_dims = (side_dim, side_dim, height_dim)
  separation = (side_dim, side_dim)
  #separation = (side_dim/2, side_dim/2)

  coordinates = list(product(range(m), range(n)))
  assert n_obj <= len(coordinates)
  obj_coordinates = sample(coordinates, n_obj)

  length = m*(box_dims[0] + separation[0])
  width = n*(box_dims[1] + separation[1])
  height = .7
  table = box_body(env, length, width, height, name='table', color=get_color('tan1'))
  set_point(table, (0, 0, 0))
  env.Add(table)

  robot = env.GetRobots()[0]
  set_default_robot_config(robot)
  set_base_values(robot, (-1.5, 0, 0))
  #set_base_values(robot, (0, width/2 + .5, math.pi))
  #set_base_values(robot, (.35, width/2 + .35, math.pi))
  #set_base_values(robot, (.35, width/2 + .35, 3*math.pi/4))

  poses = []
  z =  get_point(table)[2] + height + BODY_PLACEMENT_Z_OFFSET
  for r in range(m):
    row = []
    x = get_point(table)[0] - length/2 + (r+.5)*(box_dims[0] + separation[0])
    for c in range(n):
      y = get_point(table)[1] - width/2 + (c+.5)*(box_dims[1] + separation[1])
      row.append(Pose(pose_from_quat_point(unit_quat(), np.array([x, y, z]))))
    poses.append(row)

  initial_poses = {}
  goal_poses = {}
  # TODO - randomly assign goal poses here
  for i, (r, c) in enumerate(obj_coordinates):
    row_color = np.zeros(4)
    row_color[2-r] = 1.
    if i == 0:
      name = 'goal%d-%d'%(r, c)
      color = BLUE
      goal_poses[name] = poses[m/2][n/2]
    else:
      name = 'block%d-%d'%(r, c)
      color = RED
    initial_poses[name] = poses[r][c]
    obj = box_body(env, *box_dims, name=name, color=color)
    set_pose(obj, poses[r][c].value)
    env.Add(obj)

  #for obj in randomize(objects):
  #  randomly_place_body(env, obj, [get_name(table)])

  known_poses = list(flatten(poses))
  #known_poses = list(set(flatten(poses)) - {poses[r][c] for r, c in obj_coordinates}) # TODO - put the initial poses here

  return ManipulationProblem(function_name(inspect.stack()),
    object_names=initial_poses.keys(), table_names=[get_name(table)],
    goal_poses=goal_poses,
    initial_poses=initial_poses, known_poses=known_poses)

dantam_3 = lambda env: dantam_distract(env, n_obj=3)
dantam2 = lambda env: dantam_distract(env, n_obj=8)

#################################################################

# TODO - unclear if they actually shrink the table for smaller instances...

def grid_arrangement(env, m, n): # (Dealing with Difficult Instances of Object Rearrangment)
  assert REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'empty.xml')

  box_dims = (.12, .04, .08)
  #separation = (.08, .08)
  separation = (.12, .12)
  #separation = (.16, .16)

  length = m*(box_dims[0] + separation[0])
  width = n*(box_dims[1] + separation[1])
  height = .7
  table = box_body(env, length, width, height, name='table', color=get_color('tan1'))
  #set_point(table, (1.75, 0, 0))
  set_point(table, (0, 0, 0))
  env.Add(table)

  robot = env.GetRobots()[0]
  set_default_robot_config(robot)
  set_base_values(robot, (-1.5, 0, 0))

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

rearrangement_4 = lambda env: grid_arrangement(env, 2, 2)
rearrangement_6 = lambda env: grid_arrangement(env, 2, 3)
rearrangement_8 = lambda env: grid_arrangement(env, 2, 4)
rearrangement_10 = lambda env: grid_arrangement(env, 2, 5)
rearrangement_12 = lambda env: grid_arrangement(env, 2, 6)
rearrangement_14 = lambda env: grid_arrangement(env, 2, 7)
rearrangement_16 = lambda env: grid_arrangement(env, 2, 8)
rearrangement_18 = lambda env: grid_arrangement(env, 2, 9)
rearrangement_20 = lambda env: grid_arrangement(env, 2, 10)

#################################################################

def shelf_arrangement(env): # (Dealing with Difficult Instances of Object Rearrangment)
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'empty.xml')
  set_default_robot_config(env.GetRobots()[0])
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

#################################################################

def move_several(env, n):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'empty.xml')

  box_dims = (.07, .07, .2)
  #separation = (.08, .08)
  separation = (.10, .10)

  length = math.sqrt(n+1)*(box_dims[0] + separation[0])
  width = math.sqrt(n+1)*(box_dims[1] + separation[1])
  height = .7

  table1 = box_body(env, length, width, height, name='table1', color=get_color('tan1'))
  set_point(table1, (0, 0, 0))
  env.Add(table1)

  table2 = box_body(env, length, width, height, name='table2', color=get_color('tan1'))
  set_point(table2, (1.5, 0, 0))
  env.Add(table2)

  robot = env.GetRobots()[0]
  set_default_robot_config(robot)
  set_base_values(robot, (-1.5, 0, 0))

  # TODO - place walls and/or a roof to make more similar to pebble graph people

  objects = []
  goal_regions = {}

  obj = box_body(env, .07, .07, .2, name='blue', color=BLUE)
  set_point(obj, (0, 0, height + BODY_PLACEMENT_Z_OFFSET))
  objects.append(obj)
  goal_regions[get_name(obj)] = get_name(table2)
  env.Add(obj)

  for i in range(n):
    objects.append(box_body(env, .07, .07, .2, name='red'+str(i+1), color=RED))
  for obj in randomize(objects[1:]):
    randomly_place_body(env, obj, [get_name(table1)])

  return ManipulationProblem(None,
    object_names=[get_name(body) for body in objects], table_names=[get_name(table) for table in [table1, table2]],
    goal_regions=goal_regions)



move_several_4 = lambda env: move_several(env, 4)
move_several_8 = lambda env: move_several(env, 8)
# ...

move_several_10 = lambda env: move_several(env, 10)
move_several_12 = lambda env: move_several(env, 12)
move_several_14 = lambda env: move_several(env, 14)
move_several_16 = lambda env: move_several(env, 16)
move_several_18 = lambda env: move_several(env, 18)
move_several_20 = lambda env: move_several(env, 20)

move_several_24 = lambda env: move_several(env, 24)
move_several_28 = lambda env: move_several(env, 28)
move_several_32 = lambda env: move_several(env, 32)
