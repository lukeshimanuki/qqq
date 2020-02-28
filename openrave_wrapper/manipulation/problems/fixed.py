from itertools import count, product
import inspect

from manipulation.problems.problem import *
from manipulation.regions import create_region, create_shelf_region
from manipulation.bodies.bodies import box_body, cylinder_body, place_body, place_body_on_floor, set_body
from manipulation.primitives.transforms import get_point, set_point, pose_from_quat_point, quat_from_axis_angle, camera_look_at
from manipulation.constants import BODY_PLACEMENT_Z_OFFSET, CAMERA_POINT_SCALE, REARRANGEMENT
from manipulation.primitives.utils import Config, Pose, Point
from misc.utils import PI, function_name
from misc.functions import irange
from itertools import product, islice

ENVIRONMENTS_DIR = PROBLEMS_DIR + '/environments/'

def simple(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'room1.xml')

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  #table_names = ['table1']
  table_names = ['table1', 'table2']

  regions = [
    create_region(env, 'blue_goal', ((-1, 1), (-1, 0)), 'table1', color=np.array((0, 0, 1, .5))),
  ]

  pb = place_body
  pb(env, box_body(env, .07, .05, .2, name='blue_box', color=BLUE), (1.65, 0.115, PI/3), 'table1')
  pb(env, box_body(env, .07, .05, .2, name='green_box', color=GREEN), (1.55, -0.215, 7*PI/6), 'table1')
  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  goal_regions = {
    'blue_box': 'blue_goal',
  }
  #goal_config = 'initial' # None
  goal_config =  None

  return ManipulationProblem(function_name(inspect.stack()),
      object_names=object_names, table_names=table_names, regions=regions,
      goal_config=goal_config, goal_regions=goal_regions)

def simple2(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'room1.xml')

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  #table_names = ['table1']
  table_names = ['table1', 'table2']

  regions = [
    create_region(env, 'blue_goal', ((-1, 1), (-1, 0)), 'table1', color=np.array((0, 0, 1, .5))),
    create_region(env, 'green_goal', ((-1, -.5), (-1, 1)), 'table2', color=np.array((0, 1, 0, .5)))
  ]

  pb = place_body
  pb(env, box_body(env, .07, .05, .2, name='blue_box', color=BLUE), (1.65, 0.115, PI/3), 'table1')
  pb(env, box_body(env, .07, .05, .2, name='green_box', color=GREEN), (1.55, -0.215, 7*PI/6), 'table1')
  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  goal_regions = {
    'blue_box': 'blue_goal',
    'green_box': 'green_goal'
  }

  return ManipulationProblem(function_name(inspect.stack()),
      object_names=object_names, table_names=table_names, regions=regions,
      goal_regions=goal_regions)

#######################################################################

def blocked(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'room1.xml')
  camera_trans = camera_look_at(CAMERA_POINT_SCALE*np.array([-1.5, 1.5, 3]), look_point=(1.5, -1.5, 0))

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1', 'table2']

  regions = [
    create_region(env, 'green_goal', ((-1, 1), (-1, 0)), 'table1', color=np.array((0, 1, 0, .5))),
  ]

  pb = place_body
  center = np.array([1.75, 0.25, 0])
  pb(env, box_body(env, .07, .05, .2, name='green_box', color=GREEN), center, 'table1')
  id = count(1)
  for dx, dy in [(-.15, 0), (.15, 0), (0, -.15), (0, .15)]:
    pb(env, box_body(env, .07, .05, .2, name='red_box%d'%next(id), color=RED), center + np.array([dx, dy, 0]), 'table1')
  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  goal_regions = {
    'green_box': 'green_goal',
  }

  return ManipulationProblem(function_name(inspect.stack()), camera_trans=camera_trans,
      object_names=object_names, table_names=table_names, regions=regions,
      goal_regions=goal_regions)

#######################################################################

def distractions(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'room1.xml')
  camera_trans = camera_look_at(CAMERA_POINT_SCALE*np.array([-1.5, 1.5, 3]), look_point=(1.5, -1.5, 0))

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1', 'table2']

  regions = [
    create_region(env, 'green_goal', ((-1, 1), (-1, 0)), 'table1', color=np.array((0, 1, 0, .5))),
  ]

  pb = place_body
  center = np.array([1.75, 0.25, 0])
  pb(env, box_body(env, .07, .05, .2, name='green_box', color=GREEN), center, 'table1')
  id = count(1)
  for dx, dy in product((-.15, 0, .15), repeat=2):
    if not (dx == 0 and dy == 0):
  #for dx, dy in [(-.15, 0), (.15, 0), (0, -.15), (0, .15)]:
  #  if True:
      pb(env, box_body(env, .07, .05, .2, name='red_box%d'%next(id), color=RED), center + np.array([dx, dy, 0]), 'table1')
  #for x in irange(-.7, .7, .15):
  #  for y in irange(-2.0, -1.5, .15):
  for x in irange(-.65, .75, .15):
    for y in irange(-1.95, -1.5, .15): # TODO - why did I make these a different size?
      pb(env, box_body(env, .05, .05, .15, name='red_box%d'%next(id), color=RED), np.array([x, y, 0]), 'table2')

  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  goal_regions = {
    'green_box': 'green_goal',
  }

  return ManipulationProblem(function_name(inspect.stack()), camera_trans=camera_trans,
      object_names=object_names, table_names=table_names, regions=regions,
      goal_regions=goal_regions)

#######################################################################

def distract_n(env, n):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'room1.xml')
  camera_trans = camera_look_at(CAMERA_POINT_SCALE*np.array([-1.5, 1.5, 3]), look_point=(1.5, -1.5, 0))

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1', 'table2']

  regions = [
    create_region(env, 'green_goal', ((-1, 1), (-1, 0)), 'table1', color=np.array((0, 1, 0, .5))),
  ]

  pb = place_body
  center = np.array([1.75, 0.25, 0])
  pb(env, box_body(env, .07, .05, .2, name='green_box', color=GREEN), center, 'table1')
  id = count(1)
  for dx, dy in [(-.15, 0), (.15, 0), (0, -.15), (0, .15)]:
    pb(env, box_body(env, .07, .05, .2, name='red_box%d'%next(id), color=RED), center + np.array([dx, dy, 0]), 'table1')
  #for x in irange(-.7, .7, .15):
  #  for y in irange(-2.0, -1.5, .15):
  for x, y in islice(product(irange(-.65, .75, .15), irange(-1.95, -1.5, .15)), n):
    pb(env, box_body(env, .07, .05, .2, name='red_box%d'%next(id), color=RED), np.array([x, y, 0]), 'table2')

  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  goal_regions = {
    'green_box': 'green_goal',
  }

  return ManipulationProblem(function_name(inspect.stack()), camera_trans=camera_trans,
      object_names=object_names, table_names=table_names, regions=regions,
      goal_regions=goal_regions)

distract_0 = lambda env: distract_n(env, 0)
distract_4 = lambda env: distract_n(env, 4)
distract_8 = lambda env: distract_n(env, 8)
distract_12 = lambda env: distract_n(env, 12)
distract_16 = lambda env: distract_n(env, 16)
distract_20 = lambda env: distract_n(env, 20)
distract_24 = lambda env: distract_n(env, 24)

distract_10 = lambda env: distract_n(env, 10)
distract_30 = lambda env: distract_n(env, 30)
distract_40 = lambda env: distract_n(env, 40)

#######################################################################

def replace(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'room1.xml')
  camera_trans = camera_look_at(CAMERA_POINT_SCALE*np.array([-1.5, 1.5, 3]), look_point=(1.5, -1.5, 0))

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1', 'table2']

  pb = place_body
  id = count(1)
  center = np.array([1.75, 0.25, 0])
  pb(env, box_body(env, .07, .05, .2, name='green', color=GREEN), center, 'table1')
  for dx, dy in [(-.15, 0), (.15, 0), (0, -.15), (0, .15)]:
    pb(env, box_body(env, .07, .05, .2, name='red%d'%next(id), color=RED), center + np.array([dx, dy, 0]), 'table1')

  center = np.array([.25, -1.75, 0])
  pb(env, box_body(env, .07, .05, .2, name='blue', color=BLUE), center, 'table1')
  for dx, dy in [(-.15, 0), (.15, 0), (0, -.15), (0, .15)]:
    pb(env, box_body(env, .07, .05, .2, name='red%d'%next(id), color=RED), center + np.array([dx, dy, 0]), 'table2')

  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  goal_poses = {
    'green': 'blue',
    'blue': 'green',
  }
  #for obj in object_names: # This make the problem highly non-monotonic and thus hard (still solved it though)
  #  if 'red' in obj:
  #    goal_poses[obj] = 'initial'

  return ManipulationProblem(function_name(inspect.stack()), camera_trans=camera_trans,
      object_names=object_names, table_names=table_names,
      goal_poses=goal_poses)

#######################################################################

def simple_push(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'room1.xml')

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1', 'table2']

  place_body(env, cylinder_body(env, .07, .05, name='blue_cylinder', color=BLUE), (1.65, -.6, PI/4), 'table1')
  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  start_constrained = {'blue_cylinder': 'table1'}
  goal_constrained = {
    'blue_cylinder': ('table2', Point(get_point(env.GetKinBody('blue_cylinder')) + np.array([-1.65,-1., 0.])))
    #'blue_cylinder': ('table1', Point(get_point(env.GetKinBody('blue_cylinder')) + np.array([-.2,.8, 0.])))
  }

  return ManipulationProblem(function_name(inspect.stack()),
      object_names=object_names, table_names=table_names,
      start_constrained=start_constrained,
      goal_constrained=goal_constrained)

#######################################################################

def push_wall(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'room1.xml')
  camera_trans = camera_look_at(CAMERA_POINT_SCALE*np.array([-1, 1, 3]), look_point=(3, 0, 0))

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1', 'table2']

  place_body(env, cylinder_body(env, .07, .05, name='green_cylinder', color=GREEN), (1.65, -.5, PI/4), 'table1')
  #place_body(env, box_body(env, .07, .05, .2, name='green_box', color=GREEN), (1.65, -.6, 0), 'table1')
  #set_point(env.GetKinBody('green_box'), get_point(env.GetKinBody('green_box')) + .01*unit_z())
  place_body(env, box_body(env, .05, .05, .15, name='red_box1', color=RED), (1.50, 0, 0), 'table1')
  place_body(env, box_body(env, .05, .05, .15, name='red_box2', color=RED), (1.6, 0, 0), 'table1')
  place_body(env, box_body(env, .05, .05, .15, name='red_box3', color=RED), (1.7, 0, 0), 'table1')
  place_body(env, box_body(env, .05, .05, .15, name='red_box4', color=RED), (1.8, 0, 0), 'table1')
  place_body(env, box_body(env, .05, .05, .15, name='red_box5', color=RED), (1.9, 0, 0), 'table1')
  place_body(env, box_body(env, .05, .05, .15, name='red_box6', color=RED), (2.0, 0, 0), 'table1')
  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  start_constrained = {'green_cylinder': 'table1'}
  goal_constrained = {
    'green_cylinder': ('table1', Point(get_point(env.GetKinBody('green_cylinder')) + np.array([-.2, 1.0, 0.])))
  }

  return ManipulationProblem(function_name(inspect.stack()), camera_trans=camera_trans,
      object_names=object_names, table_names=table_names,
      start_constrained=start_constrained,
      goal_constrained=goal_constrained)


#######################################################################

def stacking(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'room1.xml')
  camera_trans = camera_look_at(CAMERA_POINT_SCALE*np.array([-1.5, 1.5, 3]), look_point=(1.5, -1.5, 0))

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1', 'table2']

  regions = [
    create_region(env, 'blue_goal', ((-1, 1), (-1, 0)), 'table1', color=np.array((0, 0, 1, .5))),
    create_region(env, 'green_goal', ((-1, -.5), (-1, 1)), 'table2', color=np.array((0, 1, 0, .5)))
  ]

  pb = place_body
  pb(env, box_body(env, .07, .07, .2, name='blue_box', color=BLUE), (1.65, 0.115, PI/3), 'table1')
  pb(env, box_body(env, .07, .07, .2, name='green_box', color=GREEN), (1.55, -0.215, 7*PI/6), 'table1')
  pb(env, box_body(env, .07, .07, .2, name='purple_box', color=BLACK), (.15, -1.8, 7*PI/6), 'table2')
  pb(env, box_body(env, .07, .07, .2, name='red_box1', color=RED), (1.55, -0.215, 0), 'green_box')
  #pb(env, box_body(env, .07, .07, .2, name='red_box2', color=RED), (1.65, .6, PI/4), 'table1')
  #pb(env, box_body(env, .07, .07, .2, name='red_box3', color=RED), (.3, -1.9, PI/12), 'table2')
  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  goal_stackings = {
  #  'purple_box': 'blue_box' # target_cylinder
  }
  goal_regions = {
    'blue_box': 'blue_goal',
    'green_box': 'green_goal'
  }

  return ManipulationProblem(function_name(inspect.stack()), camera_trans=camera_trans,
      object_names=object_names, table_names=table_names, regions=regions,
      goal_stackings=goal_stackings, goal_regions=goal_regions)

#######################################################################

def move_regrasp(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'regrasp.xml')
  camera_trans = camera_look_at(CAMERA_POINT_SCALE*np.array([-1.5, 1.5, 3]), look_point=(1.5, -1.5, 0))

  pb, bb = place_body, box_body
  pb(env, bb(env, .3, .05, .3, name='obst1', color=GREY), (1.65, .075, 0), 'table1')
  pb(env, bb(env, .3, .05, .3, name='obst2', color=GREY), (1.65, .425, 0), 'table1')
  pb(env, bb(env, .05, .4, .3, name='obst3', color=GREY), (1.825, .25, 0), 'table1')

  pb(env, bb(env, .3, .05, .3, name='obst4', color=GREY), (1.65, -.125, 0), 'table1')
  pb(env, bb(env, .3, .05, .3, name='obst5', color=GREY), (1.65, -.375, 0), 'table1')
  pb(env, bb(env, .05, .3, .3, name='obst6', color=GREY), (1.825, -.25, 0), 'table1')

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1', 'table2']

  pb(env, bb(env, .03, .1, .2, name='green', color=GREEN), (1.55, 0.25, 0), 'table1')
  pb(env, bb(env, .03, .1, .2, name='blue', color=BLUE), (1.5, 0.25, 0), 'table1')
  #pb(env, bb(env, .05, .05, .1, name='red1', color=RED), (.1, -1.8, PI/16), 'table2')
  #pb(env, bb(env, .15, .05, .15, name='red2', color=RED), (-.4, -1.95, PI/5), 'table2')
  #pb(env, bb(env, .07, .07, .07, name='red3', color=RED), (.5, -1.9, PI/3), 'table2')
  #pb(env, bb(env, .1, .1, .25, name='red4', color=RED), (1.9, -0.55, PI/7), 'table1')
  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  goal_poses = {
    'green': Pose(pose_from_quat_point(quat_from_axis_angle(0, 0, PI/2), np.array([1.55, -0.25, 0.7709]))),
    'blue': 'initial'
  }

  return ManipulationProblem(function_name(inspect.stack()), camera_trans=camera_trans,
      object_names=object_names, table_names=table_names,
      goal_poses=goal_poses)

#######################################################################

def nonmonotonic(env, N=4, M=3):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'regrasp.xml')
  #env.Load(ENVIRONMENTS_DIR + '3tables.xml')
  camera_trans = camera_look_at(CAMERA_POINT_SCALE*np.array([-1.5, 1.5, 3]), look_point=(1.5, -1.5, 0))

  pb, bb = place_body, box_body
  pb(env, bb(env, .1, 1.5, .2, name='obst1', color=GREY), (1.75, 0, 0), 'table1')
  pb(env, bb(env, 1.5, .1, .2, name='obst2', color=GREY), (0, -1.75, 0), 'table2')

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1', 'table2'] #, 'table3']

  goal_poses = {}
  for i in range(N):
    displacement = -.75 + (1.5*(i+.5))/N
    value = .25 + (.75*(i+1))/N

    if i < M:
      green = 'green%s'%i
      pb(env, bb(env, .05, .1, .2, name=green, color=(0, value, 0, 1)), (1.75 - 0.1, displacement, 0), 'table1')
      goal_poses[green] = Pose(pose_from_quat_point(quat_from_axis_angle(0, 0, -PI/2), np.array([displacement, -1.75 + 0.1, 0.7709])))

    blue = 'blue%s'%i
    pb(env, bb(env, .05, .1, .2, name=blue, color=(0, 0, value, 1)), (1.75 - 0.2, displacement, 0), 'table1')
    goal_poses[blue] = 'initial'

    cyan = 'cyan%s'%i
    pb(env, bb(env, .05, .1, .2, name=cyan, color=(0, value, value, 1)), (displacement, -1.75 + 0.2, PI/2), 'table2')
    goal_poses[cyan] = 'initial'

  #pb(env, bb(env, .05, .05, .1, name='red1', color=RED), (.1, -1.8, PI/16), 'table2')
  #pb(env, bb(env, .15, .05, .15, name='red2', color=RED), (-.4, -1.95, PI/5), 'table2')
  #pb(env, bb(env, .07, .07, .07, name='red3', color=RED), (.5, -1.9, PI/3), 'table2')
  #pb(env, bb(env, .1, .1, .25, name='red4', color=RED), (1.9, -0.55, PI/7), 'table1')
  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  return ManipulationProblem(function_name(inspect.stack()), camera_trans=camera_trans,
      object_names=object_names, table_names=table_names,
      goal_poses=goal_poses)

nonmonotonic_4_1 = lambda env: nonmonotonic(env, N=4, M=1)
nonmonotonic_4_3 = lambda env: nonmonotonic(env, N=4, M=3)

#######################################################################

def srivastava_table(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + '../srivastava/good_cluttered.dae')
  camera_trans = camera_look_at((1., -3., 4), look_point=(2, 1, 0))

  body_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  dx = .5
  for body_name in body_names:
    body = env.GetKinBody(body_name)
    point = get_point(body)
    point[0] += dx
    set_point(body, point)

  table_names = [body_name for body_name in body_names if 'table' in body_name]
  object_names = [body_name for body_name in body_names if body_name not in table_names]
  for object_name in object_names:
    body = env.GetKinBody(object_name)
    point = get_point(body)
    point[2] += BODY_PLACEMENT_Z_OFFSET
    set_point(body, point)

  goal_holding = 'object1'
  goal_config = 'initial' # None

  return ManipulationProblem(function_name(inspect.stack()), camera_trans=camera_trans,
      object_names=object_names, table_names=table_names,
      goal_config=goal_config, goal_holding=goal_holding)

#######################################################################

def trivial_namo(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'navigation.xml')
  goal_config = Config(np.array((1.5, 0, PI)))
  return ManipulationProblem(function_name(inspect.stack()), goal_config=goal_config)

#######################################################################

def simple_namo(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'navigation.xml')
  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]

  place_body_on_floor(box_body(env, .1, .3, .8, name='red1', color=RED), (0, -1.5, 0))
  place_body_on_floor(box_body(env, .1, .3, .8, name='red2', color=RED), (0, 1.5, 0))
  floor_object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and \
                  str(body.GetName()) not in obstacle_names]

  goal_config = Config(np.array((1.5, 0, PI)))
  goal_holding = False # False | None
  goal_poses = {
  #  'blue': Pose(pose_from_base_values([0, 1.5, 0], z=BODY_PLACEMENT_Z_OFFSET))
  }

  return ManipulationProblem(function_name(inspect.stack()),
      floor_object_names=floor_object_names,
      goal_config=goal_config, goal_holding=goal_holding, goal_poses=goal_poses)

#######################################################################

def decision_namo(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'navigation.xml')
  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]

  place_body_on_floor(box_body(env, .1, .3, .8, name='red1', color=RED), (0, -1.5, 0))
  place_body_on_floor(box_body(env, .1, .3, .8, name='red2', color=RED), (0, 1.5, 0))
  place_body_on_floor(box_body(env, .1, .3, .8, name='red3', color=RED), (-.5, 1.5, 0))
  place_body_on_floor(box_body(env, .1, .3, .8, name='red4', color=RED), (.5, 1.5, 0))
  place_body_on_floor(box_body(env, .1, .3, .8, name='red5', color=RED), (-1, 1.5, 0))
  place_body_on_floor(box_body(env, .1, .3, .8, name='red6', color=RED), (1, 1.5, 0))

  floor_object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and \
                  str(body.GetName()) not in obstacle_names]

  goal_config = Config(np.array((1.5, 0, PI)))
  goal_holding = False # False | None
  goal_poses = {
    #'blue': Pose(pose_from_base_values([0, 1.5, 0], z=BODY_PLACEMENT_Z_OFFSET))
  }

  return ManipulationProblem(function_name(inspect.stack()),
      floor_object_names=floor_object_names,
      goal_config=goal_config, goal_holding=goal_holding, goal_poses=goal_poses)

#######################################################################

# TODO - make it so you don't have to move some things
def wall_namo(env, rows, cols): # TODO - could make variant that requires replacing things as well
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + '2tables.xml')
  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1', 'table2']

  for x in range(cols):
    for y in range(rows):
      #if y >= rows/2 and x >= cols/2: continue
      if y >= rows/2 and x >= cols - 1: continue # NOTE - makes one side harder than the other
      place_body_on_floor(box_body(env, .1, .1, .8, name='red%s-%s'%(x, y), color=RED), (-.25 + x*.75/cols, -2.5 + (y+.5)*5/rows, 0))
  place_body(env, box_body(env, .07, .07, .2, name='blue0', color=BLUE), (1.75, +0.2, 0), 'table1')
  place_body(env, box_body(env, .07, .07, .2, name='blue1', color=BLUE), (1.75, -0.2, 0), 'table1')

  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and \
                  str(body.GetName()) not in obstacle_names and 'blue' in str(body.GetName())]
  floor_object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and \
                  str(body.GetName()) not in obstacle_names and 'red' in str(body.GetName())]

  goal_config = 'initial'
  goal_holding = None # False | None
  goal_regions = {
    'blue0': 'table2',
    'blue1': 'table2',
  }

  return ManipulationProblem(function_name(inspect.stack()),
      floor_object_names=floor_object_names, object_names=object_names, table_names=table_names,
      goal_config=goal_config, goal_holding=goal_holding, goal_regions=goal_regions)

wall_namo_2 = lambda env: wall_namo(env, 8, 2)
wall_namo_3 = lambda env: wall_namo(env, 8, 3)

#######################################################################

# TODO - make it so you don't have to move some things
def decision_wall_namo(env, rows, cols): # TODO - could make variant that requires replacing things as well
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'decision_2tables.xml')
  camera_trans = camera_look_at(CAMERA_POINT_SCALE*np.array([-1.5, 1.5, 3]), look_point=(1.5, -1.5, 0))

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1', 'table2']

  for x in range(cols):
    for y in range(rows):
      #if y >= rows/2 and x >= cols/2: continue
      if y >= rows/2 and x >= cols - 1: continue # NOTE - makes one side harder than the other
      place_body_on_floor(box_body(env, .1, .1, .8, name='red%s-%s'%(x, y), color=RED), (-.25 + x*.75/cols, -2.5 + (y+.5)*5/rows, 0))
  place_body(env, box_body(env, .07, .07, .2, name='blue0', color=BLUE), (1.75, +0.2, 0), 'table1')
  place_body(env, box_body(env, .07, .07, .2, name='blue1', color=BLUE), (1.75, -0.2, 0), 'table1')

  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and \
                  str(body.GetName()) not in obstacle_names and 'blue' in str(body.GetName())]
  floor_object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and \
                  str(body.GetName()) not in obstacle_names and 'red' in str(body.GetName())]

  goal_config = 'initial'
  goal_holding = None # False | None
  goal_regions = {
    'blue0': 'table2',
    'blue1': 'table2',
  }

  return ManipulationProblem(function_name(inspect.stack()), camera_trans=camera_trans,
      floor_object_names=floor_object_names, object_names=object_names, table_names=table_names,
      goal_config=goal_config, goal_holding=goal_holding, goal_regions=goal_regions)

decision_wall_namo_2 = lambda env: decision_wall_namo(env, 8, 2)
decision_wall_namo_3 = lambda env: decision_wall_namo(env, 8, 3)

#######################################################################

def kitchen(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'kitchen.xml')

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1']
  sink_names = ['washer']
  #stove_names = ['heater']
  stove_names = ['microwave']

  regions = [
    create_region(env, 'blue_goal', ((-1, 1), (-1, -.5)), 'table1', color=np.array((0, 0, 1, .5))),
    create_region(env, 'green_goal', ((-1, 1), (.5, 1)), 'table1', color=np.array((0, 1, 0, .5)))
  ]

  pb = place_body
  pb(env, box_body(env, .07, .07, .2, name='blue', color=BLUE), (1.65, 0.115, PI/3), 'table1')
  pb(env, box_body(env, .07, .07, .2, name='green', color=GREEN), (1.55, -0.215, 7*PI/6), 'table1')
  pb(env, box_body(env, .07, .07, .2, name='red1', color=RED), (1.6, 0.615, 0), 'table1')
  pb(env, box_body(env, .07, .07, .2, name='red2', color=RED), (1.7, -0.415, PI/8), 'table1')

  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  goal_regions = {
    'blue': 'blue_goal',
    'green': 'green_goal'
  }
  goal_cleaned = ['blue']
  goal_cooked = ['green']

  return ManipulationProblem(function_name(inspect.stack()),
      object_names=object_names, table_names=table_names, sink_names=sink_names, stove_names=stove_names, regions=regions,
      goal_regions=goal_regions, goal_cleaned=goal_cleaned, goal_cooked=goal_cooked)

#######################################################################

def easy_dinner(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'dinner.xml')

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1', 'shelves', 'plate1', 'kitchen_table']
  sink_names = ['washer']
  stove_names = ['microwave']

  pb = place_body
  pb(env, box_body(env, .07, .07, .2, name='cyan2', color=CYAN), (1.7, 0.0, PI/4), 'table1')
  pb(env, box_body(env, .1, .1, .1, name='green1', color=GREEN), (-1.7, 1.8, 0), 'shelves')

  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  goal_regions = {
    'green1': 'plate1',
  }
  goal_cleaned = ['cyan2']
  goal_cooked = ['green1']

  return ManipulationProblem(function_name(inspect.stack()),
      object_names=object_names, table_names=table_names, sink_names=sink_names, stove_names=stove_names,
      goal_regions=goal_regions, goal_cleaned=goal_cleaned, goal_cooked=goal_cooked)

#######################################################################

# TODO - maybe just do top grasps?
# TODO - make shelf regions so objects can be classified as table objects
def dinner(env):
  assert not REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'dinner.xml')

  obstacle_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot()]
  table_names = ['table1', 'shelves', 'plate1', 'plate2', 'kitchen_table']
  sink_names = ['washer']
  stove_names = ['microwave']

  regions = [
    #create_region(env, 'blue_goal', ((-1, 1), (-1, -.5)), 'table1', color=np.array((0, 0, 1, .5))),
    #create_shelf_region(env, 'shelf', ((-1, 1), (-1, 1)), 'shelves', .99),
  ]

  pb = place_body
  #pb(env, box_body(env, .07, .07, .2, name='blue1', color=BLUE), (1.5, -0.4, 0), 'table1')
  #pb(env, box_body(env, .07, .07, .2, name='blue2', color=BLUE), (1.5, 0.4, 0), 'table1')
  pb(env, box_body(env, .07, .07, .2, name='blue1', color=BLUE), (1.6, -0.1, PI/3), 'table1')
  pb(env, box_body(env, .07, .07, .2, name='blue2', color=BLUE), (1.5, 0.15, PI/6), 'table1')
  #pb(env, box_body(env, .07, .07, .2, name='cyan1', color=CYAN), (1.55, -0.5, PI/5), 'table1') # NOTE - just need to be cleaned
  pb(env, box_body(env, .07, .07, .2, name='cyan2', color=CYAN), (1.7, 0.0, PI/4), 'table1')

  pb(env, box_body(env, .1, .1, .1, name='green1', color=GREEN), (-1.7, 1.8, 0), 'shelves')
  pb(env, box_body(env, .1, .1, .1, name='green2', color=GREEN), (-1.3, 1.8, 0), 'shelves')
  #set_body(env, box_body(env, .1, .1, .15, name='green1', color=GREEN), (-1.7, 1.85, .991), 0)
  #set_body(env, box_body(env, .1, .1, .15, name='green2', color=GREEN), (-1.3, 1.85, .991), 0)
  pb(env, box_body(env, .07, .07, .07, name='pink1', color=PINK), (-1.7, 1.7, 0), 'shelves')
  pb(env, box_body(env, .07, .07, .07, name='pink2', color=PINK), (-1.3, 1.7, 0), 'shelves')
  pb(env, box_body(env, .07, .07, .07, name='pink3', color=PINK), (-1.8, 1.8, 0), 'shelves')
  pb(env, box_body(env, .07, .07, .07, name='pink4', color=PINK), (-1.2, 1.8, 0), 'shelves')
  #pb(env, box_body(env, .1, .1, .1, name='red1', color=RED), (-1.5, 1.8, 0), 'shelves') # NOTE - next to green

  #pb(env, box_body(env, .07, .07, .2, name='red1', color=RED), (1.6, 0.615, 0), 'table1')
  #pb(env, box_body(env, .07, .07, .2, name='red2', color=RED), (1.7, -0.215, PI/8), 'table1')

  #set_body(env, box_body(env, .07, .07, .2, name='red3', color=RED), (-2, 0.1, 1.07), 0)
  #set_body(env, box_body(env, .07, .07, .2, name='red4', color=RED), (-2, -0.1, .79), 0)
  #set_body(env, box_body(env, .07, .07, .2, name='red3', color=RED), (-2, 0.1, .99), 0)
  #set_body(env, box_body(env, .07, .07, .2, name='red4', color=RED), (-2, -0.1, .99), 0)

  object_names = [str(body.GetName()) for body in env.GetBodies() if not body.IsRobot() and str(body.GetName()) not in obstacle_names]

  goal_poses = {
    'blue1': Pose(pose_from_quat_point(quat_from_axis_angle(0, 0, 0), np.array([1.5, -0.4, .701]))),
    'blue2': Pose(pose_from_quat_point(quat_from_axis_angle(0, 0, 0), np.array([1.5, 0.4, .701]))),
    #'blue1': 'initial',
    #'blue2': 'initial',
    'pink1': 'initial',
    'pink2': 'initial',
    'pink3': 'initial',
    'pink4': 'initial',
  }

  # TODO - food table
  # TODO - Pantry table
  goal_regions = {
    #'blue1': 'blue_goal',
    #'green1': 'green_goal',
    'green1': 'plate1',
    'green2': 'plate2',
  }
  goal_cleaned = ['blue1', 'blue2'] + ['cyan2']
  #goal_cleaned = ['blue1', 'blue2', 'cyan1', 'cyan2']
  goal_cooked = ['green1', 'green2']

  return ManipulationProblem(function_name(inspect.stack()),
      object_names=object_names, table_names=table_names, sink_names=sink_names, stove_names=stove_names, regions=regions,
      goal_poses=goal_poses, goal_regions=goal_regions, goal_cleaned=goal_cleaned, goal_cooked=goal_cooked)
