from collections import namedtuple
import os
import sys

from misc.utils import SEPARATOR, INF, sleep
from openravepy.misc import SetViewerUserThread
from manipulation.execute import StepExecutable, Executable
from manipulation.primitives.display import is_viewer_active, save_image, update_viewer
from manipulation.bodies.robot import object_pose_from_robot_config, open_gripper
from manipulation.primitives.savers import GrabSaver
from primitives.utils import set_robot_config
from manipulation.constants import DRAW_REGIONS, DRAW_GOALS, PRE_SMOOTH_TRAJECTORIES, MIN_DELTA, REPLAN_TRAJECTORIES
from manipulation.motion.trajectories import traj_from_path

# TODO - override keyword arguments only if specified with some value. Otherwise use called method's arguments
# TODO - figure out python keyword argument spacing style convention.
# TODO - method that pulls the default arguments if None is specified.
# TODO - change object colors for some visualizations

Plan = namedtuple('Plan', ['start', 'operators'])

def execute_viewer(env, execute):
  if sys.platform == 'darwin': # NOTE - problem with OpenRAVE on OS X
    SetViewerUserThread(env, 'qtcoin', execute)
  else:
    env.SetViewer('qtcoin')
    execute()

def openrave_debug_fn(oracle):
  def fn(vertex):
    oracle.unlock()
    print '{state_space}{vertex}\n'.format(state_space=vertex.state_space, vertex=vertex)
    if DRAW_REGIONS: oracle.draw_regions()
    if DRAW_GOALS: oracle.draw_goals()
    with oracle.state_saver():
      set_state(vertex.state, oracle)
      oracle.env.UpdatePublishedBodies()
      raw_input('Hit enter to continue')
    print
    oracle.lock()
  return fn

def improvement_debug_fn(oracle):
  h_history = []
  if DRAW_REGIONS: oracle.draw_regions()
  if DRAW_GOALS: oracle.draw_goals()
  def fn(vertex):
    global h_min
    print '{state_space}{vertex}\n'.format(state_space=vertex.state_space, vertex=vertex)
    if len(h_history) != 0 and vertex.h_cost >= h_history[-1]:
      return
    print SEPARATOR
    h_history.append(vertex.h_cost)
    oracle.unlock()
    with oracle.state_saver():
      set_state(vertex.state, oracle)
      oracle.env.UpdatePublishedBodies()
      #raw_input('Hit enter to continue')
    oracle.lock()
  return fn

###########################################################################

def set_state(state, oracle):
  if state['robot'] is not False:
    set_robot_config(oracle.robot, state['robot'])
  for object_name in oracle.objects:
    if state[object_name] is not False:
      oracle.set_pose(object_name, state[object_name])
  if state['holding'] is not False:
    holding = state['holding']
    oracle.set_pose(holding.object_name, object_pose_from_robot_config(oracle, oracle.get_robot_config(), holding.grasp))

###########################################################################

def smooth_actions(plan, oracle): # TODO - move somewhere else
  #oracle.env.SetCollisionChecker(RaveCreateCollisionChecker(oracle.env, 'pqp'))
  sys.setrecursionlimit(10000) # NOTE - the recursion stack is a little small for retracing paths
  oracle.env.GetCollisionChecker().SetCollisionOptions(0)
  oracle.l_model.setRobotResolutions(xyzdelta=MIN_DELTA/10) # TODO - make this 10
  state = plan.start
  with GrabSaver(oracle.robot):
    set_state(state, oracle)
    #with collision_saver(oracle.env, openravepy_int.CollisionOptions.UseTolerance): # Just reports tolerances
    for i, action in enumerate(plan.operators):
      print 'Smoothing action %d'%(i+1)
      if hasattr(action, 'base_trajs'):
        with oracle.robot: # TODO - do I need to collision save the holding as well?
          if REPLAN_TRAJECTORIES:
            action.base_trajs = [traj.replan(restarts=INF, iterations=50, smooth=50) for traj in action.base_trajs]
          else:
            action.base_trajs = [traj.resample().shortcut() for traj in action.base_trajs] # NOTE - need to resample before shortcutting
          for traj in action.base_trajs:
            # Speed up base trajectories
            assert traj._traj is None
            traj.cspace.set_active()
            traj._traj = traj_from_path(traj.cspace.body, traj._path, vel_multi=5, accel_multi=5)
      if hasattr(action, 'pap'):
        action.pap.smooth(oracle, action.object_name)

      if isinstance(action, StepExecutable):
        for _ in action.step(oracle): pass
      else:
        state = action(state)

def execute_plan(plan, oracle, pause=True):
  if PRE_SMOOTH_TRAJECTORIES:
    smooth_actions(plan, oracle)

  def execute():
    oracle.unlock()
    oracle.problem.set_viewer(oracle.env)
    oracle.clear_regions()
    oracle.draw_goals()
    #oracle.env.StartSimulation(time_step, realtime=False) # realtime=False calls simulate step as fast as possible
    set_state(plan.start, oracle)
    raw_input('Hit enter to finish' if len(plan.operators) == 0 else 'Hit enter to start')
    for i, action in enumerate(plan.operators):
      if isinstance(action, Executable):
        action.execute(oracle)
      if i == len(plan.operators) - 1: raw_input('Hit enter to finish')
      elif pause: raw_input('Hit enter to continue')

  if is_viewer_active(oracle.env): execute()
  else: execute_viewer(oracle.env, execute)

###########################################################################

IMAGE_FORMAT = '%simage%03d-%02d.jpg'

def visualize_plan(plan, oracle, display=True, save=None):
  if not display and save is None: return
  if save is not None and not os.path.exists(save): os.makedirs(save)

  def execute():
    oracle.unlock()
    oracle.problem.set_viewer(oracle.env)
    oracle.clear_regions()
    oracle.draw_goals()
    set_state(plan.start, oracle)
    open_gripper(oracle) # TODO - do I always want this?
    update_viewer()
    if display: raw_input('Hit enter to finish' if len(plan.operators) == 0 else 'Hit enter to step')
    else: sleep(1) # NOTE - Gives time for the viewer to update
    if save is not None: save_image(oracle.env, IMAGE_FORMAT%(save, 0, 0)) # TODO - sometimes captures images before the viewer is ready
    for i, action in enumerate(plan.operators):
      if isinstance(action, StepExecutable):
        for j, _ in enumerate(action.step(oracle)):
          update_viewer()
          if display: raw_input('Hit enter to finish' if i == len(plan.operators) - 1 else 'Hit enter to step')
          else: sleep(1) # Gives time for the viewer to update
          if save is not None: save_image(oracle.env, IMAGE_FORMAT%(save, i+1, j)) # TODO - sometimes captures images before the viewer is ready

  if is_viewer_active(oracle.env): execute()
  else: execute_viewer(oracle.env, execute)

###########################################################################

def visualize_states(states, oracle, display=True, save=None):
  if not display and save is None: return
  if save is not None and not os.path.exists(save): os.makedirs(save)

  def execute():
    oracle.unlock()
    oracle.problem.set_viewer(oracle.env)
    oracle.clear_regions()
    oracle.draw_goals()
    for i, state in enumerate(states):
      set_state(state, oracle)
      oracle.env.UpdatePublishedBodies() # Updates display if env locked
      if display: raw_input('Hit enter to finish' if i == len(states) - 1 else 'Hit enter to step')
      else: sleep(1) # Gives time for the viewer to update
      if save is not None: save_image(oracle.env, (save + 'image%0' + str(len(str(len(states) - 1))) + 'd.jpg') % i) # Sometimes captures images before the viewer is ready

  if is_viewer_active(oracle.env): execute()
  else: execute_viewer(oracle.env, execute)
