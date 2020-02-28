from openravepy import planning_error, openravepy_int

from manipulation.primitives.inverse_kinematics import *
from manipulation.motion.trajectories import *
from manipulation.constants import *
from manipulation.motion.trajectories import TrajTrajectory, PathTrajectory

# step_length is the max step length for each tree extension

# GetModules so I can just reuse the existing one

def cspace_traj_helper(base_manip, cspace, goal, step_length=MIN_DELTA, max_iterations=10, max_tries=1):
  with base_manip.robot.GetEnv():
    cspace.set_active()
    with collision_saver(base_manip.robot.GetEnv(), openravepy_int.CollisionOptions.ActiveDOFs):
      try:
        return TrajTrajectory(cspace, base_manip.MoveActiveJoints(goal=goal, steplength=step_length, maxiter=max_iterations, maxtries=max_tries,
            execute=False, outputtraj=None, goals=None, outputtrajobj=True, jitter=None, releasegil=False, postprocessingplanner=None,
            postprocessingparameters=None))
      except planning_error:
        return None

def cspace_traj(oracle, cspace, goal, **kwargs):
  return cspace_traj_helper(oracle.base_manip, cspace, **kwargs)

####################

def manip_traj_helper(base_manip, goal, step_length=MIN_DELTA, max_iterations=10, max_tries=1): # start_config=None
  with base_manip.robot:
    cspace = CSpace.robot_arm(base_manip.robot.GetActiveManipulator())
    cspace.set_active()
    with collision_saver(base_manip.robot.GetEnv(), openravepy_int.CollisionOptions.ActiveDOFs):
      try: # ~12 iterations/second
        return TrajTrajectory(cspace, base_manip.MoveManipulator(goal=goal,
            maxiter=max_iterations, execute=False, outputtraj=None, maxtries=max_tries,
            goals=None, steplength=step_length, outputtrajobj=True, jitter=None, releasegil=False))
      except planning_error:
        return None

def manip_traj(oracle, goal, manip_name=None, **kwargs): # start_config=None
  with oracle.robot:
    if manip_name is not None: oracle.base_manip.robot.SetActiveManipulator(manip_name)
    return manip_traj_helper(oracle.base_manip, goal, **kwargs)

####################

# NOTE - steps relates into the number of analytical IK calls. 10 seems to be a magic number.
def workspace_traj_helper(base_manip, vector, steps=10): # TODO - use IK to check if even possibly valid
  distance, direction = length(vector), normalize(vector)
  step_length = distance/steps
  with base_manip.robot:
    cspace = CSpace.robot_arm(base_manip.robot.GetActiveManipulator())
    cspace.set_active()
    with collision_saver(base_manip.robot.GetEnv(), openravepy_int.CollisionOptions.ActiveDOFs):
      try: # TODO - Bug in specifying minsteps. Need to specify at least 2 times the desired otherwise it stops early
        return TrajTrajectory(cspace, base_manip.MoveHandStraight(direction, minsteps=10*steps, maxsteps=steps, steplength=step_length,
            ignorefirstcollision=None, starteematrix=None, greedysearch=True, execute=False, outputtraj=None, maxdeviationangle=None,
            planner=None, outputtrajobj=True))
      except planning_error:
        return None

def workspace_traj(oracle, vector, manip_name=None, **kwargs): # TODO - use IK to check if even possibly valid
  with oracle.robot:
    if manip_name is not None: oracle.robot.SetActiveManipulator(manip_name)
    return workspace_traj_helper(oracle.base_manip, vector, **kwargs)

#################################################################

def linear_arm_traj_helper(env, robot, end_config, manip_name=None):
  with robot:
    if manip_name is not None:
      robot.SetActiveManipulator(manip_name)
    robot.SetActiveDOFs(get_active_arm_indices(robot))
    with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
      start_config = get_active_config(robot)
      extend = extend_fn(robot)
      collision = collision_fn(env, robot)
      path = [start_config] + list(extend(start_config, end_config))
      if any(collision(q) for q in path): return None
      return PathTrajectory(CSpace.robot_arm(robot.GetActiveManipulator()), path)

def linear_arm_traj(oracle, end_config, manip_name=None):
  return linear_arm_traj_helper(oracle.env, oracle.robot, end_config, manip_name=manip_name)

def vector_traj_helper(env, robot, vector, manip_name=None):
  with robot:
    if manip_name is not None:
      robot.SetActiveManipulator(manip_name)
    robot.SetActiveDOFs(get_active_arm_indices(robot))
    with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
      end_config = inverse_kinematics_helper(env, robot, vector_trans(get_trans(robot.GetActiveManipulator()), vector))
      if end_config is None:
        return None
      return linear_arm_traj_helper(env, robot, end_config, manip_name=manip_name)

def vector_traj(oracle, vector, manip_name=None):
  return vector_traj_helper(oracle.env, oracle.robot, vector, manip_name=manip_name)

#################################################################

def motion_plan(env, cspace, goal, planner=birrt, self_collisions=False):
  robot = cspace.body
  with robot:
    cspace.set_active()
    start = robot.GetActiveDOFValues()
    with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
      path = planner(start, goal, distance_fn(robot), sample_fn(robot), extend_fn(robot), collision_fn(env, robot, check_self=self_collisions))
      if path is None: return None
      return PathTrajectory(cspace, path)

def birrt_wrapper(env, cspace, goal, restarts,iterations,smooth,self_collisions=False):
  robot = cspace.body
  with robot:
    cspace.set_active()
    start = robot.GetActiveDOFValues()
    with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
      path = birrt(start, goal, distance_fn(robot), sample_fn(robot), extend_fn(robot), collision_fn(env, robot, check_self=self_collisions),restarts=restarts,iterations=iterations,smooth=smooth)
      if path is None: return None
      return PathTrajectory(cspace, path)



def constrained_motion_plan(oracle, goal):
  with oracle.robot_saver():
    trajectory = motion_plan(oracle.env, CSpace.robot_arm_and_base(oracle.robot.GetActiveManipulator()),
        goal, planner=lambda *args: birrt(*args, restarts=INF, iterations=100, smooth=25))
    trajectory.smooth()
    return trajectory
