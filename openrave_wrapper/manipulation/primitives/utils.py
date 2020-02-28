from openravepy.misc import *
from manipulation.primitives.transforms import *
from manipulation.bodies.geometry import *

class Config(object): # http://docs.scipy.org/doc/numpy/user/basics.subclassing.html
  _ids = count(0)
  def __init__(self, value, indices=None): # TODO - body, CSpace, etc
    self.id = self._ids.next()
    self.value = value
    self.indices = None
  def __repr__(self):
    return self.__class__.__name__ + '(%d)'%self.id #'[%d]'%len(self.value)

class ManipVector(object):
  _ids = count(0)
  def __init__(self, manip_trans, approach_vector):
    self.id = next(self._ids)
    self.manip_trans = manip_trans
    self.approach_vector = approach_vector
  def __repr__(self):
    return self.__class__.__name__ + '(%d)'%self.id #str_object(self.value[4:7])

class Wrapper(object):
  def __init__(self, value):
    self.id = next(self._ids)
    self.value = value
  def __repr__(self):
    return self.__class__.__name__ + '(%d)'%self.id #str_object(self.value[4:7])

class Point(Wrapper): _ids = count(0)
class Pose(Wrapper): _ids = count(0)
class Transform(Wrapper): _ids = count(0)

###########################################################################

def get_env():
  return RaveGetEnvironments()[0]

def extract_arm_and_base_config(oracle, q):
  return Config(arm_and_base_from_full_config(oracle.robot.GetActiveManipulator(), q.value))

def traj_arm_and_base_configs(reference_config, traj):
  return [Config(np.concatenate([q, reference_config.value[-3:]])) for q in traj.path()]

def mirror_arm_config(config):
  robot = get_env().GetRobots()[0]
  signs = np.array([1 if left_min == right_min else -1 for left_min, right_min in zip(robot.GetDOFLimits(robot.GetManipulator('leftarm').GetArmIndices())[0],
      robot.GetDOFLimits(robot.GetManipulator('rightarm').GetArmIndices())[0])])
  return signs*config

def set_robot_config(robot, config): # TODO - store something in the config so I don't have to guess this
  if len(config.value) == 3:
    set_base_values(robot, config.value)
  elif len(config.value) == 7:
    set_config(robot, config.value, robot.GetActiveManipulator().GetArmIndices())
  elif len(config.value) == 10:
    set_config(robot, config.value[:7], robot.GetActiveManipulator().GetArmIndices())
    set_base_values(robot, config.value[-3:])
  else:
    set_full_config(robot, config.value)

def get_sample_time(operator): # TODO - assign sample_time to all operators manually
  if hasattr(operator, 'sample_time'):
    return operator.sample_time
  if hasattr(operator, 'pap'):
    return operator.pap.sample_time
  if hasattr(operator, 'push'):
    return operator.push.sample_time
  return 0

###########################################################################

def reverse_trajectories(trajs):
  return [traj.reverse() for traj in reversed(trajs)]

def smooth_trajectories(trajs):
  for traj in trajs:
    traj.smooth()
  return trajs

def identity_trajectories(trajs):
  return trajs

def execute_trajectories(trajs, time_step=None):
  if time_step is None:
    get_env().StartSimulation(0.001) # NOTE - time_step is ineffective when realtime is False
  else:
    get_env().StartSimulation(time_step, realtime=False) # TODO - causes segmentation faults when using realtime
  for traj in trajs:
    traj.execute()
  #get_env().StopSimulation()

#def step_execute_trajectories(trajs, time_step=0.1):
#  #env.StopSimulation()
#  for traj in trajs:
#    traj.execute_traj()
#    for _ in irange(traj.trajectory.GetDuration(), step=time_step):
#      env.StepSimulation(time_step)

def step_trajectories(trajs, sequence):
  for traj in trajs:
    traj.step(sequence=sequence)

def step_sample_trajectories(trajs, time_step=0.1):
  step_trajectories(trajs, lambda t: t.samples(time_step=time_step))

def step_waypoint_trajectories(trajs):
  step_trajectories(trajs, lambda t: t.waypoints())

def step_path_trajectories(trajs):
  step_trajectories(trajs, lambda t: t.path())

###########################################################################

# API and reference material
# http://openrave.org/docs/latest_stable/openravepy/openravepy/
# http://openrave.org/docs/latest_stable/openravepy/openravepy_int/
# http://openrave.org/docs/latest_stable/geometric_conventions/
# http://openrave.org/docs/latest_stable/tutorials/openravepy_beginning/
# http://openrave.org/docs/latest_stable/tutorials/openravepy_examples/
# http://openrave.org/docs/latest_stable/interface_types/#interface-planner
# http://openrave.programmingvision.com/wiki/index.php/Format:XML
# http://www.openrave.org/docs/0.6.6/coreapihtml/namespaceOpenRAVE.html