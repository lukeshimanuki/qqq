from openravepy import planningutils, PlannerStatus, RaveCreateTrajectory

from manipulation.motion.cspace import *
from motion_planners.smoothing import smooth_path
from manipulation.motion.primitives import distance_fn, sample_fn, extend_fn, collision_fn
from motion_planners.rrt_connect import birrt
from manipulation.constants import RRT_RESTARTS, RRT_ITERATIONS, RRT_SMOOTHING


# TODO - lazily compute traj_from_path because it is a little expensive. Instead work with configs

class Trajectory(object):
  def __init__(self, cspace):
    self.cspace = cspace
    self._traj = None
    self._path = None
  def traj(self):
    if self._traj is None:
      with self.cspace.body:
        self.cspace.set_active()
        self._traj = traj_from_path(self.cspace.body, self._path)
    return self._traj
  def path(self):
    if self._path is None:
      with self.cspace.body:
        self.cspace.set_active()
        self._path = [self.start()] + list(flatten(linear_interpolation(self.cspace.body, q1, q2) for q1, q2 in pairs(self.waypoints())))
    return self._path
  def config(self, data):
    return self.cspace.config(data, get_spec(self.traj()))
  def samples(self, time_step=0.1):
    return map(self.config, get_samples(self.traj(), time_step=time_step))
  def smooth(self, vel_multi=3, accel_multi=3): # Acceleration and velocity multipliers
    with self.cspace.body:
      self.cspace.set_active()
      # TODO - more systematically speed up base motions
      #print self.cspace.body.GetActiveDOFMaxAccel()
      #print self.cspace.body.GetActiveDOFMaxVel()
      if planningutils.SmoothActiveDOFTrajectory(self.traj(), self.cspace.body,
            maxvelmult=vel_multi, maxaccelmult=accel_multi) == PlannerStatus.HasSolution:
        return TrajTrajectory(self.cspace, self.traj())
    return None
  def resample(self):
    return TrajTrajectory(self.cspace, self.traj())
  def shortcut(self):
    with self.cspace.body:
      self.cspace.set_active()
      return PathTrajectory(self.cspace, smooth_path(self.path(), extend_fn(self.cspace.body),
                                                     collision_fn(get_env(), self.cspace.body)))
  def replan(self, restarts=RRT_RESTARTS, iterations=RRT_ITERATIONS, smooth=RRT_SMOOTHING):
    body = self.cspace.body
    with body:
      self.cspace.set_active()
      path = birrt(self.start(), self.end(), distance_fn(body), sample_fn(body), extend_fn(body), collision_fn(get_env(), body),
                   restarts=restarts, iterations=iterations, smooth=smooth)
      if path is None: return None
      return PathTrajectory(self.cspace, path)
  def waypoints(self):
    return map(self.config, get_waypoints(self.traj()))
  def step(self, sequence=None):
    if sequence is None: sequence = self.path()
    with self.cspace.body:
      self.cspace.set_active()
      for i, config in enumerate(sequence):
        self.cspace.body.SetActiveDOFValues(config)
        raw_input('Finish' if i == len(sequence) - 1 else 'Step %d'%(i+1))
  def execute(self):
    self.cspace.body.GetController().SetPath(self.traj())
    self.cspace.body.WaitForController(0)
  #def __getstate__(self): # https://docs.python.org/2/library/pickle.html
  #  return
  #def __setstate__(self, state):
  #  pass
  def __repr__(self):
    return self.__class__.__name__ + '(' + str(self.cspace) + ')'
  @staticmethod
  def join(trajs): # Remove repeated configs?
    if len(trajs) == 0: return None
    return PathTrajectory(trajs[0].cspace, list(flatten(traj.path() for traj in trajs)))

class TrajTrajectory(Trajectory):
  def __init__(self, cspace, traj):
    super(self.__class__, self).__init__(cspace)
    self._traj = traj
  def start(self):
    return self.config(get_start(self.traj()))
  def end(self):
    return self.config(get_end(self.traj()))
  def reverse(self):
    return self.__class__(self.cspace, reverse_traj(self.traj()))

class PathTrajectory(Trajectory):
  def __init__(self, cspace, path):
    super(self.__class__, self).__init__(cspace)
    self._path = path
  def start(self):
    return self.path()[0]
  def end(self):
    return self.path()[-1]
  def reverse(self):
    return self.__class__(self.cspace, self.path()[::-1])

#################################################################

def new_traj():
  return RaveCreateTrajectory(get_env(), '')

def get_start(traj):
  return traj.GetWaypoint(0)

def get_end(traj):
  return traj.GetWaypoint(-1)

def get_waypoints(traj):
  return [traj.GetWaypoint(i) for i in range(traj.GetNumWaypoints())]

def get_samples(traj, time_step=0.1):
  return [traj.Sample(t) for t in np.arange(0, traj.GetDuration(), time_step)]

def get_spec(traj):
  return traj.GetConfigurationSpecification()

def convert_traj_spec(traj, spec): # Mutates traj
  planningutils.ConvertTrajectorySpecification(traj, spec)

def traj_from_path(body, path, vel_multi=1, accel_multi=1):
  traj = new_traj()
  spec = body.GetActiveConfigurationSpecification()
  traj.Init(spec)
  traj.Insert(traj.GetNumWaypoints(), np.array(path).flatten(), spec)
  status = planningutils.RetimeActiveDOFTrajectory(traj, body, hastimestamps=False, # hastimestamps must be false
                                                   maxvelmult=vel_multi, maxaccelmult=accel_multi) # NOTE - kwargs don't work unless ordered correctly
  #status = planningutils.RetimeActiveDOFTrajectory(traj, body, False, 1., 1., 'lineartrajectoryretimer', '') # TODO - parabolictrajectoryretimer
  #status = planningutils.RetimeAffineTrajectory(traj, np.array([1, 1, 1]), np.array([1, 1, 1]))
  #status = RetimeTrajectory(traj)
  if status == PlannerStatus.HasSolution:
    return traj
  return None

def smoothed_traj_from_path(body, path):
  traj = new_traj()
  spec = body.GetActiveConfigurationSpecification()
  traj.Init(spec)
  traj.Insert(traj.GetNumWaypoints(), np.array(path).flatten(), spec)
  status = planningutils.SmoothActiveDOFTrajectory(traj, body, maxvelmult=1, maxaccelmult=1)
  #status = planningutils.SmoothActiveDOFTrajectory(traj, body, 1., 1., 'lineartrajectoryretimer', '') # TODO - parabolictrajectoryretimer
  #status = planningutils.SmoothAffineTrajectory(traj, np.array((1., 1., .1)), np.array((1., 1., .1)), 'lineartrajectoryretimer', '')
  #status = planningutils.SmoothTrajectory(traj, 1., 1., 'parabolictrajectoryretimer', '') # TODO - parabolictrajectoryretimer
  if status == PlannerStatus.HasSolution:
    return traj
  return None

def reverse_traj(traj):
  return planningutils.ReverseTrajectory(traj)

def simultaneous_trajs(trajs):
  return planningutils.MergeTrajectories(trajs)

