from misc.utils import arg_info, currentframe, get_path, get_directory
from manipulation.primitives.display import set_viewer_options
from manipulation.constants import VIEWER_NAME, CAMERA_TRANS
import numpy as np
import inspect

PROBLEMS_DIR = get_directory(get_path(inspect.currentframe()))
GENERATED_DIR = PROBLEMS_DIR + '/generated/'
EXTENSION = '.pickle'

BLACK = np.array((0, 0, 0, 0))
GREY = np.array((.5, .5, .5, 0))
WHITE = np.array((1, 1, 1, 0))

RED = np.array((1, 0, 0, 0))
GREEN = np.array((0, 1, 0, 0))
BLUE = np.array((0, 0, 1, 0))

YELLOW = np.array((1, 1, 0, 0))
PINK = np.array((1, 0, 1, 0))
CYAN = np.array((0, 1, 1, 0))

class ManipulationProblem(object):
  def __init__(self, name, env_file=None, camera_trans=CAMERA_TRANS,
      object_names=[], floor_object_names=[], # TODO - list which objects can be grasped in what ways/constrained
      table_names=[], sink_names=[], stove_names=[], regions=[],
      start_holding=False, start_stackings={}, start_constrained={},
      goal_config=None, goal_holding=None, goal_poses={}, goal_regions={}, goal_stackings={}, goal_constrained={},
      goal_cleaned=[], goal_cooked=[],
      known_poses=None, initial_poses=None, grasps=None):
    for k, v in arg_info(currentframe(), ignore=['self']).items():
      setattr(self, k, v)
  def load_env(self, env):
    if self.env_file is not None:
      env.Load(GENERATED_DIR + self.env_file)
  def set_viewer(self, env):
    #set_viewer_options(env, name=VIEWER_NAME+': %s'%self.name, camera_trans=CAMERA_TRANS)
    set_viewer_options(env, name=VIEWER_NAME+': %s'%self.name, camera_trans=self.camera_trans)
  def __str__(self):
    s = self.__class__.__name__ + '\n'
    for name, value in self.__dict__.iteritems():
      s += name + ': %s\n'%str(value)
    return s
  __repr__ = __str__
