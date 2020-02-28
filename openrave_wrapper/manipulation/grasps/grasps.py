from manipulation.grasps.grasp_options import *
from manipulation.collision.collision import robot_collision
from manipulation.bodies.robot import open_gripper, get_manip_trans
from manipulation.motion.trajectories import TrajTrajectory, new_traj
from manipulation.motion.cspace import CSpace
from manipulation.constants import FORCE_CREATE_GRASPS, GRASP_TYPES, GRASP_APPROACHES, \
  USE_GRASP_TYPE, USE_GRASP_APPROACH, DEBUG, ONLY_UNDER_GRASPS, MAX_GRASPS
from misc.io import abs_path

FILENAME = parent_dir(abs_path(__file__), 2) + '/databases/grasps/grasp.%d.pickle'

Holding = namedtuple('Holding', ['object_name', 'grasp'])

class Grasp(object):
  _ids = count(0)
  def __init__(self, grasp_trans, gripper_config, gripper_traj, approach_vector):
    self.id = next(self._ids)
    self.grasp_trans = grasp_trans
    self.gripper_config = gripper_config
    self.gripper_traj = gripper_traj
    self.approach_vector = approach_vector
  def __str__(self):
    return self.__class__.__name__ + '(%d)'%self.id # str_object((self.grasp_trans[0:3,3], self.gripper_config))
  __repr__ = __str__

def get_grasps(oracle, body_name, grasp_approach=USE_GRASP_APPROACH, grasp_type=USE_GRASP_TYPE):
  if not hasattr(oracle, 'grasp_database'):
    oracle.grasp_database = {}

  grasp_key = (oracle.get_geom_hash(body_name), grasp_approach, grasp_type)
  if grasp_key not in oracle.grasp_database:
    Options = get_grasp_options(oracle.bodies[body_name], grasp_approach)
    grasp_options = Options.default(grasp_type, oracle.bodies[body_name])

    filename = FILENAME%positive_hash(grasp_options) # TODO - get rid of the negative numbers
    try:
      if FORCE_CREATE_GRASPS: raise IOError
      oracle.grasp_database[grasp_key] = load_grasp_database(oracle.robot, filename)
    except IOError:
      if DEBUG: print 'Creating', body_name, GRASP_TYPES.names[grasp_type], 'database' # TODO - move this to create_grasp_database
      oracle.grasp_database[grasp_key] = create_grasp_database(oracle, grasp_options)
      save_grasp_database(filename, oracle.grasp_database[grasp_key])

    # TODO - find a better way to do this
    if ONLY_UNDER_GRASPS and grasp_approach == GRASP_APPROACHES.SIDE:
      oracle.grasp_database[grasp_key] = filter(lambda g: g.grasp_trans[0,3] > 0, oracle.grasp_database[grasp_key])
    if MAX_GRASPS != INF:
      oracle.grasp_database[grasp_key] = oracle.grasp_database[grasp_key][:MAX_GRASPS]
  return randomize(oracle.grasp_database[grasp_key])

# TODO - automatically pickle OpenRave objects
def load_grasp_database(robot, filename):
  grasps = []
  for grasp_tform, gripper_config, (trajectory, indices), approach_vector in read_pickle(filename):
    grasps.append(Grasp(np.array(grasp_tform), np.array(gripper_config),
      TrajTrajectory(CSpace(robot, indices=np.array(indices)), new_traj().deserialize(trajectory)), approach_vector))
  if DEBUG: print 'Loaded', filename
  return grasps

def save_grasp_database(filename, grasps):
  data = []
  for grasp in grasps:
    data.append((grasp.grasp_trans.tolist(), grasp.gripper_config.tolist(),
        (grasp.gripper_traj.traj().serialize(), grasp.gripper_traj.cspace.indices.tolist()), grasp.approach_vector)) # trajectory.Write(0) deprecated
  write_pickle(filename, data)
  if DEBUG: print 'Saved', filename

def create_grasp_database(oracle, grasp_options):
  grasps = []
  create_grasp = collision_free_grasp if grasp_options.collisions else collision_ignorant_grasp
  for grasp_tform, approach_vector in grasp_options.grasp_trans(oracle):
    grasp = create_grasp(oracle, grasp_options.geom_hash, grasp_tform, approach_vector)
    if grasp is not None:
      grasps.append(grasp)
  return grasps

#################################################################

def collision_free_grasp(oracle, geom_hash, grasp_tform, approach_vector, manip_name=None):
  body_name = oracle.get_body_name(geom_hash)
  with oracle.state_saver():
    if manip_name is not None: oracle.robot.SetActiveManipulator(manip_name)
    oracle.set_active_state(active_obstacles=False, active_objects=set([body_name]))
    open_gripper(oracle)
    oracle.robot.SetDOFValues(oracle.default_left_arm_config, oracle.robot.GetActiveManipulator().GetArmIndices())
    set_trans(oracle.bodies[body_name], object_trans_from_manip_trans(get_manip_trans(oracle), grasp_tform))
    if not robot_collision(oracle, body_name):
      gripper_config, gripper_traj = oracle.task_manip.CloseFingers(offset=None, movingdir=None, execute=False, outputtraj=False, outputfinal=True,
          coarsestep=None, translationstepmult=None, finestep=None, outputtrajobj=True)
      return Grasp(grasp_tform, gripper_config, TrajTrajectory(CSpace(oracle.robot,
          indices=oracle.robot.GetActiveManipulator().GetGripperIndices()), gripper_traj), approach_vector)
  return None

def collision_ignorant_grasp(oracle, geom_hash, grasp_tform, approach_vector, manip_name=None): # TODO - try to make a valid grasp if possible
  with oracle.state_saver():
    if manip_name is not None: oracle.robot.SetActiveManipulator(manip_name)
    oracle.set_active_state(active_obstacles=False, active_objects=set())
    open_gripper(oracle)
    oracle.robot.SetDOFValues(oracle.default_left_arm_config, oracle.robot.GetActiveManipulator().GetArmIndices())
    gripper_config, gripper_traj = oracle.task_manip.CloseFingers(offset=None, movingdir=None, execute=False, outputtraj=False, outputfinal=True,
        coarsestep=None, translationstepmult=None, finestep=None, outputtrajobj=True)
    return Grasp(grasp_tform, gripper_config, TrajTrajectory(CSpace(oracle.robot,
        indices=oracle.robot.GetActiveManipulator().GetGripperIndices()), gripper_traj), approach_vector)
