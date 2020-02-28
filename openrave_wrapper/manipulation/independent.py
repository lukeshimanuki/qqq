from manipulation.bodies.bodies import geometry_hash, get_name
from manipulation.grasps.grasps import Grasp

from manipulation.motion.cspace import CSpace
from manipulation.bodies.bounding_volumes import aabb_from_body
from manipulation.primitives.transforms import get_pose
from manipulation.bodies.robot import manip_from_pose_grasp
from manipulation.primitives.transforms import set_config
from manipulation.primitives.inverse_kinematics import inverse_kinematics_helper, solve_inverse_kinematics_exhaustive
from manipulation.bodies.robot import get_active_arm_indices
from manipulation.grasps.grasp_options import get_grasp_options
from manipulation.constants import GRASP_TYPES
from manipulation.primitives.utils import Pose
from manipulation.motion.single_query import motion_plan, workspace_traj_helper
from manipulation.bodies.robot import get_manipulator
from manipulation.primitives.transforms import unit_pose, set_pose, arm_from_full_config, base_values_from_full_config
from manipulation.primitives.utils import Config

from openravepy import Sensor, RaveCreateCollisionChecker
from openravepy import databases, interfaces
from openravepy import IkParameterization

# TODO - incorporate any additional methods from tamp_problems.py or motion_problems.py

def check_env_collision(env, body):
  return env.CheckCollision(body)

def check_pairwise_collision(env, body1, body2):
  return env.CheckCollision(body1, body2)

def check_self_collision(body):
  return body.CheckSelfCollision()

def check_end_effector_collision(manipulator, transform):
  return manipulator.CheckEndEffectorCollision(transform)
  # CheckIndependentCollision

class HackedOracle(object):
  def __init__(self, env):
    self.env = env
    self.robot = env.GetRobots()[0]
  def get_aabb(self, body_name):
    body = self.env.GetKinBody(body_name)
    with body: # NOTE - before I didn't have this and it worked for rlplan
      set_pose(body, unit_pose())
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
    grasps.append(Grasp(grasp_tform, None, None, approach_vector)) # TODO - grasp incomplete
  return grasps

def get_grasps(env, robot, body, grasp_approach, grasp_type):
  Options = get_grasp_options(body, grasp_approach)
  grasp_options = Options.default(grasp_type, body)
  print 'Creating', get_name(body), GRASP_TYPES.names[grasp_type], 'database' # TODO - move this to create_grasp_database
  return hacked_create_grasp_database(env, grasp_options)

def load_pr2(env):
  robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
  env.AddRobot(robot)
  return robot

def get_base_conf(conf):
  return Config(base_values_from_full_config(conf.value))

def get_arm_conf(manipulator, conf):
  return Config(arm_from_full_config(manipulator, conf.value))

def initialize_openrave(env, manipulator_name, min_delta=.01, collision_checker='ode'):
  env.StopSimulation()
  for sensor in env.GetSensors():
    sensor.Configure(Sensor.ConfigureCommand.PowerOff)
    sensor.Configure(Sensor.ConfigureCommand.RenderDataOff)
    sensor.Configure(Sensor.ConfigureCommand.RenderGeometryOff)
    env.Remove(sensor)
  env.SetCollisionChecker(RaveCreateCollisionChecker(env, collision_checker))
  env.GetCollisionChecker().SetCollisionOptions(0)

  robot = env.GetRobots()[0]
  cd_model = databases.convexdecomposition.ConvexDecompositionModel(robot)
  if not cd_model.load(): cd_model.autogenerate()
  l_model = databases.linkstatistics.LinkStatisticsModel(robot)
  if not l_model.load(): l_model.autogenerate()
  l_model.setRobotWeights()
  l_model.setRobotResolutions(xyzdelta=min_delta) # xyzdelta is the minimum Cartesian distance of an object

  robot.SetActiveManipulator(manipulator_name) # NOTE - Need this or the manipulator computations are off
  #extrema = aabb_extrema(aabb_union([aabb_from_body(body) for body in env.GetBodies()])).T
  #robot.SetAffineTranslationLimits(*extrema)

  ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot, iktype=IkParameterization.Type.Transform6D,
      forceikfast=True, freeindices=None, freejoints=None, manip=None)
  if not ikmodel.load(): ikmodel.autogenerate()

  cspace = CSpace.robot_arm(robot.GetActiveManipulator())
  cspace.set_active()

def feasible_pick_place(env, robot, obj, grasps, approach_config, do_motion_planning):
  base_manip = interfaces.BaseManipulation(robot, plannername=None, maxvelmult=None)
  pose = Pose(get_pose(obj))
  for grasp in grasps:
    manip_trans, approach_vector = manip_from_pose_grasp(pose, grasp)

    set_config(robot, approach_config, robot.GetActiveManipulator().GetArmIndices())
    if env.CheckCollision(robot) or robot.CheckSelfCollision():
      continue

    grasp_config = inverse_kinematics_helper(env, robot, manip_trans) # NOTE - maybe need to find all IK solutions
    #grasp_config = solve_inverse_kinematics_exhaustive(env, robot, manip_trans)
    if grasp_config is None:
      continue
    set_config(robot, grasp_config, get_active_arm_indices(robot))
    #approach_config = get_full_config(robot)

    traj, arm_traj = None, None
    if do_motion_planning:
      #traj = vector_traj_helper(env, robot, approach_vector)
      traj = workspace_traj_helper(base_manip, approach_vector)
      if traj is None:
        continue
      set_config(robot, traj.end(), get_active_arm_indices(robot))
      #vector_config = oracle.get_robot_config()
      arm_traj = motion_plan(env, CSpace.robot_arm(get_manipulator(robot)), approach_config, self_collisions=True)
      if arm_traj is None:
        continue
    return grasp_config, traj, arm_traj
  return None
