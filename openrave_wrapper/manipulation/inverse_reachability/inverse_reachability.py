from manipulation.bodies.robot import *
from manipulation.bodies.bodies import get_robot_hash
from manipulation.constants import DEBUG, FORCE_CREATE_IR, GRASP_APPROACHES, USE_GRASP_APPROACH, ONLY_UNDER_GRASPS, \
  SAMPLE_ARM_TRAJ, SAMPLE_VECTOR_TRAJ, IR_DATABASE_SAMPLES
from openravepy import databases, planning_error, Environment
from manipulation.primitives.display import draw_point
from manipulation.primitives.transforms import trans_from_base_values, get_trans, trans_from_pose, \
  base_values_from_trans, trans2D_from_trans, point_from_trans
from random import choice
from itertools import cycle
from math import atan2
from misc.numerical import INF, positive_hash
from misc.io import read_pickle, write_pickle
from misc.generators import take
from manipulation.inverse_reachability.pick_and_place import pap_ir_samples
from manipulation.inverse_reachability.push import push_ir_samples
from manipulation.bodies.bounding_volumes import aabb_from_points, point_in_aabb
import numpy as np

FILENAME = parent_dir(abs_path(__file__), 2) + '/databases/inverse_reachability/inverse_reachability.%d.pickle'

# TODO - handle rightarm by reflecting the distribution

def get_manip_angle(robot, manip_trans, grasp_approach): # TODO - find a better way to do this
  if grasp_approach == GRASP_APPROACHES.TOP:
    direction = vertical_manip_direction(robot, manip_trans)
  else:
    direction = forward_manip_direction(robot, manip_trans)
  return atan2(direction[1], direction[0])

# m^-1 * b = t
def is_possible_ir_trans(oracle, manip_trans, base_trans):
  load_ir_database(oracle)
  return point_in_aabb(oracle.ir_aabb, base_values_from_trans(np.dot(
    np.linalg.inv(manip_base_values(oracle.robot, manip_trans)), base_trans)))

# t^-1 = b^-1 * m
def is_possible_fr_trans(oracle, base_trans, manip_trans):
  load_ir_database(oracle)
  return point_in_aabb(oracle.fr_aabb, base_values_from_trans(
    np.dot(np.linalg.inv(base_trans), manip_base_values(oracle.robot, manip_trans))))

def manip_base_values(robot, manip_trans, grasp_approach=USE_GRASP_APPROACH):
  return trans_from_base_values([manip_trans[0, 3], manip_trans[1, 3], get_manip_angle(robot, manip_trans, grasp_approach)])

# b = m * t
def ir_base_trans(robot, manip_trans, transform, default_trans):
  manip_trans2D = manip_base_values(robot, manip_trans)
  transform_trans2D = trans_from_base_values(transform)
  base_trans2D = np.dot(manip_trans2D, transform_trans2D)
  base_trans = default_trans.copy()
  base_trans[:2, :2] = base_trans2D[:2, :2]
  base_trans[:2, 3] = base_trans2D[:2, 3]
  return base_trans

# b * t^-1 = m
def forward_transform(transform):
  return np.linalg.inv(trans_from_base_values(transform))

def get_base_generator(robot, ir_database, manip_iterator):
  default_trans = get_trans(robot)
  for manip_trans in cycle(manip_iterator):
    yield ir_base_trans(robot, manip_trans, choice(ir_database), default_trans), manip_trans

def load_ir_database(oracle):
  if not hasattr(oracle, 'ir_database'):
    oracle.ir_database = get_custom_ir(oracle.robot) # TODO - allow multiple IR databases
    oracle.ir_aabb = aabb_from_points(np.array(oracle.ir_database).T)
    fr_database = [base_values_from_trans(forward_transform(ir_trans)) for ir_trans in oracle.ir_database]
    oracle.fr_aabb = aabb_from_points(np.array(fr_database).T)
    oracle.reachability_radius = np.max(np.linalg.norm(np.array(oracle.ir_database)[:,:2], axis=1))
    # TODO - use an oobb instead
  #return oracle.ir_database

def custom_base_iterator(oracle, manip_iterator):
  load_ir_database(oracle)
  return get_base_generator(oracle.robot, oracle.ir_database, manip_iterator)

def display_custom_ir(oracle, manip_trans):
  load_ir_database(oracle)
  handles = []
  default_trans = get_trans(oracle.robot)
  for transform in oracle.ir_database:
    base_trans = ir_base_trans(oracle.robot, manip_trans, transform, default_trans)
    handles.append(draw_point(oracle.env, point_from_trans(base_trans)))
  return handles

#################################################################

def get_ir_hash(robot): # TODO - specify manipulation type (pap, push, etc)
    return positive_hash((get_robot_hash(robot), USE_GRASP_APPROACH,
                 ONLY_UNDER_GRASPS, SAMPLE_VECTOR_TRAJ, SAMPLE_ARM_TRAJ))

def get_custom_ir(robot):
  try:
    if FORCE_CREATE_IR: raise IOError
    ir_database = load_custom_ir(robot)
  except IOError:
    ir_database = create_custom_ir(robot, pap_ir_samples(Environment())) # TODO - temporarily set viewer to the new Environment?
    #ir_database = create_custom_ir(robot, push_ir_database(Environment())) # TODO - options for doing push etc
    save_custom_ir(robot, ir_database) # TODO - should hypothetically pass the new robot in
  return ir_database

def create_custom_ir(robot, manip_base_iterator, n=IR_DATABASE_SAMPLES):
  if DEBUG: print 'Creating inverse reachability database'
  ir_database = []
  for manip_trans, base_trans in take(manip_base_iterator, n):
    manip_trans2D = manip_base_values(robot, manip_trans)
    base_trans2D = trans2D_from_trans(base_trans)
    ir_database.append(base_values_from_trans(np.linalg.solve(manip_trans2D, base_trans2D))) # M * T = B
  return ir_database

def load_custom_ir(robot):
  filename = FILENAME%get_ir_hash(robot)
  ir_database = read_pickle(filename)
  if DEBUG: print 'Loaded', filename
  return ir_database

def save_custom_ir(robot, ir_database):
  filename = FILENAME%get_ir_hash(robot)
  write_pickle(filename, ir_database)
  if DEBUG: print 'Saved', filename

#################################################################

def openrave_base_iterator(oracle, manip_iterator, random_bases=True):
  if not hasattr(oracle, 'ir_model'):
    oracle.ir_model = get_openrave_ir(oracle, random_bases)
  index_manip_iterator = ((manip_trans, manip_trans) for manip_trans in manip_iterator)
  try:
    for base_pose, manip_trans, _ in oracle.ir_model.randomBaseDistributionIterator(index_manip_iterator) if random_bases else \
        oracle.ir_model.sampleBaseDistributionIterator(index_manip_iterator, logllthresh=-INF, Nprematuresamples=1):
      yield trans_from_pose(base_pose), manip_trans
  except (ValueError, planning_error): # ValueError: low >= high because of an empty sequence
    raise StopIteration

def get_openrave_ir(oracle, random_bases=False):
  ir_model = databases.inversereachability.InverseReachabilityModel(oracle.robot)
  if not random_bases and not ir_model.load():
    if DEBUG: print 'Creating openrave inverse reachability model'
    ir_model.autogenerate()
    if DEBUG: print 'Saved openrave inverse reachability model'
  else:
    if DEBUG: print 'Loaded openrave inverse reachability model'
  return ir_model
