import numpy as np

from random import choice
from manipulation.bodies.bodies import box_body
from manipulation.grasps.grasps import get_grasps
from manipulation.primitives.placements import random_region_placements
from manipulation.bodies.bounding_volumes import points_from_aabb, xy_points_from_aabb, \
  aabb_min, aabb_max, aabb_apply_trans, point_from_trans, point_in_aabb
from manipulation.primitives.display import draw_point, draw_line
from manipulation.primitives.transforms import get_trans, point_from_trans, trans_from_base_values, base_values_from_trans
from misc.generators import take, Counter
from misc.numerical import INF

def pap_ir_oracle(env): # TODO - vary object and table geometry
  from manipulation.problems.fixed import ENVIRONMENTS_DIR # NOTE - prevents circular imports
  from manipulation.oracle import ManipulationOracle
  from manipulation.problems import ManipulationProblem
  env.Load(ENVIRONMENTS_DIR + 'empty.xml')
  table_name = 'table'
  #env.Add(box_body(env, 1., 1., .75, name=table_name)) # NOTE - .75 is too tall for top grasps
  env.Add(box_body(env, 1., 1., .7, name=table_name))
  body_name = 'body'
  env.Add(box_body(env, .07, .07, .2, name=body_name))
  return ManipulationOracle(ManipulationProblem(None, object_names=[body_name],
      table_names=[table_name]), env, preload_databases=False, debug=False)

def pap_ir_samples(env, max_failures=100, max_attempts=INF): # NOTE - max_failures should be large to prevent just easy placements
  from manipulation.inverse_reachability.inverse_reachability import openrave_base_iterator, create_custom_ir
  from manipulation.pick_and_place import PickAndPlace
  oracle = pap_ir_oracle(env)
  body_name = oracle.objects[0]
  table_name = oracle.tables[0]

  num_samples = Counter()
  for i, pose in enumerate(take(random_region_placements(oracle, body_name, [table_name]), max_attempts)):
    if i % 10 == 0:
      print 'IR sampling iteration:', i, '| samples:', num_samples
    grasp = choice(get_grasps(oracle, body_name))
    pap = PickAndPlace(None, pose, grasp)
    if pap.sample(oracle, body_name, base_iterator_fn=openrave_base_iterator,
                  max_failures=max_failures, check_base=False):
      yield pap.manip_trans, pap.base_trans
      next(num_samples)

def pap_ir_statistics(env, trials=100):
  from manipulation.inverse_reachability.inverse_reachability import display_custom_ir, load_ir_database, \
    ir_base_trans, forward_transform, manip_base_values, is_possible_fr_trans, is_possible_ir_trans
  from manipulation.pick_and_place import PickAndPlace
  oracle = pap_ir_oracle(env)
  body_name = oracle.objects[0]
  table_name = oracle.tables[0]

  convert_point = lambda p: np.concatenate([p[:2], [1.]])
  """
  load_ir_database(oracle)
  print oracle.ir_aabb
  print aabb_min(oracle.ir_aabb), aabb_max(oracle.ir_aabb)
  handles = []
  vertices = xy_points_from_aabb(oracle.ir_aabb)
  print vertices
  print np.min(oracle.ir_database, axis=0), np.max(oracle.ir_database, axis=0)
  for v in vertices:
    handles.append(draw_point(oracle.env, convert_point(v)))
  for v1, v2 in zip(vertices, vertices[1:] + vertices[-1:]):
    handles.append(draw_line(oracle.env, convert_point(v1), convert_point(v2)))
  raw_input('Start?')
  """

  successes = []
  for pose in take(random_region_placements(oracle, body_name, [table_name]), trials):
    grasp = choice(get_grasps(oracle, body_name))
    pap = PickAndPlace(None, pose, grasp)
    oracle.set_pose(body_name, pap.pose)
    #if pap.sample(oracle, body_name, max_failures=50, base_iterator_fn=openrave_base_iterator, check_base=False):
    if pap.sample(oracle, body_name, max_failures=50, check_base=False):
      oracle.set_robot_config(pap.grasp_config)
      successes.append(int(pap.iterations))
    handles = display_custom_ir(oracle, pap.manip_trans)

    default_trans = get_trans(oracle.robot)
    #vertices = xy_points_from_aabb(aabb_apply_trans(oracle.ir_aabb, pap.manip_trans))
    vertices = [point_from_trans(ir_base_trans(oracle.robot, pap.manip_trans, v, default_trans))
                for v in xy_points_from_aabb(oracle.ir_aabb)]
    for v1, v2 in zip(vertices, vertices[1:] + vertices[-1:]):
      handles.append(draw_line(oracle.env, convert_point(v1), convert_point(v2)))

    point_from_inverse = lambda trans: point_from_trans(np.dot(pap.base_trans, forward_transform(trans)))
    for trans in oracle.ir_database:
      handles.append(draw_point(oracle.env, convert_point(point_from_inverse(trans)), color=(0, 1, 0, .5)))
    #vertices = [point_from_inverse(v) for v in xy_points_from_aabb(oracle.ir_aabb)] # Doesn't have the angle in it...
    vertices = [point_from_trans(np.dot(pap.base_trans, trans_from_base_values(v))) for v in xy_points_from_aabb(oracle.fr_aabb)]
    for v1, v2 in zip(vertices, vertices[1:] + vertices[-1:]):
      handles.append(draw_line(oracle.env, convert_point(v1), convert_point(v2), color=(0, 1, 0, .5)))

    assert is_possible_ir_trans(oracle, pap.manip_trans, pap.base_trans) and \
      is_possible_fr_trans(oracle, pap.base_trans, pap.manip_trans)
    raw_input('Continue?')
  return float(len(successes))/trials, np.mean(successes), np.std(successes)
