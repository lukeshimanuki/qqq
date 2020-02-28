from manipulation.bodies.bounding_volumes import aabb_min, aabb_max, fast_aabb_overlap
from manipulation.collision.collision_primitives import robot_collision
from manipulation.motion.cspace import CSpace
from manipulation.bodies.robot import get_manip_trans, object_trans_from_manip_trans
from manipulation.primitives.savers import collision_saver
from manipulation.primitives.transforms import trans_from_pose, point_from_trans, point_from_pose, length2, set_trans
from misc.caches import DeterminisiticCache
from manipulation.collision.collision_primitives import collision
from openravepy import openravepy_int

USE_ROBOT_PARTS = ['not_left', 'left_hand', 'left_forearm', 'left_upper_arm', 'left_shoulder']

def preprocess_edge(oracle, edge):
  if not hasattr(edge, 'preprocessed'):
    edge.preprocessed = True
    with oracle.robot_saver():
      CSpace.robot_arm_and_base(oracle.robot.GetActiveManipulator()).set_active()
      for q in edge.configs():
        if not hasattr(q, 'saver'):
          oracle.robot.SetActiveDOFValues(q.value)
          q.saver = oracle.robot_saver()
          q.manip_trans = get_manip_trans(oracle)
          q.aabbs = [(aabb_min(aabb), aabb_max(aabb)) for aabb in [oracle.compute_part_aabb(part) for part in USE_ROBOT_PARTS]]

class EdgeCollisionCache(DeterminisiticCache):
  def __init__(self, oracle):
    super(self.__class__, self).__init__()
    self.oracle = oracle
  def key(self, edge, object_name, object_pose):
    return (edge, self.oracle.get_geom_hash(object_name), object_pose)
  def fn(self, edge, object_name, object_pose):
    oracle = self.oracle
    preprocess_edge(oracle, edge)

    object_aabb = oracle.get_aabb(object_name, trans_from_pose(object_pose.value))
    object_aabb_min, object_aabb_max = aabb_min(object_aabb), aabb_max(object_aabb)
    configs = [q for q in edge.configs() if fast_aabb_overlap(object_aabb_min, object_aabb_max, q.aabbs)]
    if len(configs) == 0: return False

    #distance = oracle.get_radius2('pr2') + oracle.get_radius2(object_name)
    #z = get_point(oracle.robot)[-1]
    #configs = [q for q in edge.configs() if length2(np.array([q.value[-3], q.value[-2], z]) - point_from_pose(object_pose.value)) <= distance]
    #if len(configs) == 0: return False

    # TODO - oracle.robot.SetActiveDOFValues(q.value) is the bottleneck: check each object at the same time
    with oracle.body_saver(object_name):
      oracle.set_pose(object_name, object_pose)
      with oracle.robot_saver():
        CSpace.robot_arm_and_base(oracle.robot.GetActiveManipulator()).set_active()
        with collision_saver(oracle.env, openravepy_int.CollisionOptions.ActiveDOFs): # TODO - does this really do what I think?
          for q in configs: # TODO - do this with other other collision functions
            q.saver.Restore()
            if robot_collision(oracle, object_name):
              return True
    return False

class EdgeHoldingCollisionCache(DeterminisiticCache):
  def __init__(self, oracle):
    super(self.__class__, self).__init__()
    self.oracle = oracle
  def key(self, edge, object_name, object_pose, holding):
    return (edge, self.oracle.get_geom_hash(object_name), object_pose, self.oracle.get_geom_hash(holding.object_name), holding.grasp)
  def fn(self, edge, object_name, object_pose, holding):
    oracle = self.oracle
    preprocess_edge(oracle, edge)

    all_trans = [object_trans_from_manip_trans(q.manip_trans, holding.grasp.grasp_trans) for q in edge.configs()] # TODO - cache these transforms
    distance = oracle.get_radius2(object_name) + oracle.get_radius2(holding.object_name)
    nearby_trans = [trans for trans in all_trans if length2(point_from_trans(trans) - point_from_pose(object_pose.value)) <= distance]
    if len(nearby_trans) == 0: return False

    with oracle.body_saver(object_name):
      oracle.set_pose(object_name, object_pose)
      with oracle.body_saver(holding.object_name):
        for trans in nearby_trans:
          set_trans(oracle.get_body(holding.object_name), trans)
          if collision(oracle, object_name, holding.object_name):
            return True
    return False