from manipulation.collision.collision_primitives import collision, robot_collision, gripper_collision
from manipulation.bodies.robot import *
from manipulation.primitives.savers import *
from manipulation.bodies.bounding_volumes import aabb_stacked, aabb_min, aabb_max, fast_aabb_overlap
from manipulation.constants import MIN_DELTA
from openravepy import openravepy_int

# TODO - use aabb/spheres more aggressively (I did so for the EdgeCaches)

class RegionContainsCache(DeterminisiticCache):
  def __init__(self, oracle):
    super(self.__class__, self).__init__()
    self.oracle = oracle
  def key(self, region_name, object_name, object_pose):
    return (region_name, self.oracle.body_name_to_geom_hash[object_name], object_pose)
  def fn(self, region_name, object_name, object_pose):
    return self.oracle.regions[region_name].contains(self.oracle.get_aabb(object_name, trans_from_pose(object_pose.value)))

class AreStackedCache(DeterminisiticCache):
  def __init__(self, oracle):
    super(self.__class__, self).__init__()
    self.oracle = oracle
  def key(self, object_name, object_pose, base_name, base_pose):
    return (self.oracle.get_geom_hash(object_name), object_pose, self.oracle.get_geom_hash(base_name), base_pose)
  def fn(self, object_name, object_pose, base_name, base_pose):
    return aabb_stacked(self.oracle.get_aabb(object_name, trans_from_pose(object_pose.value)),
                        self.oracle.get_aabb(base_name, trans_from_pose(base_pose.value)))

class OnEdgeCache(DeterminisiticCache):
  def __init__(self, oracle):
    super(self.__class__, self).__init__()
    self.oracle = oracle
  def key(self, region_name, object_name, object_pose):
    return (region_name, self.oracle.get_geom_hash(object_name), object_pose)
  def fn(self, region_name, object_name, object_pose):
    return self.oracle.get_region(region_name).on_edge(self.oracle.get_aabb(object_name, trans_from_pose(object_pose.value)))

class GetAABBCache(DeterminisiticCache):
  def __init__(self, oracle):
    super(self.__class__, self).__init__()
    self.oracle = oracle
  def fn(self, geom_hash, pose):
    oracle = self.oracle
    body_name = oracle.geom_hash_to_body_name[geom_hash]
    with oracle.body_saver(body_name):
      oracle.set_pose(body_name, pose)
      return aabb_from_body(oracle.bodies[body_name])

#################################################################

class ObjectCollisionCache(DeterminisiticCache):
  def __init__(self, oracle):
    super(self.__class__, self).__init__()
    self.oracle = oracle
  def key(self, body_name1, pose1, body_name2, pose2):
    return (self.oracle.body_name_to_geom_hash[body_name1], pose1, self.oracle.body_name_to_geom_hash[body_name2], pose2)
  def fn(self, body_name1, pose1, body_name2, pose2):
    oracle = self.oracle

    if not oracle.necessary_collision(body_name1, pose1, body_name2, pose2): return False

    with oracle.body_saver(body_name1):
      with oracle.body_saver(body_name2):
        oracle.set_pose(body_name1, pose1)
        oracle.set_pose(body_name2, pose2)
        return collision(oracle, body_name1, body_name2)

class ApproachCollisionCache(DeterminisiticCache):
  def __init__(self, oracle):
    super(self.__class__, self).__init__()
    self.oracle = oracle
  def key(self, body_name, pose, grasp):
    return (self.oracle.get_geom_hash(body_name), pose, grasp)
  def fn(self, body_name, pose, grasp):
    oracle = self.oracle
    manip_trans, vector = manip_from_pose_grasp(pose, grasp)
    body_trans, gripper_trans = trans_from_pose(pose.value), gripper_trans_from_manip_trans(oracle, manip_trans)
    distance, direction = length(vector), normalize(vector)
    steps = int(max(ceil(distance/MIN_DELTA), 1)) + 1

    gripper = oracle.robot.GetActiveManipulator().GetEndEffector() # NOTE - the gripper is the red thing inside the hand
    with oracle.robot:
      with oracle.body_saver(body_name):
        for dist in (s*distance/steps for s in reversed(range(steps))):
          set_trans(oracle.get_body(body_name), vector_trans(body_trans, dist*direction))
          set_trans(gripper, vector_trans(gripper_trans, dist*direction))
          if any(gripper_collision(oracle, gripper, obst_name) for obst_name in oracle.obstacles) or \
              any(collision(oracle, body_name, obst_name) for obst_name in oracle.obstacles):
            return True
    return False

class ObjApproachCollisionCache(DeterminisiticCache):
  def __init__(self, oracle):
    super(self.__class__, self).__init__()
    self.oracle = oracle
  def key(self, body_name1, pose1, grasp, body_name2, pose2):
    return (self.oracle.get_geom_hash(body_name1), pose1, grasp, self.oracle.get_geom_hash(body_name2), pose2)
  """def fn(self, body_name1, pose1, grasp, body_name2, pose2):
    oracle = self.oracle
    manip_trans, vector = manip_from_pose_grasp(oracle, pose1, grasp)
    distance, direction = length(vector), normalize(vector)
    steps = int(max(ceil(distance/MIN_DELTA), 1))
    with oracle.body_saver(body_name1):
      with oracle.body_saver(body_name2):
        oracle.set_pose(body_name2, pose2)
        for dist in [s*distance/steps for s in reversed(range(steps+1))]:
          oracle.set_pose(body_name1, Pose(pose_from_trans(vector_trans(manip_trans, dist*direction))))
          if collision(oracle, body_name1, body_name2): # TODO - consider using gripper as well
            return True
    return False"""
  def fn(self, body_name1, pose1, grasp, body_name2, pose2):
    oracle = self.oracle
    manip_trans, vector = manip_from_pose_grasp(pose1, grasp)
    distance, direction = length(vector), normalize(vector)
    steps = int(max(ceil(distance/MIN_DELTA), 1)) + 1
    poses = [Pose(pose_from_trans(vector_trans(trans_from_pose(pose1.value), direction*s*distance/steps))) for s in reversed(range(steps))]

    # USE OBJECT COLLISION CACHE?

    if not any(oracle.necessary_collision(body_name1, pose, body_name2, pose2) for pose in poses): return False

    with oracle.body_saver(body_name1):
      with oracle.body_saver(body_name2):
        oracle.set_pose(body_name2, pose2)
        for pose in poses:
          oracle.set_pose(body_name1, pose)
          if collision(oracle, body_name1, body_name2): # TODO - consider using gripper as well
            return True
    return False

#################################################################

class HoldingCollisionCache(DeterminisiticCache):
  def __init__(self, oracle):
    super(self.__class__, self).__init__()
    self.oracle = oracle
  def key(self, config, object_name, object_pose, holding):
    if holding is None:
      return (config, self.oracle.body_name_to_geom_hash[object_name], object_pose)
    return (config, self.oracle.body_name_to_geom_hash[object_name], object_pose, self.oracle.body_name_to_geom_hash[holding.object_name], holding.grasp)
  def fn(self, config, object_name, object_pose, holding):
    oracle = self.oracle
    if not hasattr(config, 'preprocessed'):
      config.processed = True
      with oracle.robot_saver():
        oracle.set_robot_config(config)
        config.saver = oracle.robot_saver()
        config.manip_trans = get_manip_trans(oracle)
        config.aabbs = [(aabb_min(aabb), aabb_max(aabb)) for aabb in [oracle.compute_part_aabb(part) for part in USE_ROBOT_PARTS]]

    trans = object_trans_from_manip_trans(config.manip_trans, holding.grasp.grasp_trans)
    distance = oracle.get_radius2(object_name) + oracle.get_radius2(holding.object_name)
    check_holding = length2(point_from_trans(trans) - point_from_pose(object_pose.value)) <= distance

    object_aabb = oracle.get_aabb(object_name, trans_from_pose(object_pose.value))
    object_aabb_min, object_aabb_max = aabb_min(object_aabb), aabb_max(object_aabb)
    check_robot = fast_aabb_overlap(object_aabb_min, object_aabb_max, config.aabbs)
    if not check_holding or not check_robot: return False

    with oracle.body_saver(object_name):
      oracle.set_pose(object_name, object_pose)
      if check_robot:
        with oracle.robot_saver():
          with collision_saver(oracle.env, openravepy_int.CollisionOptions.ActiveDOFs):
            config.saver.Restore()
            if robot_collision(oracle, object_name):
              return True
      if check_holding:
        with oracle.body_saver(holding.object_name):
          set_trans(oracle.get_body(holding.object_name), trans)
          if collision(oracle, object_name, holding.object_name):
            return True
    return False

#################################################################

USE_ROBOT_PARTS = ['not_left', 'left_hand', 'left_forearm', 'left_upper_arm', 'left_shoulder']
# TODO - distinguish between base and arm trajectories

# TODO - move this preprocessing to pick_and_place
def preprocess_trajs(oracle, start_config, trajs): # TODO - should store start_config, trajs tuple
  if any(not hasattr(traj, 'preprocessed') for traj in trajs):
    with oracle.robot_saver():
      set_full_config(oracle.robot, start_config.value) # TODO - get rid of this or something
      for traj in trajs:
        traj.preprocessed = True
        traj.cspace.set_active()
        traj.savers = [] # TODO - nasty
        traj.manip_trans = []
        traj.aabbs = []
        for q in traj.path():
          traj.cspace.body.SetActiveDOFValues(q)
          traj.savers.append(oracle.robot_saver())
          traj.manip_trans.append(get_manip_trans(oracle))
          traj.aabbs.append([(aabb_min(aabb), aabb_max(aabb)) for aabb in [oracle.compute_part_aabb(part) for part in USE_ROBOT_PARTS]])

class TrajCollisionCache(DeterminisiticCache):
  def __init__(self, oracle):
    super(self.__class__, self).__init__()
    self.oracle = oracle
  def key(self, start_config, trajs, object_name, object_pose):
    return (start_config, trajs, self.oracle.body_name_to_geom_hash[object_name], object_pose)
  def fn(self, start_config, trajs, object_name, object_pose):
    oracle = self.oracle
    preprocess_trajs(oracle, start_config, trajs)

    object_aabb = oracle.get_aabb(object_name, trans_from_pose(object_pose.value))
    object_aabb_min, object_aabb_max = aabb_min(object_aabb), aabb_max(object_aabb)
    savers = [traj.savers[i] for traj in trajs for i, aabbs in enumerate(traj.aabbs)
               if fast_aabb_overlap(object_aabb_min, object_aabb_max, aabbs)]
    if len(savers) == 0: return False

    with oracle.body_saver(object_name):
      oracle.set_pose(object_name, object_pose)
      with oracle.robot_saver():
        trajs[0].cspace.set_active()
        with collision_saver(oracle.env, openravepy_int.CollisionOptions.ActiveDOFs):
          for saver in savers:
            saver.Restore()
            if robot_collision(oracle, object_name):
              return True
    return False

class TrajHoldingCollisionCache(DeterminisiticCache):
  def __init__(self, oracle):
    super(self.__class__, self).__init__()
    self.oracle = oracle
  def key(self, start_config, trajs, object_name, object_pose, holding):
    return (start_config, trajs, self.oracle.body_name_to_geom_hash[object_name], object_pose,
            self.oracle.body_name_to_geom_hash[holding.object_name], holding.grasp)
  def fn(self, start_config, trajs, object_name, object_pose, holding):
    oracle = self.oracle
    if oracle.traj_collision(start_config, trajs, object_name, object_pose): return True

    all_trans = [object_trans_from_manip_trans(manip_trans, holding.grasp.grasp_trans) for traj in trajs for manip_trans in traj.manip_trans] # TODO - cache these transforms
    distance = oracle.get_radius2(object_name) + oracle.get_radius2(holding.object_name)
    manip_trans = [trans for trans in all_trans if length2(point_from_trans(trans) - point_from_pose(object_pose.value)) <= distance]
    if len(manip_trans) == 0: return False

    with oracle.body_saver(object_name):
      oracle.set_pose(object_name, object_pose)
      with oracle.body_saver(holding.object_name):
        for trans in manip_trans:
          set_trans(oracle.get_body(holding.object_name), trans)
          if collision(oracle, object_name, holding.object_name):
            return True
    return False
