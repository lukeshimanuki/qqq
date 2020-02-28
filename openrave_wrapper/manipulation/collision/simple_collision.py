from manipulation.collision.collision_primitives import collision, robot_collision, gripper_collision
from manipulation.bodies.robot import *
from manipulation.primitives.savers import *
from manipulation.bodies.bounding_volumes import aabb_stacked, aabb_min, aabb_max, fast_aabb_overlap
from manipulation.constants import MIN_DELTA
from openravepy import openravepy_int

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


def traj_collision(self, start_config, trajs, object_name, object_pose):
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

def traj_holding_collision(self, start_config, trajs, object_name, object_pose, holding):
  oracle = self.oracle
  if oracle.traj_collision(start_config, trajs, object_name, object_pose):
    return True

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