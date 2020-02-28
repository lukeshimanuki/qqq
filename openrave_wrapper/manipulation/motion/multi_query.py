from single_query import cspace_traj
from manipulation.bodies.robot import open_gripper, grab, release, object_pose_from_robot_config
from cspace import CSpace
from manipulation.primitives.transforms import get_trans, base_values_from_trans

def plan_base_traj(oracle, start_config, end_config, holding=None, obstacle_poses={}):
  with oracle.state_saver():
    oracle.set_all_object_poses(obstacle_poses)
    oracle.set_robot_config(end_config)
    base_trans = get_trans(oracle.robot)

    oracle.set_robot_config(start_config)
    open_gripper(oracle)
    if holding is not None:
      oracle.set_pose(holding.object_name, object_pose_from_robot_config(oracle, start_config, holding.grasp))
      grab(oracle, holding.object_name)
    traj = cspace_traj(oracle, CSpace.robot_base(oracle.robot), base_values_from_trans(base_trans))
    if holding is not None:
      release(oracle, holding.object_name)
  return traj
