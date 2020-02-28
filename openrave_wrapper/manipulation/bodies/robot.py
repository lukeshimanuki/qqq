from manipulation.primitives.utils import *
from manipulation.constants import APPROACH_DISTANCE, HOLDING_LEFT_ARM, REST_LEFT_ARM

def set_default_robot_config(robot): # TODO - fix this
  robot.SetDOFValues(HOLDING_LEFT_ARM, robot.GetManipulator('leftarm').GetArmIndices())
  robot.SetDOFValues(np.array([0.54800022]), robot.GetManipulator('leftarm').GetGripperIndices())
  robot.SetDOFValues(mirror_arm_config(REST_LEFT_ARM), robot.GetManipulator('rightarm').GetArmIndices())
  robot.SetDOFValues(np.array([0.0]), robot.GetManipulator('rightarm').GetGripperIndices())
  robot.SetDOFValues([.15], [robot.GetJointIndex('torso_lift_joint')])
  #set_trans(robot, unit_trans())

def get_manipulator(robot):
  return robot.GetActiveManipulator()

def get_active_arm_indices(robot):
  return get_manipulator(robot).GetArmIndices()

def get_arm_indices(oracle):
  return get_active_arm_indices(oracle.robot)

def get_end_effector_trans(oracle):
  return get_trans(oracle.robot.GetActiveManipulator().GetEndEffector())

def get_manip_trans(oracle):
  return get_trans(oracle.robot.GetActiveManipulator()) # = oracle.robot.GetActiveManipulator().GetEndEffectorTransform()
# np.dot(oracle.get_end_effector_trans(), oracle.robot.GetActiveManipulator().GetLocalToolTransform()) = oracle.get_manipulator_trans()

def manip_trans_from_arm_config(oracle, arm_config):
  with oracle.robot:
    set_config(oracle.robot, arm_config, get_arm_indices(oracle))
    return get_manip_trans(oracle)

def manip_trans_from_full_config(oracle, full_config):
  with oracle.robot:
    set_full_config(oracle.robot, full_config)
    return get_manip_trans(oracle)

def object_pose_from_current_config(oracle, grasp):
  return Pose(pose_from_trans(object_trans_from_manip_trans(get_manip_trans(oracle), grasp.grasp_trans)))

def object_pose_from_robot_config(oracle, config, grasp):
  oracle.set_robot_config(config)
  return object_pose_from_current_config(oracle, grasp)

#################################################################

def set_gripper2(manipulator, values):
  manipulator.GetRobot().SetDOFValues(values, manipulator.GetGripperIndices())

def grasp_gripper2(manipulator, grasp):
  set_gripper2(manipulator, grasp.gripper_config)

def open_gripper2(manipulator):
  _, upper = manipulator.GetRobot().GetDOFLimits(manipulator.GetGripperIndices())
  set_gripper2(manipulator, upper)

def close_gripper2(manipulator):
  lower, _ = manipulator.GetRobot().GetDOFLimits(manipulator.GetGripperIndices())
  set_gripper2(manipulator, lower)

#################################################################

def open_gripper(oracle):
  oracle.robot.SetDOFValues(np.array([0.54800022]), oracle.robot.GetActiveManipulator().GetGripperIndices())

def close_gripper(oracle):
  oracle.robot.SetDOFValues(np.array([0.0]), oracle.robot.GetActiveManipulator().GetGripperIndices())

def set_gripper(oracle, grasp):
  oracle.robot.SetDOFValues(grasp.gripper_config, oracle.robot.GetActiveManipulator().GetGripperIndices())

def grab(oracle, body_name): # Keeps the manipulator and the object at the current relative transform
  oracle.robot.Grab(oracle.bodies[body_name])

def release(oracle, body_name):
  oracle.robot.Release(oracle.bodies[body_name])

def release_all(oracle):
  oracle.robot.ReleaseAllGrabbed()

def is_grabbing(oracle, body_name):
  return oracle.robot.IsGrabbing(oracle.bodies[body_name])

def get_grabbed(oracle):
  return oracle.robot.GetGrabbed()

def get_non_manipulator_links(manipulators):
  return reduce(operator.__and__, [set(m.GetIndependentLinks()) for m in manipulators])

def get_manipulator_links(manipulator): # Arm and hand
  return set(manipulator.GetRobot().GetLinks())-set(manipulator.GetIndependentLinks())

def get_arm_links(manipulator):
  return get_manipulator_links(manipulator)-get_hand_links(manipulator)

def get_hand_links(manipulator):
  return set(manipulator.GetChildLinks())

def set_visible(link):
  link.SetVisible(True)

def set_invisible(link):
  link.SetVisible(False)

#################################################################

def forward_manip_direction(robot, manip_trans):
  #return rot_transform_point(rot_from_trans(get_manip_trans(oracle)), np.array((0, 0, 1)))
  return rot_transform_point(rot_from_trans(manip_trans), robot.GetActiveManipulator().GetDirection()) # = GetLocalToolDirection = GetPalmDirection

def vertical_manip_direction(robot, manip_trans):
  return rot_transform_point(rot_from_trans(manip_trans), np.array((1, 0, 0)))

def lateral_manip_direction(robot, manip_trans=None):
  return rot_transform_point(rot_from_trans(manip_trans), np.array((0, 1, 0)))

def approach_vector_from_object_trans(object_trans, approach_direction):
  return rot_transform_point(rot_from_trans(object_trans), APPROACH_DISTANCE*normalize(np.array(approach_direction)))

def manip_from_pose_grasp(pose, grasp):
  object_trans = trans_from_pose(pose.value)
  manip_trans = manip_trans_from_object_trans(object_trans, grasp.grasp_trans)
  return manip_trans, approach_vector_from_object_trans(object_trans, grasp.approach_vector)

#################################################################

# Post-multiplication of matrices Manip*Tool*Grasp = Pose
# get_trans(manipulator) produces the gripper tform (Grip)
# +z is pointing, +x is perpendicular to face
# EndEff*Tool = Grip
# Can fix the orientation by applying just the rotation

# Grip * Tool = Manip
def get_tool_trans(oracle): # NOTE - really the gripper joint because T[0,3] = 0.18
  return oracle.robot.GetActiveManipulator().GetLocalToolTransform()
  # GetLocalToolTransform() = GetGraspTransform()

# Tool.T * Grip.T = Manip.T
def gripper_trans_from_manip_trans(oracle, manip_trans):
  return np.linalg.solve(get_tool_trans(oracle).T, manip_trans.T).T

# Grip * Tool = Manip
def manip_trans_from_gripper_trans(oracle, gripper):
  return np.dot(gripper, get_tool_trans(oracle))

#################################################################

def get_pr2_tool_trans(oracle):
  return np.array([[ 0.,    0.,    1.,    0   ], # NOTE - removed T[0,3] = 0.18
                   [ 0.,    1.,    0.,    0.  ],
                   [-1.,    0.,    0.,    0.  ],
                   [ 0.,    0.,    0.,    1.  ]])

def get_pr2_gripper_trans(oracle):
  return np.dot(get_manip_trans(oracle), np.linalg.inv(get_tool_trans(oracle)))
