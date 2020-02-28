from math import sqrt

import numpy as np

from transforms import quat_dot, normalize, unit_x, quat_look_at, compute_grasp, trans_from_quat_point, \
  quat_from_angle_vector, unit_trans, quat_from_trans, unit_z, manip_trans_from_object_trans, trans_from_pose, \
  quat_transform_point
from manipulation.bodies.robot import get_tool_trans
from misc.numerical import PI
from misc.objects import str_object
from manipulation.constants import APPROACH_DISTANCE
import operator

APPROACH_VECTOR = APPROACH_DISTANCE*normalize(np.array([1, 0, -1])) # TODO - move this elsewhere

class Contact(object):
  def __init__(self, contact_trans, direction, gripper_config=None, gripper_traj=None): # TODO - fill in
    self.direction = direction
    self.grasp_trans = contact_trans # TODO - rename self.contact_trans
    self.gripper_config = gripper_config
    self.gripper_traj = gripper_traj
  def __repr__(self):
    return self.__class__.__name__ + str_object(self.grasp_trans[:3, 3])

def manip_trans_from_pose_contact(pose, contact):
  return manip_trans_from_object_trans(trans_from_pose(pose.value), contact.grasp_trans)

# NOTE - cannot use center of object to infer approach vector because gripper might not be tangent
def approach_vector_from_pose_contact(pose, contact): # TODO - universal way of inferring approach_vector from manip_trans (probably not possible)
  approach_vector = quat_transform_point(quat_from_trans(manip_trans_from_pose_contact(pose, contact)), APPROACH_VECTOR)
  if contact.grasp_trans[0, 3] > 0: approach_vector[:2] *= -1
  return approach_vector

# TODO - automatically compute these from the gripper geometry
PUSH_HEIGHT = .02
PUSH_SEPERATION = -.02 # NOTE - depends if large enough to gripper or just fingers

# Assuems the body is pushable by its reference cylinder
def get_contacts(oracle, body_name, direction): # NOTE - Other objects will only have a fixed set of contacts and directions
  #print 'direction', direction[2]
  assert direction[2] == 0
  contacts = []
  direction = normalize(direction)
  aabb = oracle.get_aabb(body_name)
  radius = sqrt(oracle.get_radius2D2(body_name))
  #radius = body.GetLinks()[0].GetGeometries()[0].GetCylinderRadius()
  height = 2*aabb.extents()[2]
  #height = body.GetLinks()[0].GetGeometries()[0].GetCylinderHeight()

  distance = radius + PUSH_SEPERATION
  z = -height/2 + PUSH_HEIGHT
  tool_quat = quat_from_trans(get_tool_trans(oracle))
  manip_point = -distance*direction + np.array([0, 0, z]) + aabb.pos()
  for rotation in [0, PI]: # NOTE - 2 hand trans can push in a given direction
    manip_quat = quat_dot(quat_look_at(-direction), quat_look_at(-unit_z()), quat_from_angle_vector(rotation, unit_x()), tool_quat) # Grip * Tool = Manip
    contacts.append(Contact(compute_grasp(trans_from_quat_point(manip_quat, manip_point), unit_trans()), direction))
  return contacts

# Assumes the body is pushable by its reference AABB
def box_contacts(oracle, body_name): # TODO - more generically push boxes along their faces
  directions = [np.array([d, 0, 0]) for d in (-1, 1)] + [np.array([0, d, 0]) for d in (-1, 1)]
  return reduce(operator.add, [get_contacts(oracle, body_name, d) for d in directions])
  # TODO - validate that this works
