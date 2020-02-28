import numpy as np

from manipulation.bodies.bodies import get_name
from manipulation.primitives.transforms import length, is_upright, get_trans, get_point
from openravepy import GeometryType

def cylinder_collision(oracle, body_name1, body_name2):
  # TODO
  # - ODE cylinder-cylinder collision doesn't seem to work reliably. But PQP does work. Set the PQP just for cylinders
  # - Incorporate cylinder1.GetTransform() for local point
  # - Just use the aabb
  # - Only considers case where cylinders are resting on their base
  body1, body2 = oracle.get_body(body_name1), oracle.get_body(body_name2)
  cylinder1, cylinder2 = oracle.get_geometries(body_name1)[0], oracle.get_geometries(body_name2)[0]
  if is_upright(get_trans(body1)) and is_upright(get_trans(body2)):
    point1, point2 = get_point(body1), get_point(body2)
    return abs(point1[2] - point2[2]) < (cylinder1.GetCylinderHeight() + cylinder2.GetCylinderHeight())/2. and \
      length(point1[:2] - point2[:2]) < cylinder1.GetCylinderRadius() + cylinder2.GetCylinderRadius()
  return oracle.env.CheckCollision(body1, body2)

# TODO - perturb all poses in case identical objects are placed in the same location
def collision(oracle, body_name1, body_name2=None): # NOTE - doesn't consider self collisions
  if body_name2 is None:
    return oracle.is_active(body_name1) and oracle.env.CheckCollision(oracle.bodies[body_name1])
  if oracle.get_geometry_types(body_name1) == [GeometryType.Cylinder] and oracle.get_geometry_types(body_name2) == [GeometryType.Cylinder]:
    return cylinder_collision(oracle, body_name1, body_name2)
  return oracle.is_active(body_name1) and oracle.is_active(body_name2) and \
         oracle.env.CheckCollision(oracle.get_body(body_name1), oracle.get_body(body_name2))
        #oracle.necessary_collision_current(body_name1, body_name2) and \
        #(np.allclose(get_point(body1), get_point(body2)) or \ # TODO - catches same pose bug

def self_collision(oracle, body_name):
  return oracle.is_active(body_name) and oracle.bodies[body_name].CheckSelfCollision() # NOTE - 0.003 sec/call

def env_collision(oracle):
  bodies = oracle.bodies.keys()
  for i in range(len(bodies)):
    for j in range(i+1, len(bodies)):
      if collision(oracle, bodies[i], bodies[j]):
        return True
  return False

def obstacle_collision(oracle, body_name):
  return any(collision(oracle, body_name, obst_name) for obst_name in oracle.obstacles)

def object_collision(oracle, body_name, excluded=set()):
  return any(body_name != obj_name and obj_name not in excluded and collision(oracle, body_name, obj_name) for obj_name in oracle.objects)

def robot_collision(oracle, body_name=None, check_self=True):
  if body_name is None:
    return collision(oracle, get_name(oracle.robot)) or (not check_self and self_collision(oracle, get_name(oracle.robot)))
  return collision(oracle, get_name(oracle.robot), body_name)

def base_collision(oracle, body_name=None): # TODO - why does this take 0.001 seconds per call (more than standard collision)?
  if body_name is None:
    return oracle.env.CheckCollision(oracle.base)
  return oracle.is_active(body_name) and oracle.env.CheckCollision(oracle.base, oracle.bodies[body_name])

def body_collision(oracle, body_name=None):
  if body_name is None:
    return base_collision(oracle) or oracle.env.CheckCollision(oracle.torso)
  return oracle.is_active(body_name) and base_collision(oracle, body_name=body_name) and oracle.env.CheckCollision(oracle.torso, oracle.bodies[body_name])

def gripper_collision(oracle, gripper, body_name=None):
  if body_name is None:
    return oracle.env.CheckCollision(gripper)
  return oracle.is_active(body_name) and oracle.env.CheckCollision(gripper, oracle.bodies[body_name])
  #oracle.robot.GetActiveManipulator().CheckEndEffectorCollision(grasp_trans) # NOTE - this takes 0.001 seconds per call (more than standard collision)?

#def arm_collision(oracle):
#  return oracle.robot.GetActiveManipulator().CheckIndependentCollision() # NOTE - Actuallly does all but manipulator...

#################################################################

def is_valid_point(robot, point):
  lower, upper = robot.GetAffineTranslationLimits()
  return np.all(lower <= point) and np.all(point <= upper)

#def is_valid_base(oracle, base_trans):
#  base_values = base_values_from_trans(base_trans)
#  return np.all((oracle.env_min <= base_values)[:2]) and np.all((base_values <= oracle.env_max)[:2])

# GetDOFLimits, GetActiveDOFLimits, GetAffineRotation3DLimits, GetAffineRotationQuatLimits
