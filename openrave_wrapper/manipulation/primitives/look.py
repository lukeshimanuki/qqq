from manipulation.primitives.transforms import get_trans, get_point
from math import atan2, sqrt
import numpy as np
from openravepy import IkParameterization, IkFilterOptions

# TODO - I might need to enable IK to use multiple IK - ikmodel2.robot.Enable(ikmodel==ikmodel2)
# NOTE - no this just specifies if something should be in collision checks or physics simulations (An enabled link takes part in collision detection and physics simulations)

#target=ikmodel.manip.GetTransform()[0:3,3]+(random.rand(3)-0.5)
#solutions = ikmodel.manip.FindIKSolutions(IkParameterization(target,IkParameterization.Type.Lookat3D),IkFilterOptions.CheckEnvCollisions)
# KinBody.Link.GetGlobalCOM()

# print [manip.GetName() for manip in robot.GetManipulators()]
# [u'leftarm_camera', u'rightarm_camera', u'head_torso', u'head', u'rightarm_torso', u'rightarm', u'leftarm_torso', u'leftarm']

# openrave.py --example tutorial_iklookat_multiple

def check_ray_collisions():
  raise NotImplementedError() # CheckCollisionRays

HEAD_NAME = 'head' # head | head_torso

def look_at(robot, body):
  manipulator = robot.GetManipulator(HEAD_NAME)
  head_point = get_point(manipulator)
  body_point = get_point(body) # Use center of mass, bbox center, etc...

  dx, dy, dz = body_point - head_point
  theta = atan2(dy, dx)
  phi = -atan2(dz, sqrt(dx**2 + dy**2))
  solution = np.array([theta, phi]) # TODO - check if within limits
  robot.SetDOFValues(solution, manipulator.GetArmIndices())
  return solution

def look_at_ik(oracle, body_name):
  robot = oracle.robot
  manipulator = robot.GetManipulator(HEAD_NAME)
  body_point = get_point(oracle.bodies[body_name]) # Use center of mass, bbox center, etc...
  ik_param = IkParameterization(body_point,IkParameterization.Type.Lookat3D)
  solutions = oracle.look_model.manip.FindIKSolutions(ik_param, IkFilterOptions.CheckEnvCollisions)
  if len(solutions) == 0:
    return None
  assert len(solutions) == 1
  solution = solutions[0]
  robot.SetDOFValues(solution, manipulator.GetArmIndices())
  return solution
  #print manipulator.GetFreeParameters()
