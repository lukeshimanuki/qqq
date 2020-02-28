from manipulation.collision.collision_primitives import robot_collision
from manipulation.primitives.savers import *
from manipulation.bodies.robot import get_arm_indices
from openravepy import openravepy_int

from manipulation.bodies.robot import get_active_arm_indices, get_manipulator

def solve_all_inverse_kinematics(env, robot, manip_trans):
  with robot:
    robot.SetActiveDOFs(get_active_arm_indices(robot))
    with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
      config = get_manipulator(robot).FindIKSolution(manip_trans, 0)
      #ik_param = IkParameterization(manip_trans[0:3,3],ikmodel.iktype) # build up the translation3d ik query
      #ik_param = GetIkParameterization(get_manipulator(robot), ...)
      configs = get_manipulator(robot).FindIKSolutions(ik_param, openravepy_int.IkFilterOptions.CheckEnvCollisions) # get all solutions
      if config is None: return None
      set_config(robot, config, get_active_arm_indices(robot))
      if env.CheckCollision(robot) or robot.CheckSelfCollision(): return None
      return config

def solve_inverse_kinematics_exhaustive(env, robot, manip_trans):
  with robot:
    #robot.SetActiveDOFs(get_active_arm_indices(robot))
    #with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs): # TODO - do I want this?
    with collision_saver(env, 0):
      config = get_manipulator(robot).FindIKSolution(manip_trans, openravepy_int.IkFilterOptions.CheckEnvCollisions) # NOTE - only handles manipulator collisions
      set_config(robot, config, get_active_arm_indices(robot))
      if env.CheckCollision(robot) or robot.CheckSelfCollision(): return None # NOTE - not reliably avoiding collisions
      return config

# GetIkSolver, SetIKSolver
# IKSolver.solve()
def inverse_kinematics_helper(env, robot, manip_trans):
  with robot:
    robot.SetActiveDOFs(get_active_arm_indices(robot))
    with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
      #print get_manipulator(robot).GetIkSolver()
      config = get_manipulator(robot).FindIKSolution(manip_trans, 0) # NOTE - Finds a close solution to the current robot's joint values
      if config is None: return None
      set_config(robot, config, get_active_arm_indices(robot))
      if env.CheckCollision(robot) or robot.CheckSelfCollision(): return None
      return config

def inverse_kinematics(oracle, manip_trans): # 0.005 sec
  return inverse_kinematics_helper(oracle.env, oracle.robot, manip_trans)
  # with oracle.robot:
  #   oracle.robot.SetActiveDOFs(get_arm_indices(oracle)) # TODO - is this slow and/or necessary?
  #   with collision_saver(oracle.env, openravepy_int.CollisionOptions.ActiveDOFs):
  #     config = oracle.robot.GetActiveManipulator().FindIKSolution(manip_trans, 0)
  #     if config is None: return None
  #     set_config(oracle.robot, config, oracle.robot.GetActiveManipulator().GetArmIndices())
  #     if robot_collision(oracle): return None
  #     return config

# def inverse_kinematics(oracle, manip_trans): # 0.03 sec
#   with oracle.robot:
#     oracle.robot.SetActiveDOFs(oracle.robot.GetActiveManipulator().GetArmIndices())
#     with collision_saver(oracle.env, openravepy_int.CollisionOptions.ActiveDOFs):
#       t1 = time()
#       options = openravepy.IkFilterOptions.IgnoreEndEffectorCollisions | \
#         openravepy.IkFilterOptions.IgnoreEndEffectorEnvCollisions | \
#         openravepy.IkFilterOptions.IgnoreEndEffectororacleCollisions | \
#         openravepy.IkFilterOptions.IgnoreoracleCollisions | \
#         openravepy.IkFilterOptions.IgnoreJointLimits | \
#         openravepy.IkFilterOptions.IgnoreCustomFilters
#
#       configs = oracle.robot.GetActiveManipulator().FindIKSolutions(manip_trans, options)
#       t1 = time()
#       config = oracle.robot.GetActiveManipulator().FindIKSolution(manip_trans, options)
#
#       print len(configs), time() - t1
#       if len(configs) == 0:
#         return None
#       config = configs[0]
#       return config

# def inverse_kinematics(oracle, manip_trans, check_collisions=True): #FindIKSolutions
#   with oracle.robot:
#     oracle.robot.SetActiveDOFs(oracle.robot.GetActiveManipulator().GetArmIndices())
#     with collision_saver(oracle.env, openravepy_int.CollisionOptions.ActiveDOFs):
#       if check_collisions:
#         options = openravepy.openravepy_int.IkFilterOptions.CheckEnvCollisions
#       else:
#         options = openravepy.IkFilterOptions.IgnoreEndEffectorCollisions | \
#           openravepy.IkFilterOptions.IgnoreEndEffectorEnvCollisions | \
#           openravepy.IkFilterOptions.IgnoreEndEffectororacleCollisions | \
#           openravepy.IkFilterOptions.IgnoreoracleCollisions
#       return oracle.robot.GetActiveManipulator().FindIKSolution(manip_trans, options) #RegisterCustomFilter

# # Set joint limits also on the IK (from test_robot.py)
# def customfilter(solution, manip, ikparam):
#   out = manip.GetIkSolver().SendCommand('GetRobotLinkStateRepeatCount')
#   if out=='1':
#       numrepeats[0] += 1
#   out = manip.GetIkSolver().SendCommand('GetSolutionIndices')
#   for index in out.split()[1:]:
#       indices.append(int(index))
#   return IkReturnAction.Success
#
# handle = ikmodel.manip.GetIkSolver().RegisterCustomFilter(0,customfilter)
# sols = ikmodel.manip.FindIKSolutions(ikparam,IkFilterOptions.CheckEnvCollisions)
