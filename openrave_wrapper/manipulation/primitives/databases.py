from openravepy import *
from openravepy.misc import InitOpenRAVELogging
from time import time, sleep
from datetime import timedelta

ENVIRONMENT = 'environments/single_table.env.xml' 
ROBOT = 'pr2'
MANIPULATORS = ['leftarm'] #['leftarm', 'leftarm_torso', 'rightarm', 'rightarm_torso']
OBJECTS = ['chair']

try:
  #InitOpenRAVELogging()
  #RaveSetDebugLevel(DebugLevel.Verbose) # Debug, Info, Error, Fatal, VerifyPlans, Warn, Verbose
  t0 = time()

  def time_passed():
    return str(timedelta(seconds=int(time()-t0)))

  env = Environment()
  env.Load(ENVIRONMENT)
  print '\n------------------------------------------------------------------------------\n'
 
  robot = env.GetRobot(ROBOT)
  cd_model = databases.convexdecomposition.ConvexDecompositionModel(robot)
  if not cd_model.load():
    print time_passed(), 'Generating', ROBOT,'convex decomposition model'
    cd_model.autogenerate()
  print time_passed(), 'Loaded', ROBOT, 'convex decomposition model\n'

  for manipulator in MANIPULATORS:
    robot.SetActiveManipulator(manipulator)

    """
    ik_model = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
    if not ik_model.load():
      print time_passed(), 'Generating', ROBOT, manipulator, 'inverse kinematics model'
      ik_model.autogenerate()
    print time_passed(), 'Loaded', ROBOT, manipulator, 'inverse kinematics model\n'

    kr_model = databases.kinematicreachability.ReachabilityModel(robot)
    if not kr_model.load():
      print time_passed(), 'Generating', ROBOT, manipulator, 'kinematic reachability model'
      kr_model.autogenerate()
    print time_passed(), 'Loaded', ROBOT, manipulator, 'kinematic reachability model\n'

    ir_model = databases.inversereachability.InverseReachabilityModel(robot)
    if not ir_model.load():
      print time_passed(), 'Generating', ROBOT, manipulator, 'inverse reachability model'
      ir_model.autogenerate()
    print time_passed(), 'Loaded', ROBOT, manipulator, 'inverse reachability model\n'
    """

    for obj in OBJECTS:
      #env.Add(env.ReadKinBodyXMLFile(obj))
      body = env.GetKinBody(obj)
      g_model = databases.grasping.GraspingModel(robot, body)
      if not g_model.load():
        print time_passed(), 'Generating', obj, 'grasp model\n'
        g_model.autogenerate()
        print time_passed(), 'Loaded', obj, 'grasp model\n'
finally:
  RaveDestroy()
