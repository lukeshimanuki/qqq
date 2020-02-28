from manipulation.bodies.bodies import *

def body_saver(body):
  return body.CreateKinBodyStateSaver()

def robot_saver(robot):
  return robot.CreateRobotStateSaver()

def collision_saver(env, options):
  return CollisionOptionsStateSaver(env.GetCollisionChecker(), options=options)

class ManipulationOracleStateSaver(object): # NOTE - Allows nesting by creating a new object
  def __init__(self, oracle):
    self.oracle = oracle
    #self.oracle.env.Lock()
    self.robot_saver = robot_saver(self.oracle.robot)
    self.body_savers = [] # TODO - only save objects in some instances
    self.active = set()
    for name in self.oracle.get_objects() + self.oracle.obstacles:
      self.body_savers.append(body_saver(self.oracle.bodies[name])) # NOTE - can create even if not added
      if self.oracle.is_active(name):
        self.active.add(name)
  #def Release(self):
  #  self.oracle.env.Unlock()
  def Restore(self):
    for body_saver in self.body_savers:
      name = get_name(body_saver.GetBody())
      self.oracle.set_active(name, True) # NOTE - can only restore if added
      body_saver.Restore()
      self.oracle.set_active(name, name in self.active)
    #self.oracle.set_active(get_name(self.robot_saver.GetBody())) # TODO - deactivate if initially inactive?
    self.robot_saver.Restore()
    #self.oracle.env.Unlock()
  def __enter__(self):
    pass
  def __exit__(self, type, value, traceback):
    self.Restore()

class EnvironmentStateSaver(object): # TODO - will fail if any objects have been deleted
  def __init__(self, env):
    self.env = env
    self.robot_savers = [robot_saver(robot) for robot in env.GetRobots()]
    self.body_savers = [body_saver(body) for body in env.GetBodies()]
  def Restore(self):
    for bs in self.body_savers:
      bs.Restore()
    for rs in self.robot_savers:
      rs.Restore()
  def __enter__(self):
    pass
  def __exit__(self, type, value, traceback):
    self.Restore()

class DynamicEnvironmentStateSaver(object): # TODO - will fail if any objects have been deleted
  def __init__(self, env):
    self.env = env
    self.robot_savers = {robot: robot_saver(robot) for robot in env.GetRobots()}
    self.body_savers = {body: body_saver(body) for body in env.GetBodies()}
  def Restore(self):
    for body in (self.env.GetRobots() + self.env.GetBodies()):
      if body not in self.robot_savers or body not in self.body_savers:
        self.env.Remove(body)
    for b, bs in self.body_savers.iteritems():
      if self.env.GetKinBody(b.GetName()) is None:
        self.env.AddKinBody(b)
      bs.Restore()
    for r, rs in self.robot_savers.iteritems():
      if self.env.GetRobot(r.GetName()) is None:
        self.env.AddRobot(r)
      rs.Restore()
  def __enter__(self):
    pass
  def __exit__(self, type, value, traceback):
    self.Restore()

class GrabSaver(object):
  def __init__(self, robot):
    self.robot = robot
    self.grabbed = robot.GetGrabbed()
  def Restore(self):
    for body in self.robot.GetGrabbed():
      if body not in self.grabbed:
        self.robot.Release(body)
    for body in self.grabbed:
      if not self.robot.IsGrabbing(body):
        self.robot.Grab(body)
  def __enter__(self):
    pass
  def __exit__(self, type, value, traceback):
    self.Restore()

class HoldingRobotSaver(object):
  def __init__(self, robot):
    self.robot_saver = robot_saver(robot)
    self.grab_saver = GrabSaver(robot)
    self.body_savers = [body_saver(body) for body in robot.GetGrabbed()]
  def Restore(self):
    self.robot_saver.Restore() # NOTE - the order is important
    for saver in self.body_savers:
      saver.Restore()
    self.grab_saver.Restore()
  def __enter__(self):
    pass
  def __exit__(self, type, value, traceback):
    self.Restore()
