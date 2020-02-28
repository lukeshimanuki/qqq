from manipulation.bodies.bodies import *
from manipulation.constants import GRASP_APPROACHES, GRASP_TYPES
from manipulation.bodies.robot import get_tool_trans
from openravepy import databases

TOUCH_DISTANCE = 0.02
FINGER_DISTANCE = 0.03
RADIUS_DISTANCE = 0.01

# TODO - encode approach vector into grasp (to handle side versus top)
def get_grasp_options(body, grasp_approach): # TODO - top and/or side grasps
  if grasp_approach == GRASP_APPROACHES.SIDE:
    if is_box(body):
      return BoxSideGraspOptions
    if is_cylinder(body):
      return CylinderSideGraspOptions
    if is_sphere(body):
      return SphereSideGraspOptions
    return OpenRaveSideGraspOptions
  if grasp_approach == GRASP_APPROACHES.TOP:
    if is_box(body):
      return BoxTopGraspOptions
    if is_cylinder(body):
      return CylinderTopGraspOptions
    if is_sphere(body):
      pass
    pass
  raise RuntimeError()

class GraspOptions(object):
  def __init__(self, geom_hash, collisions): #, manip_hash): # GetKinematicsStructureHash()
    self.geom_hash = geom_hash
    self.collisions = collisions
  def options(self):
    return {name: value for name, value in self.__dict__.items() if name != '__hash'}
  @staticmethod #@abstractmethod
  def default(grasp_type, body):
    raise NotImplementedError()
  def __eq__(self, other):
    return type(self) == type(other) and hash(self) == hash(other) and self.options() == other.options()
  def __ne__(self, other):
    return not self == other
  def __hash__(self):
    if '__hash' not in self.__dict__: self.__dict__['__hash'] = hash(frozenset(self.options().items()))
    return self.__dict__['__hash']
  def __str__(self):
    return self.__class__.__name__ + str_object(self.options())

class SideGraspOptions(GraspOptions):
  approach_vector = (1, 0, 1) # NOTE - with respect to the object at the origin (angle = 0)

class TopGraspOptions(GraspOptions):
  approach_vector = (0, 0, 1)

#################################################################

class BoxSideGraspOptions(SideGraspOptions):
  def __init__(self, geom_hash, collisions, xy_distance, z_distance):
    super(self.__class__, self).__init__(geom_hash, collisions)
    self.xy_distance = xy_distance
    self.z_distance = z_distance
  @staticmethod
  def default(grasp_type, body):
    assert is_box(body)
    if grasp_type == GRASP_TYPES.GRASP:
      return BoxSideGraspOptions(geometry_hash(body), True, FINGER_DISTANCE, 0.01)
    if grasp_type == GRASP_TYPES.TOUCH:
      return BoxSideGraspOptions(geometry_hash(body), False, -TOUCH_DISTANCE, 0.01)
    if grasp_type == GRASP_TYPES.COLLIDE:
      return BoxSideGraspOptions(geometry_hash(body), False, FINGER_DISTANCE, 0.01)
    raise RuntimeError()
  def grasp_trans(self, oracle): # TODO - top / bottom and sample different heights
    # NOTE - in the grasp transform (!= gripper transform)
    #   +z is the outwards pointing direction of the gripper
    #   +x is perpendicular to the gripper's plane
    #   Max width to grasp is about 0.07

    #NOTE: this assumes that object is at the origin
    aabb = oracle.get_aabb(oracle.get_body_name(self.geom_hash)) # body.GetLinks()[0].GetGeometries()[0].GetBoxExtents()
    radii = np.tile(np.array([aabb.extents()[i]-self.xy_distance for i in range(2)]), 2)
    z = aabb.extents()[2]-self.z_distance
    thetas = np.linspace(0, 2*PI, 4, endpoint=False)

    print 'Side grasp generation'
    manip_point = aabb.pos() + np.array([0, 0, aabb.extents()[2] - self.z_distance])
    tool_quat = quat_from_trans(get_tool_trans(oracle))
    for rotation in np.linspace(0, 2*PI, 4, endpoint=False): # I think the rotation is the rotation of the manipulator
      #approach_vector = quat_transform_point((quat_from_z_rot(theta)), self.approach_vector)
      manip_quat = quat_dot(quat_look_at(-unit_z()), quat_from_angle_vector(rotation, unit_x()), tool_quat) # Grip * Tool = Manip
      """
      print '==============='
      print oracle.robot.GetActiveManipulator().GetLocalToolTransform()
      print tool_quat
      print rotation
      print aabb
      print self.approach_vector
      print manip_quat
      print manip_point
      print '==============='
      """
      yield compute_grasp(trans_from_quat_point(manip_quat, manip_point), unit_trans()), self.approach_vector

    """
    import pdb;pdb.set_trace()
    for radius, theta in zip(radii, thetas):
    radius = 
      direction = np.array([cos(theta), sin(theta), 0])
      manip_point = radius*direction + np.array([0, 0, z]) + aabb.pos()
      #for rotation in [0, PI]:
      for rotation in np.linspace(0, 2*PI, 4, endpoint=False): # I think the rotation is the rotation of the manipulator
        #manip_quat = quat_dot(quat_look_at(-direction), quat_from_angle_vector(rotation, unit_x()), tool_quat) # Grip * Tool = Manip
        manip_quat = quat_dot(quat_look_at(-unit_z()), quat_from_angle_vector(rotation, unit_x()), tool_quat) # Grip * Tool = Manip
        approach_vector = quat_transform_point((quat_from_z_rot(theta)), self.approach_vector)
        yield compute_grasp(trans_from_quat_point(manip_quat, manip_point), unit_trans()), approach_vector
    """

class BoxTopGraspOptions(TopGraspOptions):
  def __init__(self, geom_hash, collisions, z_distance):
    super(self.__class__, self).__init__(geom_hash, collisions)
    self.z_distance = z_distance
  @staticmethod
  def default(grasp_type, body):
    assert is_box(body)
    if grasp_type == GRASP_TYPES.GRASP:
      return BoxTopGraspOptions(geometry_hash(body), True, FINGER_DISTANCE)
    if grasp_type == GRASP_TYPES.TOUCH:
      return BoxTopGraspOptions(geometry_hash(body), False, -TOUCH_DISTANCE)
    if grasp_type == GRASP_TYPES.COLLIDE:
      return BoxTopGraspOptions(geometry_hash(body), False, FINGER_DISTANCE)
    raise RuntimeError()
  def grasp_trans(self, oracle):
    aabb = oracle.get_aabb(oracle.get_body_name(self.geom_hash))
    manip_point = aabb.pos() + np.array([0, 0, aabb.extents()[2] - self.z_distance])

    tool_quat = quat_from_trans(get_tool_trans(oracle))
    #print np.linspace(0, 2*PI, 4, endpoint=False)
    for rotation in np.linspace(0, 2*PI, 4, endpoint=False): # I think the rotation is the rotation of the manipulator
      #manip_quat = quat_dot(quat_look_at(-unit_z()), quat_from_angle_vector(rotation, unit_z()), tool_quat) # NOTE - previously was a bug
      manip_quat = quat_dot(quat_look_at(-unit_z()), quat_from_angle_vector(rotation, unit_x()), tool_quat) # Grip * Tool = Manip
      yield compute_grasp(trans_from_quat_point(manip_quat, manip_point), unit_trans()), self.approach_vector

#################################################################

class CylinderSideGraspOptions(SideGraspOptions):
  def __init__(self, geom_hash, collisions, r_distance, z_distance, number):
    super(self.__class__, self).__init__(geom_hash, collisions)
    self.r_distance = r_distance
    self.z_distance = z_distance
    self.number = number
  @staticmethod
  def default(grasp_type, body):
    assert is_cylinder(body)
    radius = geometries(body)[0].GetCylinderRadius()
    return CylinderSideGraspOptions(geometry_hash(body), False, radius+TOUCH_DISTANCE, 0.01, 4) # TODO - UNDO THIS!!!
    #if grasp_type == GRASP_TYPES.GRASP:
    #  return CylinderSideGraspOptions(geometry_hash(body), True, RADIUS_DISTANCE, 0.01, 4)
    #if grasp_type == GRASP_TYPES.TOUCH:
    #  return CylinderSideGraspOptions(geometry_hash(body), False, radius+TOUCH_DISTANCE, 0.01, 4)
    #if grasp_type == GRASP_TYPES.COLLIDE:
    #  return CylinderSideGraspOptions(geometry_hash(body), False, RADIUS_DISTANCE, 0.01, 4)
    #raise RuntimeError()
  def grasp_trans(self, oracle): # TODO - top / bottom, sample different heights, and infinite generator
    # NOTE - grasp max radius is about .035
    geometry = geometries(oracle.get_body(oracle.get_body_name(self.geom_hash)))[0]
    pos = point_from_trans(geometry.GetTransform())
    z = .5*geometry.GetCylinderHeight()-self.z_distance

    tool_quat = quat_from_trans(get_tool_trans(oracle))
    for theta in np.linspace(0, 2*PI, self.number, endpoint=False):
      direction = np.array([cos(theta), sin(theta), 0])
      manip_point = self.r_distance*direction + np.array([0, 0, z]) + pos
      for rotation in [0, PI]:
        manip_quat = quat_dot(quat_look_at(-direction), quat_from_angle_vector(rotation, unit_x()), tool_quat) # Grip * Tool = Manip
        approach_vector = quat_transform_point((quat_from_z_rot(theta)), self.approach_vector)
        yield compute_grasp(trans_from_quat_point(manip_quat, manip_point), unit_trans()), approach_vector

class CylinderTopGraspOptions(TopGraspOptions):
  def __init__(self, geom_hash, collisions, z_distance, number):
    super(self.__class__, self).__init__(geom_hash, collisions)
    self.z_distance = z_distance
    self.number = number
  @staticmethod
  def default(grasp_type, body):
    assert is_box(body)
    if grasp_type == GRASP_TYPES.GRASP:
      return CylinderTopGraspOptions(geometry_hash(body), True, FINGER_DISTANCE, 4)
    if grasp_type == GRASP_TYPES.TOUCH:
      return CylinderTopGraspOptions(geometry_hash(body), False, -TOUCH_DISTANCE, 4)
    if grasp_type == GRASP_TYPES.COLLIDE:
      raise RuntimeError()
    raise RuntimeError()
  def grasp_trans(self, oracle):
    geometry = geometries(oracle.get_body(oracle.get_body_name(self.geom_hash)))[0]
    manip_point = point_from_trans(geometry.GetTransform()) + \
                  np.array([0, 0, .5*geometry.GetCylinderHeight()[2] + self.z_distance])

    tool_quat = quat_from_trans(get_tool_trans(oracle))
    for rotation in np.linspace(0, 2*PI, self.number, endpoint=False):
      manip_quat = quat_dot(quat_look_at(-unit_z()), quat_from_angle_vector(rotation, unit_z()), tool_quat) # Grip * Tool = Manip
      yield compute_grasp(trans_from_quat_point(manip_quat, manip_point), unit_trans()), self.approach_vector

#################################################################

class SphereSideGraspOptions(SideGraspOptions):
  def __init__(self, geom_hash, collisions, r_distance, number):
    super(self.__class__, self).__init__(geom_hash, collisions)
    self.r_distance = r_distance
    self.number = number
  @staticmethod
  def default(grasp_type, body):
    assert is_sphere(body)
    radius = geometries(body)[0].GetSphereRadius()
    if grasp_type == GRASP_TYPES.GRASP:
      return SphereSideGraspOptions(geometry_hash(body), True, RADIUS_DISTANCE, 4)
    if grasp_type == GRASP_TYPES.TOUCH:
      return SphereSideGraspOptions(geometry_hash(body), False, radius+TOUCH_DISTANCE, 4)
    if grasp_type == GRASP_TYPES.COLLIDE:
      return SphereSideGraspOptions(geometry_hash(body), False, RADIUS_DISTANCE, 4)
    raise RuntimeError()
  def grasp_trans(self, oracle): # TODO - top / bottom, sample different heights, and infinite generator
    # NOTE - grasp max radius is about .035
    geometry = geometries(oracle.get_body(oracle.get_body_name(self.geom_hash)))[0]
    pos = point_from_trans(geometry.GetTransform())

    tool_quat = quat_from_trans(get_tool_trans(oracle))
    for theta in np.linspace(0, 2*PI, self.number, endpoint=False):
      direction = np.array([cos(theta), sin(theta), 0])
      manip_point = self.r_distance*direction + pos
      for rotation in [0, PI]:
        manip_quat = quat_dot(quat_look_at(-direction), quat_from_angle_vector(rotation, unit_x()), tool_quat) # Grip * Tool = Manip
        approach_vector = quat_transform_point((quat_from_z_rot(theta)), self.approach_vector)
        yield compute_grasp(trans_from_quat_point(manip_quat, manip_point), unit_trans()), approach_vector

#################################################################

class OpenRaveSideGraspOptions(SideGraspOptions):
  def __init__(self, geom_hash, collisions):
    super(self.__class__, self).__init__(geom_hash, collisions)
  @staticmethod
  def default(grasp_type, body):
    if grasp_type == GRASP_TYPES.GRASP:
      return OpenRaveSideGraspOptions(geometry_hash(body))
    if grasp_type in [GRASP_TYPES.TOUCH, GRASP_TYPES.COLLIDE]:
      return OpenRaveSideGraspOptions(geometry_hash(body))
    raise RuntimeError()
  def grasp_trans(self, oracle):
    body = oracle.bodies[oracle.get_body_name(self.geom_hash)]
    # class _GraspOptions(object):
    #   def __init__(oracle):
    #     oracle.normalanglerange, oracle.standoffs, oracle.rolls, oracle.boxdelta, oracle.directiondelta = 0.0, [0], np.arange(0.5*np.pi, 2*np.pi, np.pi), 0.01, approach_dist
    g_model = databases.grasping.GraspingModel(oracle.robot, body)
    if not g_model.load():
      g_model.autogenerate() #g_model.autogenerate(_GraspOptions())
    horizontal_grasps = filter(lambda g: abs(g_model.getGlobalApproachDir(g)[2]) < 0.01, g_model.grasps)
    # upper_grasps = filter(lambda g: g_model.GetLocalGraspTransform(g, collisionfree=True)[1][3] > 0, g_model.grasps) # upper half (+y is up)
    g_model.grasps = horizontal_grasps
    return map(lambda grasp: g_model.GetLocalGraspTransform(grasp, collisionfree=False), g_model.grasps)  # compute_grasp(g_model.getGlobalGraspTransform(grasp, collisionfree=False), get_trans(body))
