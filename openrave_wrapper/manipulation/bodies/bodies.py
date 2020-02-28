from manipulation.constants import BODY_PLACEMENT_Z_OFFSET
from manipulation.regions import AARegion
from manipulation.primitives.utils import get_env
from manipulation.bodies.bounding_volumes import aabb_from_body, aabb_overlap
from manipulation.primitives.transforms import *
from openravepy import KinBody, RaveCreateKinBody, GeometryType

import copy

def aabb_collision(body1, body2):
  return aabb_overlap(aabb_from_body(body1), aabb_from_body(body2))

def body_collision(env, body1, body2=None):
  if body2 is not None:
    if is_cylinder(body1) and is_cylinder(body2): # TODO - ode cylinder collision checks are having issues
      return aabb_collision(body1, body2)
    return env.CheckCollision(body1, body2)
  for body2 in env.GetBodies():
    if body2 != body1 and body_collision(env, body1, body2):
      return True
  return False

def set_body(env, body, point, theta):
  set_point(body, point)
  set_quat(body, quat_from_axis_angle(0, 0, theta))
  env.Add(body)

def place_body(env, body, (x, y, theta), base_name): # TODO - make it place on the relative table
  body_aabb = aabb_from_body(body)
  base_aabb = aabb_from_body(env.GetKinBody(base_name))
  z = get_point(body)[2] - (body_aabb.pos()[2] - body_aabb.extents()[2]) + \
      (base_aabb.pos()[2] + base_aabb.extents()[2]) + BODY_PLACEMENT_Z_OFFSET
  set_point(body, (x, y, z))
  set_quat(body, quat_from_axis_angle(0, 0, theta))
  env.Add(body)

def place_body(env, body, (x, y, theta), base_name): # TODO - make it place on the relative table
  body_aabb = aabb_from_body(body)
  base_aabb = aabb_from_body(env.GetKinBody(base_name))
  z = get_point(body)[2] - (body_aabb.pos()[2] - body_aabb.extents()[2]) + \
      (base_aabb.pos()[2] + base_aabb.extents()[2]) + BODY_PLACEMENT_Z_OFFSET
  set_point(body, (x, y, z))
  set_quat(body, quat_from_axis_angle(0, 0, theta))
  env.Add(body)

def place_xyz_body(env, body, (x, y, z, theta), base_name): # TODO - make it place on the relative table
  body_aabb = aabb_from_body(body)
  base_aabb = aabb_from_body(env.GetKinBody(base_name))
  z = get_point(body)[2] - (body_aabb.pos()[2] - body_aabb.extents()[2]) + \
      (base_aabb.pos()[2] + base_aabb.extents()[2]) + BODY_PLACEMENT_Z_OFFSET + z
  set_point(body, (x, y, z))
  set_quat(body, quat_from_axis_angle(0, 0, theta))
  env.Add(body)

def place_body_on_floor(body, (x, y, theta)):
  aabb = aabb_from_body(body)
  z = get_point(body)[2] - (aabb.pos()[2] - aabb.extents()[2]) + BODY_PLACEMENT_Z_OFFSET
  set_point(body, (x, y, z))
  set_quat(body, quat_from_axis_angle(0, 0, theta))
  get_env().Add(body)

def randomly_place_region(env, body, region):
  if env.GetKinBody(get_name(body)) is None: env.Add(body)
  while True:
    set_quat(body, quat_from_z_rot(uniform(0, 2*PI)))
    aabb = aabb_from_body(body)
    cspace = region.cspace(aabb)
    if cspace is None: continue
    set_point(body, np.array([uniform(*range) for range in cspace]+[region.z + aabb.extents()[2] + BODY_PLACEMENT_Z_OFFSET]) - aabb.pos() + get_point(body))
    if not body_collision(env, body): return

def randomly_place_body(env, body, table_names):
  randomly_place_region(env, body, AARegion.create_on_body(env.GetKinBody(choice(table_names))))

def randomly_place_region_with_constraint(env,body,region,const,max_trial=1000):
  if env.GetKinBody(get_name(body)) is None: env.Add(body)

  target_pose = copy.deepcopy(const)
  for _ in range(max_trial):
    aabb   = aabb_from_body(body) # note: how come calling this sets objects back in the region?
    cspace = region.cspace(aabb)
    # check if there is no constraint; if there isnt, sample one
    if const[0] == None: # sample x
      target_pose[0] = uniform(*cspace[0])-aabb.pos()[0]+get_point(body)[0]             
    if const[1] == None: # sample y
      target_pose[1] = uniform(*cspace[1])-aabb.pos()[1]+get_point(body)[1]              
    if const[2] == None: # sample theta
      set_quat(body, quat_from_z_rot(uniform(0, 2*PI))) # sample th
    else:
      # note: pose_from_base_values use quat_from_z_rot on theta; this indicates const[2] is the rotation about z-axis
      set_quat(body, quat_from_z_rot(const[2]))

    target_pose[2] = region.z + aabb.extents()[2] + BODY_PLACEMENT_Z_OFFSET - aabb.pos()[2] + get_point(body)[2]
    set_point(body,target_pose)
    if not body_collision(env, body): 
      return
  return True
  

def randomly_place_on_floor(env, body, floor='floorwalls'):
  if env.GetKinBody(get_name(body)) is None: env.Add(body)
  while True:
    set_quat(body, quat_from_z_rot(uniform(0, 2*PI)))
    aabb = aabb_from_body(body)
    region = AARegion.create_in_body(env.GetKinBody(floor))
    cspace = region.cspace(aabb)
    if cspace is None: continue
    set_point(body, np.array([uniform(*range) for range in cspace] +
                     [region.z + aabb.extents()[2] + 0.01]) - aabb.pos() + get_point(body)) # TODO - need 0.01 for some reason...
    if not body_collision(env, body): return

#################################################################

def get_name(body):
  return str(body.GetName())

def set_name(body, name):
  return body.SetName(name)

def get_filename(body):
  return str(body.GetXMLFilename()) # str(body.GetURI())

def get_color(body):
  return geometries(body)[0].GetDiffuseColor()

def set_color(body, color):
  for geometry in geometries(body):
    geometry.SetDiffuseColor(color)
    #geometry.SetAmbientColor(color) # Color when in shadow
    #geometry.SetTransparency(0)

def set_transparency(body, transparency):
  for geometry in geometries(body):
    geometry.SetTransparency(transparency)

def geometries(body):
  return [geometry for link in body.GetLinks() for geometry in link.GetGeometries()]

def has_geometry(link):
  return len(link.GetGeometries()) != 0

def geometry_types(body):
  return [geometry.GetType() for geometry in geometries(body)]

def is_box(body):
  return geometry_types(body) == [GeometryType.Box]

def is_cylinder(body):
  return geometry_types(body) == [GeometryType.Cylinder]

def is_sphere(body):
  return geometry_types(body) == [GeometryType.Sphere]

"""
def is_symmetrical(body, rot):
  if not is_upright(rot): return False # TODO - remove this later
  if is_box(body):
    # TODO - check cube vs rectangular prism
    pass
  return is_cylinder(body) or is_sphere(body)
"""

def geometry_tuples(body): # GetKinematicsGeometryHash
  tuples = []
  for link in body.GetLinks():
    for geometry in link.GetGeometries():
      ty = geometry.GetType()
      if ty == GeometryType.Box:
        tup = (str(ty), tuple(geometry.GetBoxExtents()))
      elif ty == GeometryType.Cylinder:
        tup = (str(ty), (geometry.GetCylinderHeight(), geometry.GetCylinderRadius()))
      elif ty == GeometryType.Sphere:
        tup = (str(ty), (geometry.GetSphereRadius(),))
      else:
        assert False
      tuples.append(tup)
  return frozenset(tuples)

def geometry_hash(body):
  return body.GetKinematicsGeometryHash()

def get_robot_hash(robot):
  return robot.GetRobotStructureHash()

def get_manipulator_hash(manipulator):
  return manipulator.GetKinematicsStructureHash() # GetStructureHash

def load_body(env, filename):
  return env.ReadKinBodyXMLFile(filename) # = ReadKinBodyURI

def get_mass(body):
  return sum(link.GetMass() for link in body.GetLinks())

def get_center_of_mass(body): # GetGlobalMassFrame, GetLocalCOM, GetLocalInertia, GetLocalMassFrame, GetPrincipalMomentsOfInertia
  return sum(link.GetMass()*link.GetGlobalCOM() for link in body.GetLinks())/get_mass(body)

#################################################################

def box_body(env, dx, dy, dz, name=None, color=None, transparency=None):
  body = RaveCreateKinBody(env, '')
  body.InitFromBoxes(np.array([[0, 0, .5*dz, .5*dx, .5*dy, .5*dz]]), draw=True)
  if name is not None: set_name(body, name)
  if color is not None: set_color(body, color)
  if transparency is not None: set_transparency(body, transparency)
  return body

"""
def box_body(env, dx, dy, dz, color=None):
  geometry = KinBody.Link.GeometryInfo()
  geometry._type = GeometryType.Box
  geometry._t[3,3] = dz/2
  geometry._vGeomData = [dx,dy,dz]
  if color is not None: geometry._vDiffuseColor = color
  body = RaveCreateKinBody(env,'')
  body.InitFromGeometries([geom_info])
  if name is not None: set_name(body, name)
  return geometry
"""

def cylinder_body(env, radius, dz, name=None, color=None):
  geom_info = KinBody.Link.GeometryInfo()
  geom_info._type = KinBody.Link.GeomType.Cylinder
  geom_info._t[3,3] = dz/2 # Local transformation of the geom primitive with respect to the link's coordinate system.
  geom_info._vGeomData = [radius,dz]
  # boxes - first 3 values are extents
  # sphere - radius
  # cylinder - first 2 values are radius and height
  # trimesh - none
  geom_info._bVisible = True
  geom_info._fTransparency = 0.0 # value from 0-1 for the transparency of the rendered object, 0 is opaque
  if color is not None: geom_info._vDiffuseColor = color
  body = RaveCreateKinBody(env,'')
  body.InitFromGeometries([geom_info])
  if name is not None: set_name(body, name)
  return body

def sphere_body(env, radius, name=None, color=None, transparency=None):
  geom_info = KinBody.Link.GeometryInfo()
  geom_info._type = KinBody.Link.GeomType.Sphere
  geom_info._t[3,3] = radius # Local transformation of the geom primitive with respect to the link's coordinate system.
  geom_info._vGeomData = [radius]
  geom_info._bVisible = True
  geom_info._fTransparency = 0.0 # value from 0-1 for the transparency of the rendered object, 0 is opaque
  if color is not None: geom_info._vDiffuseColor = color
  body = RaveCreateKinBody(env,'')
  body.InitFromGeometries([geom_info])
  if name is not None: set_name(body, name)
  if transparency is not None: set_transparency(body, transparency)
  return body

"""
with env:
  infobox0 = KinBody.Link.GeometryInfo()
  infobox0._type = GeometryType.Box
  infobox0._t[0,3] = 0
  infobox0._vGeomData = [0.1,0.2,0.3]
  infobox0._vDiffuseColor = [1,0,0]
  infobox1 = KinBody.Link.GeometryInfo()
  infobox1._type = GeometryType.Box
  infobox1._t[0,3] = 0.1
  infobox1._vGeomData = [0.3,0.05,0.05]
  infobox1._vDiffuseColor = [0,1,0]
  link0 = KinBody.LinkInfo()
  link0._vgeometryinfos = [infobox0, infobox1]
  link0._name = 'link0'
  link0._mapFloatParameters = {'param0':[1,2.3]}
  link0._mapIntParameters = {'param0':[4,5.6]}


  infobox2 = KinBody.Link.GeometryInfo()
  infobox2._type = GeometryType.Box
  infobox2._t[0,3] = 0
  infobox2._vGeomData = [0.1,0.2,0.3]
  infobox2._vDiffuseColor = [0,0,1]
  link1 = KinBody.LinkInfo()
  link1._vgeometryinfos = [infobox2]
  link1._name = 'link1'
  link1._mapFloatParameters = {'param0':[1,2.3]}
  link1._mapIntParameters = {'param0':[4,5.6]}
  link1._t[0,3] = 0.5

  joint0 = KinBody.JointInfo()
  joint0._name = 'j0'
  joint0._linkname0 = 'link0'
  joint0._linkname1 = 'link1'
  joint0._type = KinBody.JointType.Hinge
  joint0._vlowerlimit = [-0.5]
  joint0._vupperlimit = [1.0]
  joint0._vaxes = [[0,0,1]]

  body = RaveCreateKinBody(env,'')
  success = body.Init([link0,link1],[joint0])
"""

"""
KinBody.Init((KinBody)arg1, (object)linkinfos, (object)jointinfos)
KinBody.InitFromSpheres((KinBody)arg1, (object)spherex, (bool)draw)
KinBody.InitFromTrimesh((KinBody)arg1, (object)trimesh, (bool)draw)
KinBody.Link.InitGeometries((Link)arg1, (object)geometries)
"""
