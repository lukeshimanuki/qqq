from manipulation.bodies.bounding_volumes import aabb_base_center, aabb_from_body
from manipulation.primitives.transforms import unit_quat
from manipulation.constants import REGION_Z_OFFSET, BODY_Z_OFFSET
import numpy as np

# TODO - make static
def create_region(env, region_name, ((nx, px), (ny, py)), table_name, color=None):
  table_aabb = aabb_from_body(env.GetKinBody(table_name))
  return AARegion(region_name, ((table_aabb.pos()[0] + nx*table_aabb.extents()[0], table_aabb.pos()[0] + px*table_aabb.extents()[0]),
      (table_aabb.pos()[1] + ny*table_aabb.extents()[1], table_aabb.pos()[1] + py*table_aabb.extents()[1])),
      table_aabb.pos()[2] + table_aabb.extents()[2] + REGION_Z_OFFSET, color=color)

def create_shelf_region(env, region_name, ((nx, px), (ny, py)), table_name, z, color=(0, 0, 0, 1)):
  table_aabb = aabb_from_body(env.GetKinBody(table_name))
  return AARegion(region_name, ((table_aabb.pos()[0] + nx*table_aabb.extents()[0], table_aabb.pos()[0] + px*table_aabb.extents()[0]),
      (table_aabb.pos()[1] + ny*table_aabb.extents()[1], table_aabb.pos()[1] + py*table_aabb.extents()[1])),
      z + REGION_Z_OFFSET, color=color)

class AARegion(object):
  def __init__(self, name, box, z, color=(0,0,0,0)):
    self.name = name
    self.box = box
    self.z = z
    self.color = color
    self.draw_handle = None
  def pose(self):
    return np.concatenate([unit_quat(), .5*np.array([self.box[0][1]+self.box[0][0], self.box[1][1]+self.box[1][0]]), [self.z]])
  #def center(self):
  #  return np.average(self.box, axis=1)
  def area(self):
    return (self.box[0][1] - self.box[0][0])*(self.box[1][1] - self.box[1][0])
  def contains_point(self, point):
    return not (any(self.box[k][0] > point[k] or self.box[k][1] < point[k] \
          for k in range(2)) or self.z > point[2])
  def contains_xy(self, aabb):
    return not (any(self.box[k][0] > aabb.pos()[k]-aabb.extents()[k] or self.box[k][1] < aabb.pos()[k]+aabb.extents()[k] for k in range(2)))
  def contains(self, aabb):
    return self.contains_xy(aabb) and self.z <= aabb.pos()[2]-aabb.extents()[2]
  def on(self, aabb, epsilon=.01):
    return self.contains_xy(aabb) and self.z <= aabb.pos()[2]-aabb.extents()[2] <= self.z + epsilon
  def region_on(self, other, epsilon=.01):
    return all(self.box[k][0] <= other.box[k][0] and other.box[k][1] <=  self.box[k][1] \
          for k in range(2)) and (self.z <= other.z <= self.z + epsilon)
  def on_edge(self, aabb):
    return self.contains_point(aabb_base_center(aabb)) and not self.contains(aabb)
  def cspace(self, aabb):
    cspace = []
    for k in range(2):
      low, high = self.box[k][0] + aabb.extents()[k], self.box[k][1] - aabb.extents()[k]
      if low > high: return None
      cspace.append((low, high))
    return cspace
  def draw(self, env):
    self.draw_handle = env.drawtrimesh(points=np.array(((self.box[0][0],self.box[1][0],self.z), (self.box[0][0],self.box[1][1],self.z),
        (self.box[0][1],self.box[1][1],self.z), (self.box[0][1],self.box[1][0],self.z))),
        indices=np.array(((0,1,2), (0,3,2)),np.int64), colors=np.array((self.color)))
  def clear(self):
    self.draw_handle = None
  @staticmethod
  def create_on_body(body, color=None):
    aabb = aabb_from_body(body)
    return AARegion(body.GetName(), ((aabb.pos()[0]-aabb.extents()[0],aabb.pos()[0]+aabb.extents()[0]),
                    (aabb.pos()[1]-aabb.extents()[1],aabb.pos()[1]+aabb.extents()[1])),
                    aabb.pos()[2]+aabb.extents()[2] + BODY_Z_OFFSET, color=color)
  @staticmethod
  def create_in_body(body, color=None):
    aabb = aabb_from_body(body)
    return AARegion(body.GetName(), ((aabb.pos()[0]-aabb.extents()[0],aabb.pos()[0]+aabb.extents()[0]),
                    (aabb.pos()[1]-aabb.extents()[1],aabb.pos()[1]+aabb.extents()[1])),
                    aabb.pos()[2]-aabb.extents()[2] + BODY_Z_OFFSET, color=color)
  def __str__(self):
    return self.__class__.__name__ + '(' + self.name + ', ' + str(self.box) + ', ' + str(self.z) + ')'
  __repr__ = __str__

class AASurface(AARegion):
  SURFACE_CLEARANCE = 0.01
  def __init__(self, name, box, z, color=(0,0,0,0)):
    super(AASurface, self).__init__(name, box, z, color=color)
  def contains(self, aabb):
    aabb_base =  aabb.pos()[2]-aabb.extents()[2]
    return not (any(self.box[k][0] > aabb.pos()[k]-aabb.extents()[k] or self.box[k][1] < aabb.pos()[k]+aabb.extents()[k] \
          for k in range(2)) or self.z > aabb_base or self.z + self.SURFACE_CLEARANCE < aabb_base)
