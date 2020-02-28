from numpy.random import triangular

from manipulation.collision.collision import *
from manipulation.constants import REGION_PLACEMENT_Z_OFFSET
from manipulation.collision.collision_primitives import obstacle_collision, object_collision
from manipulation.bodies.bounding_volumes import aabb_area

def weight_regions(oracle, body_name, region_names): # TODO - take into account the size of obstacles
  region_weights = {region_name: oracle.regions[region_name].area() for region_name in region_names}
  for obj in oracle.objects:
    if oracle.is_active(obj) and obj != body_name:
      for region in region_names:
        if oracle.region_contains(region, obj, oracle.get_pose(obj)):
          region_weights[region] -= aabb_area(oracle.get_aabb(obj))
  return region_weights

def random_region_placements(oracle, body_name, region_names, region_weights=False, max_failures=INF):
  if region_weights is True: region_weights = weight_regions(oracle, body_name, region_names)
  sample_region = lambda: (choice(region_names) if region_weights is False else sample_categorical(region_weights))

  body_saver = oracle.body_saver(body_name)
  failures = 0
  while failures <= max_failures:
    failures += 1
    quat = quat_from_z_rot(uniform(0, 2*PI))
    aabb = oracle.get_aabb(body_name, trans_from_quat(quat))
    region = oracle.get_region(sample_region())
    cspace = region.cspace(aabb)
    if cspace is None: continue
    point = np.array([uniform(*range) for range in cspace] +
                     [region.z + aabb.extents()[2] + REGION_PLACEMENT_Z_OFFSET]) - aabb.pos() # NOTE - okay because the aabb is relative to unit_trans()
    pose = pose_from_quat_point(quat, point)
    set_pose(oracle.get_body(body_name), pose)
    if not (obstacle_collision(oracle, body_name) or object_collision(oracle, body_name)):
      body_saver.Restore()
      yield Pose(pose)
      failures = 0

#################################################################

EDGE_DISTANCE = .01 # TODO - ensure the object is till on the table

class UniformEdgeDistribution(object):
  def __init__(self, surface):
    self.surface = surface
  def sample(self):
    ((x1, x2), (y1, y2)) = self.surface.box
    edge_weights = {0: (x2-x1), 1: (y2-y1), 2: (x2-x1), 3: (y2-y1)}
    index = sample_categorical(edge_weights)
    if index == 0: point = np.array([uniform(x1, x2), y1+EDGE_DISTANCE, 0])
    elif index == 1: point = np.array([x1+EDGE_DISTANCE, uniform(y1, y2), 0])
    elif index == 2: point = np.array([uniform(x1, x2), y2-EDGE_DISTANCE, 0])
    else: point = np.array([x2-EDGE_DISTANCE, uniform(y1, y2), 0])
    return point + self.surface.z*unit_z()

class ClosestEdgeDistribution(object):
  def __init__(self, surface, point):
    self.surface = surface
    self.point = point
    assert surface.contains_point(point)
    ((x1, x2), (y1, y2)) = self.surface.box
    x, y, _ = point
    perp_distances = [abs(y1 - y), abs(x1 - x), abs(y2 - y), abs(x2 - x)]
    self.index = argmin(lambda i: perp_distances[i], range(len(perp_distances)))
  def sample(self):
    ((x1, x2), (y1, y2)) = self.surface.box
    x, y, _ = self.point
    if self.index == 0: point = np.array([triangular(x1, x, x2), y1+EDGE_DISTANCE, 0])
    elif self.index == 1: point = np.array([x1+EDGE_DISTANCE, triangular(y1, y, y2), 0])
    elif self.index == 2: point = np.array([triangular(x1, x, x2), y2-EDGE_DISTANCE, 0])
    else: point = np.array([x2+EDGE_DISTANCE, triangular(y1, y, y2), 0])
    return point + self.surface.z*unit_z()

class ClosestEdgePoint(object):
  def __init__(self, surface, point):
    self.surface = surface
    self.point = point
    assert surface.contains_point(point)
    ((x1, x2), (y1, y2)) = self.surface.box
    x, y, _ = point
    perp_distances = [abs(y1 - y), abs(x1 - x), abs(y2 - y), abs(x2 - x)]
    self.index = argmin(lambda i: perp_distances[i], range(len(perp_distances)))
  def sample(self):
    ((x1, x2), (y1, y2)) = self.surface.box
    x, y, _ = self.point
    if self.index == 0: point = np.array([x, y1+EDGE_DISTANCE, 0])
    elif self.index == 1: point = np.array([x1+EDGE_DISTANCE, y, 0])
    elif self.index == 2: point = np.array([x, y2-EDGE_DISTANCE, 0])
    else: point = np.array([x2+EDGE_DISTANCE, y, 0])
    return point + self.surface.z*unit_z()

def random_edge_placements(oracle, body_name, region_name, z=None, use_quat=None, bias_point=None, max_failures=INF):
  body_saver = oracle.body_saver(body_name)
  edge_dist = ClosestEdgeDistribution(oracle.get_region(region_name), bias_point) if bias_point is not None \
      else UniformEdgeDistribution(oracle.get_region(region_name))
  failures = 0
  while failures <= max_failures:
    failures += 1
    quat = quat_from_z_rot(uniform(0, 2*PI)) if use_quat is None else use_quat
    aabb = oracle.get_aabb(body_name, trans_from_quat(quat))
    point = edge_dist.sample() + (aabb.extents()[2]+REGION_PLACEMENT_Z_OFFSET)*unit_z() - aabb.pos()
    if z is not None: point[2] = z
    pose = pose_from_quat_point(quat, point)
    set_pose(oracle.get_body(body_name), pose)
    if not (obstacle_collision(oracle, body_name) or object_collision(oracle, body_name)):
      body_saver.Restore()
      yield Pose(pose)
      failures = 0

#################################################################

def center_stackings(oracle, body_name, stackings, max_failures=INF): # NOTE - aligns the center of the bounding boxes
  body_saver = oracle.body_saver(body_name)
  failures = 0
  while failures <= max_failures:
    failures += 1
    quat = quat_from_z_rot(uniform(0, 2*PI))
    aabb = oracle.get_aabb(body_name, trans_from_quat(quat))
    base_name, base_pose = choice(stackings)
    base_aabb = oracle.get_aabb(base_name, trans_from_pose(base_pose.value))
    point = base_aabb.pos() + (base_aabb.extents()[2]+aabb.extents()[2]+REGION_PLACEMENT_Z_OFFSET)*unit_z() - aabb.pos()
    pose = Pose(pose_from_quat_point(quat, point))
    oracle.set_pose(body_name, pose)
    if not (obstacle_collision(oracle, body_name) or object_collision(oracle, body_name)):
      body_saver.Restore()
      yield pose
      failures = 0

#def random_stackings(oracle, body_name, stackings, max_failures=INF): # TODO - random placements on object

#################################################################

# TODO - record which poses lead to bad samples to avoid them
class RandomPlacementsCache(DeterminisiticCache):
  def __init__(self, oracle, poses_per_ratio=2):
    super(self.__class__, self).__init__()
    self.oracle = oracle
    self.poses_per_ratio = poses_per_ratio
  def key(self, body_name, region_name):
    return (self.oracle.get_geom_hash(body_name), region_name)
  def fn(self, body_name, region_name):
    oracle = self.oracle
    poses = []
    region = oracle.get_region(region_name)
    for _ in range(int(self.poses_per_ratio*region.area()/aabb_area(oracle.get_aabb(body_name)) + 1)):
      theta = uniform(0, 2*PI)
      quat = quat_from_z_rot(theta)
      aabb = oracle.get_aabb(body_name, trans_from_quat(quat))
      cspace = region.cspace(aabb)
      if cspace is None: continue # NOTE - does not add more placements
      ((minx, maxx), (miny, maxy)) = cspace
      x = uniform(minx, maxx)
      y = uniform(miny, maxy)
      z = region.z + aabb.extents()[2] + REGION_PLACEMENT_Z_OFFSET
      point = np.array([x, y, z]) - aabb.pos()
      poses.append(Pose(pose_from_quat_point(quat, point)))
    return poses

class GridPlacementsCache(DeterminisiticCache):
  def __init__(self, oracle, dx=.1, dy=.1, dtheta=PI/8):
    super(self.__class__, self).__init__()
    self.oracle = oracle
    self.dx = dx
    self.dy = dy
    self.dtheta = dtheta
  def key(self, body_name, region_name):
    return (self.oracle.get_geom_hash(body_name), region_name)
  def fn(self, body_name, region_name):
    oracle = self.oracle
    poses = []
    region = oracle.get_region(region_name)
    for theta in irange(0, 2*PI, self.dtheta):
      quat = quat_from_z_rot(theta)
      aabb = oracle.get_aabb(body_name, trans_from_quat(quat))
      cspace = region.cspace(aabb)
      if cspace is None: continue
      (minx, maxx), (miny, maxy) = cspace
      z = region.z + aabb.extents()[2] + REGION_PLACEMENT_Z_OFFSET
      for x in erange(minx, maxx, self.dx):
        for y in erange(miny, maxy, self.dy):
          point = np.array([x, y, z]) - aabb.pos()
          poses.append(Pose(pose_from_quat_point(quat, point)))
    return poses

def cached_region_placements(oracle, body_name, region_names, order=None, random=False):
  if random:
    poses = randomize(reduce(operator.add, (oracle.random_placement_cache(body_name, region_name) for region_name in region_names)))
  else:
    poses = randomize(reduce(operator.add, (oracle.grid_placement_cache(body_name, region_name) for region_name in region_names)))
  # TODO - merge them
  if order is not None: poses = order(poses)
  body_saver = oracle.body_saver(body_name)
  for pose in poses:
    oracle.set_pose(body_name, pose)
    if not hasattr(pose, 'obstacle_collision'):
      pose.obstacle_collision = obstacle_collision(oracle, body_name)
    if not (pose.obstacle_collision or object_collision(oracle, body_name)): # TODO - cache object_collision as well
      body_saver.Restore()
      yield pose

#################################################################

# NOTE - combined angle + random sampling
def grid_search_region_placement(oracle, object_name, region_name, attempts=INF, dtheta=PI/16):
  obj = oracle.bodies[object_name]
  region = oracle.regions[region_name]
  with oracle.body_saver(object_name):
    orientations = [] #deque()
    for theta in irange(2*pi, step=dtheta): # TODO - binary search over successively smaller dthetas?
      set_quat(obj, quat_from_axis_angle(0, 0, theta))
      aabb = aabb_from_body(obj)
      ranges = []
      for k in range(2):
        low, high = region.box[k][0] + aabb.extents()[k], region.box[k][1] - aabb.extents()[k]
        if low > high: break
        ranges.append((low, high))
      if len(ranges) == 2:
        orientations.append((theta, tuple(ranges)))

    while len(orientations) != 0:
      theta, ranges = choice(orientations)
      set_quat(obj, quat_from_axis_angle(0, 0, uniform(0, 2*PI)))
      point = [uniform(low, high) for low, high in ranges] + [region.z + aabb.extents()[2] + REGION_PLACEMENT_Z_OFFSET]
      set_point(obj, point + get_point(obj) - aabb.pos())
      if not (obstacle_collision(oracle, object_name) or object_collision(oracle, object_name)):
        return oracle.get_pose(object_name)
  return None

# TODO - make placement method that computes CSPACE and places at the vertices
# TODO - use RADIUS 2D to construct circular cspace and sample at distance away

