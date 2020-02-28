from manipulation.primitives.utils import *
from openravepy import TriMesh, AABB

# NOTE - individual points are in each column

def len_rows(matrix):
  return matrix.shape[0]

def len_cols(matrix):
  return matrix.shape[1]

def points_iter(points):
  for i in range(points.shape[1]):
    yield points[:,i].T

def get_radius2D(body):
  points = points_from_aabb(aabb_from_body(body))
  center = get_point(body)
  return max(length((center - point)[:2]) for point in points_iter(points))

def get_radius(body):
  points = points_from_aabb(aabb_from_body(body))
  center = get_point(body)
  return max(length((center - point)) for point in points_iter(points))

#################################################################

def mesh_from_body(body):
  return get_env().Triangulate(body)

def mesh_apply_trans(mesh, trans):
  return TriMesh(np.dot(np.hstack([mesh.vertices, np.ones([len_rows(mesh.vertices), 1])]), trans.T)[:,:3], mesh_indices(mesh))

def mesh_indices(mesh):
  return mesh.indices

def mesh_vertices(mesh):
  return mesh.vertices.T

def mesh_vertex(mesh, index):
  return mesh.vertices[index, :]

#################################################################

# TODO - extend AABB class

def aabb_center(aabb):
  return aabb.pos()

def aabb_base_center(aabb):
  return aabb.pos() - aabb.extents()[2]*unit_z()

def aabb_dims(aabb):
  return 2*aabb.extents()

def aabb_min(aabb):
  return aabb.pos() - aabb.extents()

def aabb_max(aabb):
  return aabb.pos() + aabb.extents()

def radius2D_aabb(aabb):
  return length(aabb.extents()[:2])

def radius_aabb(aabb):
  return length(aabb.extents())

def aabb_from_body(body):
  return body.ComputeAABB()

def aabb_from_extrema(small, large):
  if np.all(small < large):
    return AABB((small+large)/2., (large-small)/2.)
  return None

def point_in_aabb(aabb, point):
  return np.all(point >= aabb_min(aabb)) and np.all(point <= aabb_max(aabb))

def aabb_contains(aabb1, aabb2): # testing aabb1 contains aabb2
  return np.all((aabb_min(aabb1) <= aabb_min(aabb2))) and np.all((aabb_max(aabb2) <= aabb_max(aabb1)))

def aabb_overlap(aabb1, aabb2):
  return np.all((aabb_min(aabb1) <= aabb_max(aabb2))) and np.all((aabb_min(aabb2) <= aabb_max(aabb1)))

def fast_aabb_overlap(aabb1_min, aabb1_max, aabbs):
  return any(np.all(aabb1_min <= aabb2_max) and np.all(aabb2_min <= aabb1_max)
             for aabb2_min, aabb2_max in aabbs)

def aabb_stacked(aabb1, aabb2, epsilon=.01): # aabb1 on aabb2
  return aabb_min(aabb1)[2] >= aabb_max(aabb2)[2] and aabb_min(aabb1)[2] <= aabb_max(aabb1)[2] + epsilon and \
      np.all((aabb_center(aabb1) >= aabb_min(aabb2))[:2]) and np.all((aabb_center(aabb1) <= aabb_max(aabb2))[:2])

def aabb_on_edge(aabb1, aabb2):
  return aabb_stacked(aabb1, aabb2) and not (np.all((aabb_min(aabb1) <= aabb_min(aabb2))[:2])
                                             and np.all((aabb_max(aabb2) <= aabb_max(aabb1))[:2]))

def aabb_area(aabb):
  return np.prod(aabb_dims(aabb)[:2])

def aabb_volume(aabb):
  return np.prod(aabb_dims(aabb))

def aabb_extrema(aabb):
  return np.array([aabb_min(aabb), aabb_max(aabb)]).T

def aabb_from_points(points):
  return aabb_from_extrema(np.min(points, axis=1), np.max(points, axis=1))

def points_from_aabb(aabb):
  return np.array([aabb.pos() + np.multiply(aabb.extents(), np.array(signs))
      for signs in product([-1, 1], repeat=len(aabb.pos()))]).T

def xy_points_from_aabb(aabb):
  return [aabb.pos() + np.multiply(aabb.extents(), np.array(list(signs) + [0]))
      for signs in product([-1, 1], repeat=2)]

def aabb_union(aabbs):
  return aabb_from_points(np.hstack([aabb_extrema(aabb) for aabb in aabbs]))

def aabb_intersection(aabbs):
  points = np.hstack(aabb_extrema(aabb) for aabb in aabbs)
  return aabb_from_extrema(np.max(points, axis=1), np.min(points, axis=1))

def aabb_apply_trans(aabb, trans):
  return AABB(point_from_trans(trans) + aabb.pos(), np.dot(np.abs(rot_from_trans(trans)), aabb.extents())) # NOTE - works because sign cancellation

#################################################################

OOBB = namedtuple('OOBB', ['aabb', 'trans'])

def oobb_from_points(points): # NOTE - not necessarily optimal
  mu = np.resize(np.mean(points, axis=1), (3,1))
  centered = points - mu
  u, _, _ = np.linalg.svd(centered)
  if np.linalg.det(u) < 0: u[:,1] *= -1

  aabb = aabb_from_points(np.dot(u.T, centered))
  trans = np.identity(4)
  trans[:3,:3] = u
  trans[:3,3] = mu.T
  return OOBB(aabb, trans)

def oobb2D_from_points(points): # NOTE - not necessarily optimal
  mu = np.resize(np.mean(points, axis=1), (3,1))
  u, _, _ = np.linalg.svd((points - mu)[:2])
  if np.linalg.det(u) < 0: u[:,1] *= -1
  rot = np.identity(3)
  rot[:2,:2] = u

  aabb = aabb_from_points(np.dot(rot.T, points-mu))
  trans = np.identity(4)
  trans[:3,:3] = rot
  trans[:3,3] = mu.T
  return OOBB(aabb, trans)

def aabb_from_oobb(oobb):
  return aabb_apply_trans(oobb.aabb, oobb.trans)
