from scipy.spatial import ConvexHull  # NOTE - cKDTree is faster, but KDTree can do all pairs closest
import numpy as np
from openravepy import TriMesh, databases
from misc.functions import flatten

def are_points_inside(oracle, points):
  return oracle.cd_model.testPointsInside(points)

def transform_hull(hull, trans):
  return databases.convexdecomposition.ConvexDecompositionModel.transformHull(trans, hull)

def mesh_from_hulls(hulls):
  all_vertices = np.zeros((0,3),np.float64)
  all_indices = np.zeros((0,3),int)
  for hull in hulls:
    all_indices = np.r_[all_indices,hull[1]+len(all_vertices)]
    all_vertices = np.r_[all_vertices,hull[0]]
  return TriMesh(all_vertices, all_indices)
  #return databases.convexdecomposition.ConvexDecompositionModel.generateTrimeshFromHulls(hulls)

def merge_meshes(meshes):
  return mesh_from_hulls([(m.vertices, m.indices) for m in meshes])

def link_hulls(oracle, link):
  all_hulls = []
  for index, hulls in oracle.cd_model.linkgeometry[link.GetIndex()]:
    for points, indices, planes in hulls:
      all_hulls.append((points, indices))
  return all_hulls

def link_mesh(oracle, link):
  return mesh_from_hulls(link_hulls(oracle, link))

def convex_hull_vertices(hull): # NOTE - old scipy uses points while the new scipy uses vertices
  if not hasattr(hull, 'vertices'):
    return sorted(set(flatten(hull.simplices)))
  return hull.vertices

def convex_hull(verts):
  hull = ConvexHull(verts)
  new_indices = {i: ni for ni, i in enumerate(convex_hull_vertices(hull))}
  return hull.points[convex_hull_vertices(hull), :], np.vectorize(lambda i: new_indices[i])(hull.simplices)

def convex_mesh(verts):
  return TriMesh(*convex_hull(verts[:3, :].T))

def convex_mesh2D(verts):
  zmin, zmax = min(verts[2, :]), max(verts[2, :])
  hull = ConvexHull(verts[:2, :].T)
  vertices2D = hull.points[hull.vertices, :]
  n = len(vertices2D)
  vertices = np.vstack([
    np.hstack([vertices2D, zmin * np.ones([n, 1])]),
    np.hstack([vertices2D, zmax * np.ones([n, 1])])
  ])
  indices = np.array([(0, i, i + 1) for i in range(1, n - 1)] + \
                     [(n, i, i + 1) for i in range(n + 1, 2 * n - 1)] + \
                     [(i - 1, i, i + n) for i in range(n)] + \
                     [(n + i - 1, n + i, i - 1) for i in range(n)])
  return TriMesh(vertices, indices)

def minkowski_addition(verts1, verts2):
  return np.hstack([verts1 + verts2[:, i:i + 1] for i in range(verts2.shape[1])])

def cspace_object_mesh(robot_mesh, object_mesh):
  return convex_mesh(minkowski_addition(-robot_mesh.vertices.T, object_mesh.vertices.T))

def cspace_object_mesh2D(robot_mesh, object_mesh):  # TODO - can be more efficient by treating z separately
  return convex_mesh2D(minkowski_addition(-robot_mesh.vertices.T, object_mesh.vertices.T))

"""
def cspace_interior(A, B): # Only for convex B
  def vertex_from_planes(planes):
    verts = []
    one = np.ones(1)
    for inds in itertools.combinations(range(planes.shape[0]), 3):
      try:
        v = np.linalg.solve(planes[inds,:3], -planes[inds,3])
      except: continue
      pt = np.hstack([v, one])
      if np.all(np.dot(planes, pt) <= tiny):
        verts.append(pt)
    return np.vstack(verts).T if verts else None

  aVerts = A.vertices()
  bPlanes = B.planes()
  fxv = np.dot(bPlanes[:,:3], aVerts[:3,:])
  off = np.resize(np.max(fxv, axis=1), (bPlanes.shape[0], 1))
  cPlanes = np.hstack([bPlanes.copy()[:,:3], bPlanes[:,3:4]+off])
  cVerts = vertex_from_planes(cPlanes)
  return shapes.Polygon(cVerts) if not cVerts is None else None

def cspace_interior2D(A, B): # Only for convex B
  def verts_from_planes2D(planes): # only side planes
    verts = []
    one = np.array([0., 1.])
    for inds in itertools.combinations(range(planes.shape[0]), 2):
      try:
        v = np.linalg.solve(planes[inds,:2], planes[inds,3])
      except: continue
      pt = np.hstack([v, one])
      if np.all(np.dot(planes, pt) <= tiny):
        verts.append(pt)
    return geom.convexHullVertsXY(np.vstack(verts).T) if verts else None

  Az = A.zRange()
  Bz = B.zRange()
  zRange = (Bz[0] - Az[0], Bz[1]-Az[1])
  if zRange[1] < zRange[0]:
    return None
  aVerts = A.vertices()
  bPlanes = B.planes()
  fxv = np.dot(bPlanes[:,:3], aVerts[:3,:])
  off = np.resize(np.max(fxv, axis=1), (bPlanes.shape[0], 1))
  cPlanes = np.hstack([bPlanes.copy()[:,:3], bPlanes[:,3:4]+off])
  indices = [i for i in range(cPlanes.shape[0]) if abs(cPlanes[i,2]) < 0.01]
  cVerts = verts_from_planes2D(cPlanes[indices, :])
  return shapes.Polygon(cVerts, zRange) if not cVerts is None else None
"""
