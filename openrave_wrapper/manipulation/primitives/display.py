from itertools import product

import numpy as np

from manipulation.bodies.geometry import convex_mesh
from manipulation.primitives.transforms import trans_transform_points, point_from_trans, get_trans
from manipulation.constants import VIEWER_NAME, CAMERA_TRANS, VIEWER_POSITION, VIEWER_SIZE
from manipulation.primitives.utils import get_env, Config
from misc.numerical import positive_angle, PI
from manipulation.primitives.transforms import trans_transform_point


# NOTE -
# RGB alpha where the alpha channel governs transparency
# Set handle to None to remove
# http://openrave.org/docs/latest_stable/openravepy/examples.tutorial_plotting/

def draw_point(env, point, color=(1, 0, 0, .5), size=.04):
  return env.plot3(points=np.array((point)),
      pointsize=size, colors=np.array((color)), drawstyle=1)
  #return env.plot3(points=np.array((point)),
  #    pointsize=5, colors=np.array((color)), drawstyle=0)

def draw_line(env, point1, point2, color=(1, 0, 0, .5), width=1.5):
  return env.drawlinelist(np.array([point1, point2]), width, colors=np.array(color)) # Sequential list of lines
  #return env.drawlinestrip(np.array([point1, point2]), 1.5, colors=np.array(color)) # Connected list of lines

def draw_arrow(env, point1, point2, color=(1, 0, 0, .5)):
  return env.drawarrow(point1, point2, .01, color=np.array(color))

def draw_aabb(env, aabb, color=(1, 0, 0, .5)):
  #return env.drawbox(aabb.pos(), aabb.extents(), color=color) # NOTE - Not currently implemented in OpenRAVE
  points = np.array([aabb.pos() + np.multiply(aabb.extents(), np.array(signs)) # TODO - use points_from_aabb
      for signs in product([-1, 1], repeat=len(aabb.pos()))]).T
  return draw_mesh(env, convex_mesh(points), color=color)

def draw_oobb(env, oobb, color=(1, 0, 0, .5)):
  points = trans_transform_points(oobb.trans, np.array([oobb.aabb.pos() + np.multiply(oobb.aabb.extents(), np.array(signs)) # TODO - use points_from_aabb
      for signs in product([-1, 1], repeat=len(oobb.aabb.pos()))]).T)
  return draw_mesh(env, convex_mesh(points), color=color)

def draw_mesh(env, mesh, color=(1, 0, 0, 1)):
  return env.drawtrimesh(points=mesh.vertices, indices=mesh.indices, colors=np.array(color))

def draw_plane(env, trans, x_extent, y_extent, texture):
  return env.drawplane(trans, np.array([x_extent, y_extent]), texture)

def draw_axes(env, trans, length=.25):
    origin = np.array([0, 0, 0])
    x = length*np.array([1, 0, 0])
    y = length*np.array([0, 1, 0])
    z = length*np.array([0, 0, 1])
    draw_fn = draw_arrow # draw_arrow
    handles = [
      draw_fn(env, trans_transform_point(trans, origin),
                 trans_transform_point(trans, x), color=(1, 0, 0, .5)),
      draw_fn(env, trans_transform_point(trans, origin),
                 trans_transform_point(trans, y), color=(0, 1, 0, .5)),
      draw_fn(env, trans_transform_point(trans, origin),
                 trans_transform_point(trans, z), color=(0, 0, 1, .5)),
      draw_point(env, trans[:3, 3], color=(0, 0, 0, .5)),
    ]
    return handles

###########################################################################

def fix_z(q, draw_height=1.0):
  return np.concatenate([q.value[-3:-1], [draw_height]])

def angle_z(q, draw_range=(.5, 1.5)):
  point = q.value[-3:].copy()
  point[-1] = draw_range[0] + positive_angle(point[-1])*(draw_range[1] - draw_range[0])/(2*PI)
  return point

def manip_point(q):
  if isinstance(q, Config): q = q.value
  robot = get_env().GetRobots()[0]
  with robot:
    robot.SetActiveDOFValues(q)
    return point_from_trans(get_trans(robot.GetActiveManipulator()))

def variable_display(q):
  if len(q) <= 3:
    return np.concatenate([q[:2], [.1]])
  return manip_point(q)

DISPLAY_ROBOT_CONFIG = variable_display # fix_z | angle_z | manip_point

def draw_node(env, q, color=(1, 0, 0, .5)):
  return draw_point(env, DISPLAY_ROBOT_CONFIG(q), color=np.array(color))

def draw_edge(env, q1, q2, color=(1, 0, 0, .5)):
  return draw_line(env, DISPLAY_ROBOT_CONFIG(q1), DISPLAY_ROBOT_CONFIG(q2), color=color)

###########################################################################

def update_viewer():
  get_env().UpdatePublishedBodies()

def is_viewer_active(env):
  return env.GetViewer() is not None

def get_camera_trans(env):
  return env.GetViewer().GetCameraTransform()

def set_viewer_options(env, name=VIEWER_NAME, camera_trans=CAMERA_TRANS):
  if is_viewer_active(env):
    viewer = env.GetViewer()
    viewer.SetName(name)
    viewer.SetCamera(camera_trans, focalDistance=5.0)
    viewer.SetSize(*VIEWER_SIZE)
    viewer.Move(*VIEWER_POSITION)
    #print viewer.GetCameraTransform()
    #openravepy.misc.DrawAxes(env, target, dist=1.0, linewidth=1, coloradd=None)
    return True
  return False

def save_image(env, filename):
  if is_viewer_active(env):
    viewer = env.GetViewer()
    viewer.SendCommand('SetFiguresInCamera 1')
    image = viewer.GetCameraImage(VIEWER_SIZE[0], VIEWER_SIZE[1], get_camera_trans(env), [640,640,320,240]) # if 0, the size of the viewer used
    from scipy.misc import imsave
    imsave(filename, image)
    return True
  return False

###########################################################################

"""
def start_record(oracle):
  if is_viewer_active(oracle.env)
    recorder = RaveCreateModule(oracle.env, 'viewerrecorder')
    oracle.env.AddModule(recorder, 'recorder')
    #codecs = recorder.SendCommand('GetCodecs')
    filename = 'openrave.mpg'
    codec = 13 # mov
    recorder.SendCommand('Start 600 480 30 codec %d timing realtime filename %s\nviewer %s'%(codec,filename,self.env.GetViewer().GetName()))
    return True
  return False

def stop_record(oracle):
  if is_viewer_active(oracle.env)
    recorder = oracle.env.GetModule('recorder')
    recorder.SendCommand('Stop') # stop the video
    env.Remove(recorder) # remove the recorder
    return True
  return False
"""
