import sys
import getopt

sys.path.append('../..') # http://stackoverflow.com/questions/1054271/how-to-import-a-python-class-that-is-in-a-directory-above
# NOTE - python run_ir_script.py != python manipulation/inverse_reachability/run_ir_script.py
from openravepy import Environment, RaveDestroy, RaveSetDebugLevel, DebugLevel
from manipulation.primitives.display import set_viewer_options
from manipulation.inverse_reachability.pick_and_place import pap_ir_samples, pap_ir_statistics, pap_ir_oracle
from manipulation.inverse_reachability.push import push_ir_samples
from manipulation.inverse_reachability.inverse_reachability import create_custom_ir, save_custom_ir, custom_base_iterator, load_ir_database
from manipulation.bodies.bounding_volumes import aabb_from_points, aabb_min, aabb_max
from manipulation.visualizations import execute_viewer

HELP_MESSAGE = 'python %s -d <database>'%(__file__)

def script(env):
  RaveSetDebugLevel(DebugLevel.Fatal)
  set_viewer_options(env)

  """
  oracle = pap_ir_oracle(env)
  load_ir_database(oracle)
  print len(oracle.ir_database)

  #aabb = aabb_from_points(oracle.ir_database)
  aabb = oracle.ir_aabb
  print aabb_min(aabb)
  print aabb_max(aabb)
  #return
  """

  #samples = pap_ir_samples(Environment())
  #robot = env.GetRobots()[0]
  #ir_database = create_custom_ir(robot, samples)
  #save_custom_ir(robot, ir_database)

  #ir_database =

  env.Reset()
  print pap_ir_statistics(env)

  raw_input('Done!')

def main(argv):
  try:
    opts, args = getopt.getopt(argv, 'h:v', ['help', 'viewer'])
  except getopt.GetoptError as err:
    print str(err)
    print HELP_MESSAGE
    return

  viewer = False
  for opt, arg in opts:
    if opt in ('-h', '--help'):
       print HELP_MESSAGE
       return
    elif opt in ('-v', '--viewer'):
      viewer = True

  env = Environment()
  try:
    execute = lambda: script(env)
    if viewer: execute_viewer(env, execute)
    else: execute()
  finally:
    if env.GetViewer() is not None:
      env.GetViewer().quitmainloop()
    RaveDestroy()

if __name__ == '__main__':
  main(sys.argv[1:])