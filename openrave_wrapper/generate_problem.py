import sys
import getopt

from manipulation.problems import generate
from openravepy import Environment, RaveDestroy
from manipulation.problems.problem import GENERATED_DIR
from misc.io import write_pickle
from manipulation.visualizations import execute_viewer

ENV_FORMAT = '%s.dae'
PROBLEM_FORMAT = '%s.pickle'

def save(env, problem_fn):
  while True:
    env.Reset()
    problem = problem_fn(env)
    problem_name = raw_input('Save problem?\n')
    if problem_name != '': break

  problem.name = problem_name
  problem.env_file = ENV_FORMAT%problem_name
  env_file = GENERATED_DIR + problem.env_file
  env.Save(env_file, Environment.SelectionOptions.Everything) #https://github.com/rdiankov/openrave/blob/master/test/test_collada.py
  print 'Saved', env_file

  problem_file = GENERATED_DIR + PROBLEM_FORMAT%problem_name
  write_pickle(problem_file, problem)
  print 'Saved', problem_file

def main(argv):
  HELP_MESSAGE = 'python generate_problem.py -p <problem>'
  try:
    opts, args = getopt.getopt(argv, 'hp:v', ['help', 'problem=', 'visualize'])
  except getopt.GetoptError:
    print HELP_MESSAGE
    return

  problem = None
  visualize = False
  for opt, arg in opts:
    if opt in ('-h', '--help'):
       print HELP_MESSAGE
       return
    elif opt in ('-p', '--problem'):
       problem = arg
    elif opt in ('-v', '--visualize'):
      visualize = not visualize

  if problem is None:
    print HELP_MESSAGE
    return
  if not hasattr(generate, problem):
    print problem, 'is not a valid problem'
    return

  env = Environment()
  try:
    execute = lambda: save(env, getattr(generate, problem))
    if visualize: execute_viewer(env, execute)
    else: execute()
  finally:
    if env.GetViewer() is not None:
      env.GetViewer().quitmainloop()
    RaveDestroy()

if __name__ == '__main__':
  main(sys.argv[1:])
