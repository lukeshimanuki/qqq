from misc.utils import read_pickle
from problem import GENERATED_DIR, EXTENSION
from inspect import getmembers, isfunction, getargspec
from manipulation.primitives.utils import get_env
from manipulation.problems.problem import ManipulationProblem
import fixed
import distribution
import glob

modules = [fixed, distribution]

def list_generated_problems():
  return [f[len(GENERATED_DIR):-len(EXTENSION)] for f in glob.glob(GENERATED_DIR + '*' + EXTENSION)]

def load_generated_problem(problem_name):
  problem = read_pickle(GENERATED_DIR + problem_name + EXTENSION)
  for attribute, value in ManipulationProblem(None).__dict__.items(): # TODO - older problems (ex 4tables) don't have all the problem attributes
    if not hasattr(problem, attribute):
      setattr(problem, attribute, value)
  return problem

def list_fixed_problems():
  return [name for module in modules for name, value in getmembers(module)
          if isfunction(value) and getargspec(value)[0] == ['env']]

def load_fixed_problem(problem_name):
  for module in modules:
    if hasattr(module, problem_name):
      return getattr(module, problem_name)(get_env()) # dir(fixed)
  raise ValueError(problem_name)

def list_problems():
  return list_fixed_problems() + list_generated_problems()

def load_problem(problem_name):
  if problem_name in list_fixed_problems():
    return load_fixed_problem(problem_name)
  return load_generated_problem(problem_name)
