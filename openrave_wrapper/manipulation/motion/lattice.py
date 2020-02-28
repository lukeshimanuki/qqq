from heapq import heappop, heappush
from time import time

from openravepy import openravepy_int
from recordclass import recordclass
import numpy as np
from misc.numerical import INF

from motion_planners.discrete import astar
from manipulation.primitives.savers import collision_saver
from manipulation.motion.primitives import distance_fn, neighbors_fn, collision_fn




# TODO - bidirectional discrete search
# TODO - allow grid to be larger than resolution

Node = recordclass('Node', ['q', 'start_dist', 'goal_dist', 'collision',
                            'g_dist', 'g_step',
                            'it', 'num', 'parent', 'exp']) #, 'start_diff', 'goal_diff']) # TODO - inner products

def retrace2(visited, q):
  if q is None:
    return []
  return retrace2(visited, visited[tuple(q)].parent) + [q]

# TODO - lazy version of this which decides which states to check collisions against

#def best_first(start, goal, distance, neighbors, collision, cost=lambda n: n.g + n.h, max_iterations=INF):
"""
def best_first(start, goal, distance, neighbors, collision, select, max_iterations=INF):
  if collision(start) or collision(goal): return None, [], {}
  iterations = 0
  visited = {tuple(start): Node(start, distance(start, start), distance(start, goal), False,
                                0, 0,
                                iterations, 0, None, None)}
  colliding = set()
  expanded = []
  queue = [start]
  while len(queue) != 0 and iterations < max_iterations:
    current = select(expanded, queue, visited)
    iterations += 1
    expanded.append(current)
    visited[tuple(current)].exp = iterations
    if tuple(current) == tuple(goal):
      return retrace2(visited, current), expanded, visited
    for next in neighbors(current):
      if tuple(next) not in visited and tuple(next) not in colliding:
        if not collision(next): # TODO - make edges for real (and store bad edges)
          visited[tuple(next)] = Node(next, distance(start, next), distance(next, goal), False,
                                      visited[tuple(current)].g_dist + distance(current, next), visited[tuple(current)].g_step + 1,
                                      iterations, len(visited), current, None)
          queue.append(next)
        else:
          colliding.add(tuple(next))
  return None, expanded, visited
"""

# TODO - fix the tuple numpy array thing
def lazy_best_first(start, goal, distance, neighbors, collision, select, max_iterations=INF):
  if collision(goal): return None, [], {}
  iterations = 0
  expanded = []
  queue = [start]
  visited = {tuple(start): Node(start, distance(start, start), distance(start, goal), None,
                                0, 0,
                                iterations, 0, None, None)}
  while len(queue) != 0 and iterations < max_iterations:
    current = select(expanded, queue, visited)
    iterations += 1 # TODO - should I count collisions as expansions? Probably
    expanded.append(current)
    if collision(current):
      visited[tuple(current)].collision = True
      continue
    visited[tuple(current)].collision = False
    visited[tuple(current)].exp = iterations
    if tuple(current) == tuple(goal):
      return retrace2(visited, current), expanded, visited
    for next in neighbors(current):
      if tuple(next) not in visited:
        visited[tuple(next)] = Node(next, distance(start, next), distance(next, goal), None,
                                    visited[tuple(current)].g_dist + distance(current, next), visited[tuple(current)].g_step + 1,
                                    iterations, len(visited), current, None)
        queue.append(next)
      #else:
      #  print 'revisited' # NOTE - avoids revisiting
  return None, expanded, visited

# TODO - how do I combine revolute joints?
COLLISION_WEIGHT = .5
EXPANSION_WEIGHT = .5
DISABLE_STATE_FEATURES = False
from misc.numerical import PI

def get_scale_fn(robot):
  circular_joints = [i for i, index in enumerate(robot.GetActiveDOFIndices()) if robot.GetJointFromDOFIndex(index).IsCircular(0)]
  lower_limits, upper_limits = robot.GetActiveDOFLimits()
  for i in circular_joints:
    lower_limits[i] = -PI
    upper_limits[i] = PI
  #robot.SetActiveDOFLimits() # NOTE - no need to set the limits

  def scale(q): # TODO - normalize for real as well
    wrapped_q = np.copy(q)
    for i in circular_joints:
      wrapped_q[i] = (wrapped_q[i] - lower_limits[i])%(upper_limits[i] - lower_limits[i]) + lower_limits[i]
    return 2*(wrapped_q - lower_limits)/(upper_limits - lower_limits) - 1
  return scale

class StateSpace(object):
  def __init__(self, start, goal, distance, scale, max_iterations):
    self.start = start
    self.goal = goal
    self.distance = distance
    self.scale = scale
    self.max_iterations = max_iterations # TODO - replace this information with just the problem
    self.start_time = time()
    self.processed = []
    self.collided = []
    self.expanded = []
    self.visited = {} # TODO - should I add the start automatically?

    self.min_goal_distance = distance(start, goal)
    self.min_goal_config = start

    #self.max_goal_distance = 0
    self.max_start_distance = 0
    self.max_g = 0
    self.collision_vector = None
    self.expanded_vector = None
    self.features = [self.get_feature]
  def __len__(self):
    return len(self.visited)
  @property
  def iterations(self):
    #return len(self.expansions)
    return len(self.expanded) + len(self.collided)
  @property
  def time(self):
    return time() - self.start_time
  @property
  def get_feature(self):
    if DISABLE_STATE_FEATURES:
      return np.array([0])
    #return np.array([float(self.max_iterations - self.iterations)/self.max_iterations])
    #return np.array([float(self.max_iterations - self.iterations)/self.max_iterations, # NOTE - redundant
    #                 float(len(self.expanded))/self.max_iterations,
    #                 float(len(self.collided))/self.max_iterations,
    #                 .01*self.min_goal_distance])
    return np.concatenate([[float(self.max_iterations - self.iterations)/self.max_iterations, # NOTE - redundant
                     float(len(self.expanded))/self.max_iterations,
                     float(len(self.collided))/self.max_iterations,
                     .01*self.min_goal_distance], self.scale(self.min_goal_config),
                     self.scale(self.start), self.scale(self.goal)])
    #return np.array([self.time(), len(self.collided)])
  @staticmethod
  def num_features(n):
    if DISABLE_STATE_FEATURES:
      return 1
    #return 4
    return 4 + 3*n
    #return 4 + len(self.start) + len(self.goal)
  def visit(self, state, parent=None):
    np_state = np.array(state)
    if parent is not None:
      self.visited[state] = Node(np_state, self.distance(self.start, np_state), self.distance(np_state, self.goal), None,
                                 self.visited[parent].g_dist + self.distance(self.visited[parent].q, np_state),
                                 self.visited[parent].g_step + 1,
                                 self.iterations, len(self.visited), parent, None)
    else:
      self.visited[state] = Node(np_state, self.distance(self.start, np_state), self.distance(np_state, self.goal), None,
                                  0, 0, self.iterations, len(self.visited), None, None)
  def collision(self, state):
    self.visited[state].collision = True
    self.collided.append(state)
    self.processed.append(state)
    #if self.collision_vector is None:
    #  self.collision_vector = state
    #else:
    #  self.collision_vector = COLLISION_WEIGHT*state + (1-COLLISION_WEIGHT)*self.collision_vector
    self.features.append(self.get_feature)
  def expansion(self, state):
    self.visited[state].collision = False
    self.visited[state].exp = self.iterations
    self.expanded.append(state)
    self.processed.append(state)
    goal_distance = self.distance(state, self.goal)
    if goal_distance < self.min_goal_distance:
      self.min_goal_distance = goal_distance
      self.min_goal_config = state

    #self.max_goal_distance = max(self.max_goal_distance, goal_distance)
    start_distance = self.distance(self.start, state)
    self.max_start_distance = max(self.max_start_distance, start_distance)
    self.max_g = max(self.max_g, self.visited[state].g_dist)
    #if self.expanded_vector is None:
    #  self.expanded_vector = state
    #else:
    #  self.expanded_vector = EXPANSION_WEIGHT*state + (1-EXPANSION_WEIGHT)*self.expanded_vector
    self.features.append(self.get_feature)

# TODO - fix the tuple numpy array thing
def state_best_first(start, goal, distance, neighbors, collision, select, scale, max_iterations=INF):
  state_space = StateSpace(start, goal, distance, scale, max_iterations)
  if collision(goal): return None, state_space
  current = tuple(start)
  queue = [current] # TODO - make this a deque
  state_space.visit(current)
  while len(queue) != 0 and state_space.iterations < max_iterations:
    current = select(queue, state_space)
    if collision(current):
      state_space.collision(current)
      continue
    state_space.expansion(current)
    if current == tuple(goal):
      return retrace2(state_space.visited, current), state_space
    for neighbor in neighbors(np.array(current)):
      successor = tuple(neighbor)
      if successor not in state_space.visited:
        queue.append(successor)
        state_space.visit(successor, parent=current)
  return None, state_space

# TODO - support adding or removing priority functions dynamically
class MultiPriorityQueue(object):
  def __init__(self, priority_fns, strategy):
    self.priority_fns = priority_fns
    self.strategy = strategy
    self.queues = [[] for _ in self.priority_fns]
    self.popped = set()
    self.size = 0
  def __len__(self):
    return self.size
  def push(self, config, node):
    for i in range(len(self.priority_fns)):
      heappush(self.queues[i], (self.priority_fns[i](node), config)) # TODO - lazily compute this
    self.size += 1
  def pop(self):
    i = self.strategy()
    while len(self.queues[i]) != 0:
      _, config = heappop(self.queues[i])
      if tuple(config) not in self.popped:
        self.popped.add(tuple(config))
        self.size -= 1
        return config
    raise ValueError('MultiPriorityQueue is empty.')

# Maybe convert the np arrays to tuples as soon as they are generated

def abstract_lazy_best_first(start, goal, distance, neighbors, collision, queue, max_iterations=INF):
  if collision(goal): return None, [], {}
  iterations = 0
  expanded = []
  current = tuple(start)
  visited = {current: Node(start, distance(start, start), distance(start, goal), None,
                                0, 0, iterations, 0, None, None)}
  queue.push(current, visited[current])
  while len(queue) != 0 and iterations < max_iterations:
    current = queue.pop()
    iterations += 1 # TODO - should I count collisions as expansions? Probably
    expanded.append(current)
    if collision(current):
      visited[current].collision = True
      continue
    visited[current].collision = False
    visited[current].exp = iterations
    if tuple(current) == tuple(goal):
      return retrace2(visited, current), expanded, visited
    for neighbor in neighbors(visited[current].q):
      successor = tuple(neighbor)
      if successor not in visited:
        visited[successor] = Node(neighbor, distance(start, neighbor), distance(neighbor, goal), None,
                                    visited[current].g_dist + distance(visited[current].q, neighbor),
                                    visited[current].g_step + 1,
                                    iterations, len(visited), current, None)
        queue.push(successor, visited[successor]) # TODO - push all at once to use multi-batching
  return None, expanded, visited

def rl_motion_plan(env, cspace, goal, select, planner=abstract_lazy_best_first, self_collisions=False, max_iterations=INF):
  robot = cspace.body
  with robot:
    cspace.set_active()
    #with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs | openravepy_int.CollisionOptions.Distance):
    #with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs | openravepy_int.CollisionOptions.UseTolerance):
    with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
      #return planner(robot.GetActiveDOFValues(), goal, distance_fn(robot), neighbors_fn(robot, goal=goal),
      #               collision_fn(env, robot, check_self=self_collisions), select, max_iterations=max_iterations)
      return planner(robot.GetActiveDOFValues(), goal, distance_fn(robot), neighbors_fn(robot, goal=goal),
                     collision_fn(env, robot, check_self=self_collisions), select, get_scale_fn(robot), max_iterations=max_iterations)

def lattice_motion_plan(env, cspace, goal, planner=astar, self_collisions=False):
  robot = cspace.body
  with robot:
    cspace.set_active()
    start = robot.GetActiveDOFValues()
    with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
      path = planner(start, goal, distance_fn(robot), neighbors_fn(robot, goal=goal), collision_fn(env, robot, check_self=self_collisions))
      if path is None: return None
      return path
      #return PathTrajectory(cspace, path)

def lattice_interpolation(robot, start, goal):
  return astar(start, goal, distance_fn(robot), neighbors_fn(robot, goal=goal), lambda *args: False, cost=lambda g, h: h)
  #return astar(start, goal, l1_distance_fn(robot), neighbors_fn(robot, goal=goal), lambda *args: False, cost=lambda g, h: h)

# TODO - lattice search