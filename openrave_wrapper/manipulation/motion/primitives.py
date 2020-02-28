from manipulation.motion.cspace import cspace_sample, cspace_sample_collisions, cspace_partial_sample, \
  cspace_distance, cspace_distance_2, linear_interpolation, cspace_neighbors, cspace_collision_free_sample, cspace_l1_distance, \
  limited_linear_interpolation, sequential_linear_interpolation
from manipulation.primitives.utils import Config
from openravepy import CollisionReport, CollisionOptions

def deterministic_sample_fn(body, collisions=False, seed=False): # TODO - collisions, seed
  import ghalton
  sequencer = ghalton.Halton(body.GetActiveDOF())
  #sequencer = ghalton.GeneralizedHalton(body.GetActiveDOF(), 68)
  lower_limits, upper_limits = body.GetActiveDOFLimits()
  return lambda: sequencer.get(1)[0]*(upper_limits - lower_limits) + lower_limits

def sample_fn(body, collisions=False):
  return lambda: cspace_sample(body) if not collisions else cspace_sample_collisions(body)

def collision_free_sample_fn(env, body, self_collisions=True):
  return lambda: cspace_collision_free_sample(env, body, self_collisions=self_collisions)

def partial_sample_fn(body, q, indices):
  return lambda: cspace_partial_sample(body, q, indices)

def distance_fn(body):
  return lambda q1, q2: cspace_distance_2(body, q1, q2)

def sqrt_distance_fn(body):
  return lambda q1, q2: cspace_distance(body, q1, q2)

def l1_distance_fn(body):
  return lambda q1, q2: cspace_l1_distance(body, q1, q2)

def extend_fn(body):
  return lambda q1, q2: linear_interpolation(body, q1, q2)

def sequential_extend_fn(body, limit):
  return lambda q1, q2: sequential_linear_interpolation(body, q1, q2, limit)

def limited_extend_fn(body, limit):
  return lambda q1, q2: limited_linear_interpolation(body, q1, q2, limit=limit)

def collision_fn(env, body, check_self=False):
  def fn(q):
    body.SetActiveDOFValues(q)
    return env.CheckCollision(body) or (check_self and body.CheckSelfCollision())
  return fn

def collision_distance_fn(env, body):
  def fn(q):
    body.SetActiveDOFValues(q)
    report = CollisionReport()
    collision = env.CheckCollision(body, report=report)
    return collision, report.minDistance
  return fn

def neighbors_fn(body, goal=None):
  return lambda q: cspace_neighbors(body, q, goal=goal)

###########################################################################

def q_sample_fn(body, collisions=False):
  s_fn = sample_fn(body, collisions=collisions)
  return lambda: Config(s_fn())

def q_distance_fn(body):
  d_fn = distance_fn(body)
  return lambda q1, q2: d_fn(q1.value, q2.value)

def q_extend_fn(body):
  e_fn = extend_fn(body)
  return lambda q1, q2: (Config(q) for q in e_fn(q1.value, q2.value))

def q_collision_fn(env, body, check_self=False):
  c_fn = collision_fn(env, body, check_self=check_self)
  return lambda q: c_fn(q.value)