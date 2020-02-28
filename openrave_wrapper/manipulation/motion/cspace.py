from manipulation.bodies.bodies import *
from openravepy import RaveGetAffineDOFFromIndex, RaveGetIndexFromAffineDOF, RaveGetAffineDOF, RaveGetAffineDOFValuesFromTransform

# noinspection PyUnresolvedReferences,PyUnresolvedReferences,PyUnresolvedReferences,PyUnresolvedReferences,PyUnresolvedReferences,PyUnresolvedReferences
class CSpace(object):
  def __init__(self, body, indices=[], affine_mask=None, rotation_axis=None):
    self.body = body
    assert (len(indices) != 0) or (affine_mask is not None)
    self.indices = indices
    self.affine_mask = affine_mask
    self.rotation_axis = rotation_axis
  def num_dofs(self):
    return len(self.indices) + (RaveGetAffineDOF(self.affine_mask) if self.affine_mask is not None else 0)
  def trans(self, data, spec): # Data is a length 7 array representing the affine pose in the mask format
    return spec.ExtractTransform(np.identity(4), data, self.body)
  def affine_config(self, data, spec):
    return get_affine_values(self.trans(data, spec), self.affine_mask, rotation_axis=self.rotation_axis)
  def joint_config(self, data, spec): # Data is a length 15 array for the configuration of the arms and torso
    return spec.ExtractJointValues(data, self.body, self.indices)
  def config(self, data, spec):
    if len(self.indices) != 0:
      if self.affine_mask is not None:
        return np.concatentate([self.joint_config(data, spec), self.affine_config(data, spec)])
      return self.joint_config(data, spec)
    return self.affine_config(data, spec)
  #def spec(self):
  #  if self.affine_mask is not None:
  #    return RaveGetAffineConfigurationSpecification(self.affine_mask)
  #  return self.body.GetConfigurationSpecificationIndices(self.indices)
  def set_active(self):
    if self.affine_mask is None:
      self.body.SetActiveDOFs(self.indices)
    elif self.rotation_axis is None:
      self.body.SetActiveDOFs(self.indices, self.affine_mask)
    else:
      self.body.SetActiveDOFs(self.indices, self.affine_mask, self.rotation_axis)
  @staticmethod
  def robot_base_xy(body): #base_config = data[:2]
    return CSpace(body, affine_mask=DOFAffine.X|DOFAffine.Y)
  @staticmethod
  def robot_base(body): #base_config = data[:3]
    return CSpace(body, affine_mask=DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis, rotation_axis=[0, 0, 1])
  @staticmethod
  def robot_arm(*arms): #arm.GetArmConfigurationSpecification()
    assert len(arms) != 0
    return CSpace(arms[0].GetRobot(), indices=reduce(operator.add, (list(arm.GetArmIndices()) for arm in arms)))
  @staticmethod
  def robot_arm_and_base(*arms):
    assert len(arms) != 0
    return CSpace(arms[0].GetRobot(), indices=reduce(operator.add, (list(arm.GetArmIndices()) for arm in arms)),
      affine_mask=DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis, rotation_axis=[0, 0, 1])
  @staticmethod
  def robot_body(body):
    return CSpace(body, indices=range(body.GetDOF()))
  # noinspection PyUnresolvedReferences
  @staticmethod
  def robot_body_and_base(body):
    # noinspection PyUnresolvedReferences
    return CSpace(body, indices=range(body.GetDOF()), affine_mask=DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis, rotation_axis=[0, 0, 1])
  def __str__(self):
    return self.__class__.__name__ + '(' + self.body.GetName() + ', %d)'%self.num_dofs()
  __repr__ = __str__


#################################################################

def get_affine_dofs(affine_mask):
  return [RaveGetAffineDOFFromIndex(affine_mask, i) for i in range(RaveGetAffineDOF(affine_mask))]

def get_affine_index(affine_mask, affine_dof):
  return RaveGetIndexFromAffineDOF(affine_mask, affine_dof)

def get_affine_values(trans, affine_mask, rotation_axis=None):
  if rotation_axis is None:
    return RaveGetAffineDOFValuesFromTransform(trans, affine_mask)
  return RaveGetAffineDOFValuesFromTransform(trans, affine_mask, rotation_axis)

def set_active_dofs(body, indices=[], affine_mask=None, rotation_axis=None):
  if affine_mask is None:
    body.SetActiveDOFs(indices)
  elif rotation_axis is None:
    body.SetActiveDOFs(indices, affine_mask)
  else:
    body.SetActiveDOFs(indices, affine_mask, rotation_axis)

def within_limits(body, config):
  return all(body.GetActiveDOFLimits()[0] <= config) and all(config <= body.GetActiveDOFLimits()[1])

#################################################################

def cspace_sample(body):
  return np.random.uniform(*body.GetActiveDOFLimits()) # TODO - adjust theta limits to be between [-PI, PI)

def cspace_sample_collisions(body):
  while True:
    config = cspace_sample(body)
    body.SetActiveDOFValues(config)
    if not get_env().CheckCollision(body): # NOTE - not thread-safe get rid of
      return config

def cspace_collision_free_sample(env, body, self_collisions=True):
  while True:
    config = cspace_sample(body)
    body.SetActiveDOFValues(config)
    if not env.CheckCollision(body) and not (self_collisions and body.CheckSelfCollision()):
      return config

def cspace_partial_sample(body, q, indices):
  sample = np.random.uniform(*body.GetActiveDOFLimits())
  sample[indices] = q
  return sample

def cspace_distance_2(body, q1, q2):
  diff = body.SubtractActiveDOFValues(q2, q1)
  return np.dot(body.GetActiveDOFWeights(), diff*diff)

def cspace_distance(body, q1, q2):
  return sqrt(cspace_distance_2(body, q1, q2))

def cspace_l1_distance(body, q1, q2):
  return np.dot(body.GetActiveDOFWeights(), np.abs(body.SubtractActiveDOFValues(q2, q1)))

def linear_interpolation(body, q1, q2): # Sequence doesn't include q1
  n = int(np.max(np.abs(np.divide(body.SubtractActiveDOFValues(q2, q1), body.GetActiveDOFResolutions())))) + 1
  q = q1
  for i in range(n):
    q = (1./(n-i))*body.SubtractActiveDOFValues(q2, q) + q # NOTE - do I need to repeatedly do the subtract?
    yield q

def sequential_linear_interpolation(body, q1, q2, limit): # Sequence doesn't include q1
  n = int(cspace_distance(body, q1, q2)/limit) + 1
  current_q = q1
  for i in range(n):
    next_q = (1./(n-i))*body.SubtractActiveDOFValues(q2, current_q) + current_q
    yield list(linear_interpolation(body, current_q, next_q))
    current_q = next_q

def limited_linear_interpolation(body, q1, q2, limit=INF): # Sequence doesn't include q1
  direction = body.SubtractActiveDOFValues(q2, q1)
  distance = cspace_distance(body, q1, q2)
  if distance > limit:
    q2 = limit/distance*direction + q1
  return linear_interpolation(body, q1, q2)

def single_step(body, q1, q2):
  n = int(np.max(np.abs(np.divide(body.SubtractActiveDOFValues(q2, q1), body.GetActiveDOFResolutions())))) + 1
  yield (1./n)*body.SubtractActiveDOFValues(q2, q1)

# Resolution should just determine when to check collisions, not the trajectory right?
def gradient_interpolation(body, q1, q2, t=1.): # Sequence doesn't include q1
  #n = int(np.max(np.abs(np.divide(body.SubtractActiveDOFValues(q2, q1), t*body.GetActiveDOFWeights())))) + 1
  q = q1
  #for i in range(n):
  while not np.allclose(q, q2):
    q += t*body.GetActiveDOFWeights()*body.SubtractActiveDOFValues(q2, q)
    yield q

def cspace_neighbors(body, q, goal=None, combine=False):
  if goal is not None:
    diff = body.SubtractActiveDOFValues(goal, q)
    if all(abs(diff[i]) <= r for i, r in enumerate(body.GetActiveDOFResolutions())): # TODO - be too large of a movement
      yield goal

  if combine: # TODO - combine only parts of these
    for direction in randomize(list(product([-1, 0, 1], repeat=body.GetActiveDOF()))):
      if not all(d == 0 for d in direction):
        config = q + np.array(direction)*body.GetActiveDOFResolutions()
        if within_limits(body, config):
          yield config
  else:
    for index in range(body.GetActiveDOF()):
      for sign in [-1, 1]:
        config = q.copy()
        #direction = np.zeros(body.GetActiveDOF())
        #direction[index] += sign*body.GetActiveDOFResolutions()[index]
        #config[index] += direction
        config[index] += sign*body.GetActiveDOFResolutions()[index]
        if within_limits(body, config):
          yield config
  #diff = body.SubtractActiveDOFValues(q2, q1) # TODO - to handle wrap around

# def linear_interpolation(body, q1, q2): # Sequence doesn't include q1
#   n = int(np.max(np.abs(np.divide(body.SubtractActiveDOFValues(q2, q1), body.GetActiveDOFResolutions())))) + 1
#   sequence = [q1]
#   for i in range(n):
#     sequence.append((1./(n-i))*body.SubtractActiveDOFValues(q2, sequence[-1]) + sequence[-1])
#   return sequence[1:]
