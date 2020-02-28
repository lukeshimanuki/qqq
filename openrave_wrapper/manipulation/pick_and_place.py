import copy

from manipulation.collision.collision_primitives import is_valid_point
from manipulation.grasps.grasps import *
from manipulation.inverse_reachability.inverse_reachability import *
from manipulation.primitives.placements import *
from motion.single_query import *
from time import time

def get_symmetrical_paps(oracle, body_name, pap):
  symmetrical_paps = []
  for grasp in get_grasps(oracle, body_name):
    if grasp == pap.grasp: continue
    trans = np.dot(np.linalg.inv(pap.grasp.grasp_trans), grasp.grasp_trans) # M*G = P | M = P*G^-1 | P*(G^-1*G') = P'
    if np.allclose(point_from_trans(trans), unit_point()): # NOTE - signifies only rotation of the object
      pose = Pose(pose_from_trans(np.dot(trans_from_pose(pap.pose.value), trans)))
      new_pap = copy.copy(pap)
      new_pap.pose = pose
      new_pap.grasp = grasp
      symmetrical_paps.append(new_pap)
  return symmetrical_paps

def current_base_iterator(oracle, manip_iterator):
  current_trans = get_trans(oracle.robot)
  for manip_trans in manip_iterator:
    yield current_trans, manip_trans

def sample_approach_config(oracle, base_trans):
  if not is_valid_point(oracle.robot, point_from_trans(base_trans)):
    return None
  manip_name = oracle.robot.GetActiveManipulator().GetName()
  default_config = oracle.default_left_arm_config if manip_name == 'leftarm' else oracle.default_right_arm_config
  set_config(oracle.robot, default_config, get_arm_indices(oracle))
  set_trans(oracle.robot, base_trans)
  if robot_collision(oracle, check_self=False):
    return None
  return oracle.get_robot_config()

def sample_grasp_config(oracle, manip_trans):
  grasp_arm_config = inverse_kinematics(oracle, manip_trans)
  if grasp_arm_config is None:
    return None
  set_config(oracle.robot, grasp_arm_config, get_arm_indices(oracle))
  return oracle.get_robot_config()

def sample_vector_traj(oracle, approach_vector):
  return vector_traj(oracle, approach_vector)

def get_vector_config(oracle, vec_traj):
  set_config(oracle.robot, vec_traj.end(), get_arm_indices(oracle))
  return oracle.get_robot_config()

def sample_arm_traj(oracle, approach_config):
  goal_arm_config = approach_config.value[get_arm_indices(oracle)]
  if OPENRAVE_ARM_TRAJ:
    arm_traj = manip_traj(oracle, goal_arm_config)
  else:
    arm_traj = motion_plan(oracle.env, CSpace.robot_arm(get_manipulator(oracle.robot)), goal_arm_config)
  if arm_traj is None: return False
  self.trajs.append(arm_traj)
  self.trajs = tuple(self.trajs)
  return True

class PickAndPlace(object):
  def __init__(self, geom_hash, pose, grasp):
    self.geom_hash = geom_hash
    self.pose = pose
    self.grasp = grasp
    self.sampled = False
    self.successful = False
    self.iterations = Counter()

  def ir_base_vector(self): # TODO - infer the base vector
    raise NotImplemented()

  def sample_approach_config(self, oracle):
    self.approach_config = sample_approach_config(oracle, self.base_trans)
    return self.approach_config is not None

  def sample_grasp_config(self, oracle):
    self.grasp_config = sample_grasp_config(oracle, self.manip_trans)
    return self.grasp_config is not None

  def sample_vector_traj(self, oracle):
    vec_traj = sample_vector_traj(oracle, self.approach_vector)
    if vec_traj is None:
      return False
    self.vector_config = get_vector_config(oracle, vec_traj)
    #if isinstance(self.trajs, tuple):
    #  print self, self.sampled, self.successful, self.trajs, self.iterations
    self.trajs.append(vec_traj)
    return True

  def sample_arm_traj(self, oracle):
    goal_arm_config = self.approach_config.value[get_arm_indices(oracle)]
    if OPENRAVE_ARM_TRAJ:
      arm_traj = manip_traj(oracle, goal_arm_config)
    else:
      arm_traj = motion_plan(oracle.env, CSpace.robot_arm(get_manipulator(oracle.robot)), goal_arm_config)
    if arm_traj is None: return False
    self.trajs.append(arm_traj)
    self.trajs = tuple(self.trajs)
    return True

  def sample(self, oracle, object_name, base_iterator_fn=custom_base_iterator, max_failures=INF,
      sample_vector=SAMPLE_VECTOR_TRAJ, sample_arm=SAMPLE_ARM_TRAJ, check_base=CHECK_BASE_REACHABLE):
    if self.sampled: return self.successful
    t0 = time()
    self.sampled = True
    self.manip_trans, self.approach_vector = manip_from_pose_grasp(self.pose, self.grasp)
    self.trajs = []
    with oracle.env:
      with oracle.robot:
        with oracle.body_saver(object_name):
          oracle.set_pose(object_name, object_pose_from_current_config(oracle, self.grasp))
          grab(oracle, object_name) # TODO - only grab when motion planning if this is slow
          base_iterator = base_iterator_fn(oracle, [self.manip_trans])
          while int(self.iterations) < max_failures:
            next(self.iterations)
            try:
              self.base_trans, _ = next(base_iterator)
              #print oracle.ikmodels['rightarm'].manip.FindIKSolution(self.manip_trans, 0)
            except StopIteration:
              break
            if self.sample_approach_config(oracle) and \
                self.sample_grasp_config(oracle) and \
                (not check_base or oracle.grow_base_roadmap(self.approach_config)) and \
                (not sample_vector or self.sample_vector_traj(oracle)) and \
                (not sample_arm or self.sample_arm_traj(oracle)): # NOTE - can put check_base before grasp_config
              self.successful = True
              break
            else:
              self.approach_config = None # NOTE - this was the bug where it made trajectories that weren't reachable
              self.grasp_config = None
              self.trajs = []
          release(oracle, object_name) # TODO - grow_base_roadmap may still have the attachment
    self.sample_time = time() - t0
    return self.successful

  def smooth(self, oracle, object_name): # TODO
    trajs = list(self.trajs)
    with GrabSaver(oracle.robot):
      release_all(oracle)
      with oracle.body_saver(object_name):
        with oracle.robot:
          oracle.set_robot_config(self.approach_config)
          while True:
            oracle.set_pose(object_name, object_pose_from_current_config(oracle, self.grasp))
            grab(oracle, object_name)
            if REPLAN_TRAJECTORIES:
              trajs[-1] = trajs[-1].replan(restarts=INF, iterations=50, smooth=50)
            else:
              trajs[-1] = trajs[-1].resample().shortcut()

            release(oracle, object_name)
            oracle.set_pose(object_name, self.pose)
            is_colliding = False
            trajs[-1].cspace.set_active()
            for config in trajs[-1].path():
              oracle.robot.SetActiveDOFValues(config)
              if collision(oracle, get_name(oracle.robot), object_name):
                is_colliding = True
                print 'Colliding pap trajectory'
                break
            if not is_colliding: break
    self.trajs = tuple(trajs)

  def convert_configs(self, oracle):
    configs = ['approach_config', 'grasp_config', 'vector_config']
    for config in configs:
      if hasattr(self, config):
        setattr(self, config, extract_arm_and_base_config(oracle, getattr(self, config)))
  def __repr__(self):
    if hasattr(self, 'approach_config'):
      return self.__class__.__name__ + str_object((self.pose, self.grasp, self.approach_config))
    return self.__class__.__name__ + str_object((self.pose, self.grasp))

#################################################################

def is_relevant(pap, oracle, body_name, poses, num_poses, grasps, num_grasps, regions, stackings):
  return (num_poses is not None or pap.pose in poses) and \
        (num_grasps is not None or pap.grasp in grasps) and \
        (regions is None or any(oracle.region_contains(region, body_name, pap.pose) for region in regions)) and \
        (stackings is None or any(oracle.are_stacked(body_name, pap.pose, base_name, base_pose) for base_name, base_pose in stackings))

def is_priority(pap, poses, grasps):
  return pap.pose in poses or pap.grasp in grasps

def existing_paps(oracle, body_name, poses=[], num_poses=None, grasps=[], num_grasps=None, regions=None, stackings=None,
    object_poses={}, pose_sort_fn=None):
  priority, relevant = [], []
  for pap in oracle.get_paps(body_name):
    if is_relevant(pap, oracle, body_name, poses, num_poses, grasps, num_grasps, regions, stackings):
      if is_priority(pap, poses, grasps):
        priority.append(pap)
      else:
        relevant.append(pap)

  sort_fn = (lambda pap: pose_sort_fn(pap.pose)) if pose_sort_fn is not None else None
  def generator(paps):
    paps = randomize(paps)
    if sort_fn is not None: paps = sorted(paps, key=sort_fn)
    for pap in paps:
      if (ONLY_HOLDING_COLLISIONS and not any(oracle.holding_collision(pap.grasp_config,
            o, p, Holding(body_name, pap.grasp)) for o, p in object_poses.iteritems())) or \
         (not ONLY_HOLDING_COLLISIONS and not any(oracle.traj_holding_collision(pap.approach_config,
          pap.trajs, o, p, Holding(body_name, pap.grasp)) for o, p in object_poses.iteritems())):
        yield pap, False # TODO - return all of these at once!!!

  yield generator(priority)
  yield generator(relevant)

# TODO - return the stacking and/or region it satisfies!!!!
# NOTE - cannot have infinite grasps and poses at the same time
def generate_paps(oracle, body_name, poses=[], num_poses=None, grasps=[], num_grasps=None, regions=None, stackings=None,
    object_poses={}, pose_sort_fn=None, max_failures=PAP_MAX_FAILURES):
  geom_hash = oracle.get_geom_hash(body_name)
  poses = sorted(filter(lambda p: not any(oracle.object_collision(body_name, p, o, p)
      for o, p in object_poses.iteritems()), poses), key=pose_sort_fn)

  with oracle.state_saver():
    oracle.set_all_object_poses(merge_dicts(object_poses, {body_name: oracle.get_pose(body_name)}))
    if stackings is None: # TODO - are stackings and regions mutually exclusive?
      sample_regions = regions if regions is not None else oracle.get_counters()
      #print sample_regions, num_poses, len(object_poses)
      #new_poses = sorted(list(take(random_region_placements(oracle, body_name, sample_regions), num_poses)), key=pose_sort_fn)
      #new_poses = take(random_region_placements(oracle, body_name, sample_regions), num_poses) # For possibly infinite
      order = None # lambda p: randomize(p) # NOTE - already in cached_region_placements
      if pose_sort_fn is not None:
        order = lambda p: sorted(p, key=pose_sort_fn) # TODO - chain randomize first part and sort
      new_poses = take(cached_region_placements(oracle, body_name, sample_regions, order=order), num_poses) # TODO - chain randomize first part and sort
    else:
      new_poses = take(center_stackings(oracle, body_name, stackings), num_poses)
  new_grasps = list(take(get_grasps(oracle, body_name), num_grasps))

  def generator(paps):
    for pap in paps:
      if (oracle.approach_collision(body_name, pap.pose, pap.grasp) or (not ONLY_HOLDING_COLLISIONS and \
          any(oracle.obj_approach_collision(body_name, pap.pose, pap.grasp, o, p) for o, p in object_poses.iteritems()))): continue
      with oracle.state_saver():
        oracle.set_all_object_poses(merge_dicts(object_poses, {body_name: pap.pose}))
        pap.sample(oracle, body_name, max_failures=max_failures)
      if pap.successful:
        oracle.add_pap(body_name, pap)
        symmetrical_paps = get_symmetrical_paps(oracle, body_name, pap) if SYMMETRICAL_PAPS else []
        for new_pap in symmetrical_paps:
          oracle.add_pap(body_name, new_pap)
        yield pap, True
        for new_pap in symmetrical_paps:
          if is_relevant(new_pap, oracle, body_name, poses, num_poses, grasps, num_grasps, regions, stackings): # TODO - check is priority?
            yield pap, True

  yield generator(chain((PickAndPlace(geom_hash, pose, grasp) for pose in poses for grasp in grasps),
              (PickAndPlace(geom_hash, pose, grasp) for pose in poses for grasp in new_grasps),
              (PickAndPlace(geom_hash, pose, grasp) for pose in new_poses for grasp in grasps) if len(grasps) != 0 else [])) # NOTE - prevents new_poses from being exhausted if infinite
  yield generator(PickAndPlace(geom_hash, pose, grasp) for pose in new_poses for grasp in new_grasps)

# TODO - will not get newly added manip_tuples, need to add asynchronously added
# NOTE - okay to have repeated copies of PAPs, so multiple attempts to sample

def query_paps(oracle, body_name, poses=[], num_poses=None, grasps=[], num_grasps=None, regions=None, stackings=None,
    object_poses={}, pose_sort_fn=None, max_failures=PAP_MAX_FAILURES):
  generators = deque([ # TODO - change this because barely use the priority function
    existing_paps(oracle, body_name, poses, num_poses, grasps, num_grasps, regions, stackings, object_poses, pose_sort_fn),
    generate_paps(oracle, body_name, poses, num_poses, grasps, num_grasps, regions, stackings, object_poses, pose_sort_fn, max_failures)
  ])
  while len(generators) != 0:
    generator = generators.popleft()
    results = next(generator, None)
    if results is None: continue
    generators.append(generator)
    for result in results:
      yield result
