from manipulation.collision.collision_primitives import is_valid_point
from manipulation.inverse_reachability.inverse_reachability import *
from manipulation.primitives.placements import *
from motion.single_query import *
from openravepy import planning_error
from primitives.contacts import get_contacts, manip_trans_from_pose_contact, approach_vector_from_pose_contact
from misc.functions import irange
from manipulation.primitives.utils import Pose
from manipulation.primitives.transforms import normalize, length, quat_from_pose, pose_from_quat_point, \
  quat_transform_point, point_from_pose, quat_inv
from pick_and_place import query_paps

# TODO - make push basis given available pushes and plan using that (ie push x, then y, etc)
# TODO - check that the trajectory actually stays on the table
def linear_push_traj(oracle, object_name, initial_pose, goal_point, max_failures=LINEAR_PUSH_MAX_FAILURES):
  initial_quat = quat_from_pose(initial_pose.value)
  initial_point = point_from_pose(initial_pose.value)
  total_distance = length(goal_point.value-initial_point)
  steps = int(ceil(total_distance/PUSH_MAX_DISTANCE)+1)
  distances = np.linspace(0., total_distance, steps)
  direction = normalize(goal_point.value-initial_point)
  poses = [initial_pose] + [Pose(pose_from_quat_point(initial_quat, initial_point+d*direction)) for d in distances[1:]]
  contacts = get_contacts(oracle, object_name, quat_transform_point(quat_inv(initial_quat), direction))
  pushes = []
  for start_pose, end_pose in pairs(poses):
    failures = count()
    cycled_contacts = cycle(randomize(contacts))
    while next(failures) < max_failures:
      contact = next(cycled_contacts)
      push = Push(oracle.get_geom_hash(object_name), start_pose, end_pose, contact)
      if push.sample(oracle, object_name):
        pushes.append(push)
        break
    else:
      return None
  return pushes

"""
def get_symmetrical_push_trajs(oracle, body_name, push_traj):
  symmetrical_push_trajs = []
  # Assuming can push in any direction, can just rotate the contacts and objects in place
  for grasp in get_grasps(oracle, body_name):
    if grasp == pap.grasp: continue
    trans = np.dot(np.linalg.inv(pap.grasp.grasp_trans), grasp.grasp_trans) # M*G = P | M = P*G^-1 | P*(G^-1*G') = P'
    if np.allclose(point_from_trans(trans), unit_point()): # NOTE - signifies only rotation of the object
      pose = Pose(pose_from_trans(np.dot(trans_from_pose(pap.pose.value), trans)))
      new_pap = copy.copy(pap)
      new_pap.pose = pose
      new_pap.grasp = grasp
      symmetrical_paps.append(new_pap)
  return symmetrical_push_trajs
"""

#################################################################

def sample_approach_config(oracle, base_trans, check_base=CHECK_BASE_REACHABLE):
  if not is_valid_point(oracle.robot, point_from_trans(base_trans)): return None
  set_trans(oracle.robot, base_trans)
  set_config(oracle.robot, oracle.default_left_arm_config, get_arm_indices(oracle))
  if robot_collision(oracle, check_self=False): return None
  approach_config = oracle.get_robot_config()
  if check_base and not oracle.grow_base_roadmap(approach_config): return None
  return approach_config

def sample_contact_config(oracle, manip_trans):
  grasp_arm_config = inverse_kinematics(oracle, manip_trans)
  if grasp_arm_config is None: return None
  set_config(oracle.robot, grasp_arm_config, get_arm_indices(oracle))
  return oracle.get_robot_config()

def sample_vector_traj(oracle, approach_vector):
  traj = vector_traj(oracle, approach_vector)
  if traj is None: return None
  set_config(oracle.robot, traj.end(), get_arm_indices(oracle))
  return traj, oracle.get_robot_config()

def sample_arm_traj(oracle, approach_config):
  #return manip_traj(oracle, approach_config.value[get_arm_indices(oracle)])
  return motion_plan(oracle.env, CSpace.robot_arm(get_manipulator(oracle.robot)), approach_config.value[get_arm_indices(oracle)])

class Push(object):
  def __init__(self, geom_hash, start_pose, end_pose, contact):
    self.geom_hash = geom_hash
    self.start_pose = start_pose
    self.end_pose = end_pose
    self.contact = contact
    self.sampled = False

  def sample_push(self, oracle, object_name):
    self.approach_config = sample_approach_config(oracle, self.base_trans)
    if self.approach_config is None: return False
    oracle.set_pose(object_name, self.start_pose)
    self.start_contact_config = sample_contact_config(oracle, manip_trans_from_pose_contact(self.start_pose, self.contact))
    if self.start_contact_config is None: return False
    grab(oracle, object_name)
    self.push_traj = workspace_traj(oracle, point_from_pose(self.end_pose.value)-point_from_pose(self.start_pose.value))
    release(oracle, object_name)
    if self.push_traj is None: return False
    self.end_contact_config = Config(self.start_contact_config.value.copy())
    self.end_contact_config.value[get_arm_indices(oracle)] = self.push_traj.end()
    return True

  def sample_setup(self, oracle, object_name, object_pose, contact_config):
    oracle.set_pose(object_name, object_pose)
    oracle.set_robot_config(contact_config)
    result = sample_vector_traj(oracle, approach_vector_from_pose_contact(object_pose, self.contact))
    if result is None: return None
    vector_traj, vector_config = result
    arm_traj = sample_arm_traj(oracle, self.approach_config)
    if arm_traj is None: return None
    return vector_config, (vector_traj, arm_traj)

  # TODO - seems like having trouble sampling bases. Make a distribution that is for pushing
  def sample(self, oracle, object_name, max_failures=PUSH_MAX_FAILURES):
    if self.sampled: return
    t0 = time()
    self.sampled = True
    self.successful = False
    with oracle.env:
      with oracle.robot:
        with oracle.body_saver(object_name):
          #mean_pose = pose_interpolate(self.start_pose.value, self.end_pose.value)
          base_iterator = custom_base_iterator(oracle, [(manip_trans_from_object_trans(
              trans_from_pose(self.start_pose.value), self.contact.grasp_trans), get_trans(oracle.robot), tuple())])
          for _ in irange(max_failures):
            #oracle.set_pose(object_name, None) # Probably don't need this
            try:
              self.base_trans, _ = next(base_iterator)
            except (StopIteration, ValueError, planning_error): # ValueError: low >= high because of an empty sequence
              break
            if not self.sample_push(oracle, object_name): continue
            result = self.sample_setup(oracle, object_name, self.end_pose, self.end_contact_config) # Postpush seems to be harder to sample than prepush
            if result is None: continue
            self.end_vector_config, self.end_trajs = result
            result = self.sample_setup(oracle, object_name, self.start_pose, self.start_contact_config)
            if result is None: continue
            self.start_vector_config, self.start_trajs = result
            self.start_trajs = tuple(reverse_trajectories(self.start_trajs))
            self.successful = True
            break
    self.sample_time = time() - t0
    return self.successful
  def smooth(self, oracle, object_name): # TODO
    pass
  def __repr__(self):
    return self.__class__.__name__ + str_object((self.start_pose, self.end_pose, self.contact))

#################################################################

# TODO - connect to nearest pose that will complete the roadmap
def get_push_trajs(oracle, body_name, initial_pose, goal_point, max_failures=LINEAR_PUSH_MAX_FAILURES):
  geom_hash = oracle.get_geom_hash(body_name)
  goal_pose = first(lambda pose: np.allclose(point_from_pose(pose.value), goal_point.value) and
                    oracle.pushes[geom_hash](initial_pose, pose) is not None, oracle.pushes[geom_hash])
  #print goal_pose, initial_pose not in oracle.pushes[geom_hash], initial_pose
  #print oracle.pushes[geom_hash].vertices.values()
  #print filter(lambda pose: np.allclose(point_from_pose(pose.value), goal_point.value), oracle.pushes[geom_hash])
  if goal_pose is None or initial_pose not in oracle.pushes[geom_hash]: # NOTE - do not need later check if ensuring the traj is good
    with oracle.state_saver():
      oracle.set_all_object_poses({body_name: initial_pose})
      push_traj = linear_push_traj(oracle, body_name, initial_pose, goal_point, max_failures=max_failures)
      if push_traj is None: return []
      for push in push_traj:
        oracle.pushes[geom_hash].connect(push.start_pose, push.end_pose, push)
      goal_pose = push_traj[-1].end_pose
  return [oracle.pushes[geom_hash](initial_pose, goal_pose)[1]]

def query_from_edge_push_trajs(oracle, body_name, region_name, goal_point):
  geom_hash = oracle.get_geom_hash(body_name)
  for pose in filter(lambda p: oracle.on_edge(region_name, body_name, p), # and \ oracle.pushes[geom_hash](p, goal_point) is not None, # NOTE - goal_point not a valid arg!!!!
                     oracle.pushes[oracle.get_geom_hash(body_name)]):
    pap, _ = next(query_paps(oracle, body_name, poses=[pose], num_grasps=4, max_failures=15), (None, None))
    if pap is None: continue
    push_trajs = get_push_trajs(oracle, body_name, pose, goal_point)
    if len(push_trajs) == 0: continue
    for push_traj in push_trajs:
      yield pap, push_traj

  for pose in random_edge_placements(oracle, body_name, region_name, z=goal_point.value[2], bias_point=goal_point.value):
    pap, _ = next(query_paps(oracle, body_name, poses=[pose], num_grasps=4, max_failures=15), (None, None))
    if pap is None: continue
    #for new_pap in (get_symmetrical_paps(oracle, body_name, pap) if SYMMETRICAL_PAPS else []):
    #  oracle.add_pap(body_name, new_pap)
    #  print new_pap
    push_trajs = get_push_trajs(oracle, body_name, pose, goal_point)
    if len(push_trajs) == 0: continue
    for push_traj in push_trajs:
      yield pap, push_traj

def query_to_edge_push_trajs(oracle, body_name, region_name, start_pose, grasps):
  geom_hash = oracle.get_geom_hash(body_name)
  start_point = point_from_pose(start_pose.value)
  for pose in filter(lambda p: oracle.on_edge(region_name, body_name, p) and \
                     oracle.pushes[geom_hash](start_pose, p) is not None,
                     oracle.pushes[oracle.get_geom_hash(body_name)]):
    pap, _ = next(query_paps(oracle, body_name, poses=[pose], grasps=grasps, max_failures=15), (None, None))
    if pap is None: continue
    push_trajs = get_push_trajs(oracle, body_name, start_pose, Point(point_from_pose(pose.value)))
    if len(push_trajs) == 0: continue
    for push_traj in push_trajs:
      yield pap, push_traj

  for pose in random_edge_placements(oracle, body_name, region_name, z=start_point[2], use_quat=quat_from_pose(start_pose.value), bias_point=start_point):
    pap, _ = next(query_paps(oracle, body_name, poses=[pose], grasps=grasps, max_failures=15), (None, None))
    if pap is None: continue
    push_trajs = get_push_trajs(oracle, body_name, start_pose, Point(point_from_pose(pose.value)))
    if len(push_trajs) == 0: continue
    for push_traj in push_trajs:
      yield pap, push_traj
