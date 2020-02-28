from motion.primitives import *
from manipulation.primitives.placements import *
from manipulation.primitives.savers import *
from manipulation.inverse_reachability.inverse_reachability import load_ir_database
from manipulation.grasps.grasps import get_grasps
from manipulation.regions import *
from manipulation.motion.trajectories import *
from manipulation.primitives.display import *
from motion_planners.graph import Graph
from manipulation.bodies.bounding_volumes import *
from manipulation.collision.edge_collision import EdgeCollisionCache, EdgeHoldingCollisionCache
from motion_planners.multi_rrt import MultiBiRRT
from constants import *

class ManipulationOracle(object):
  def __init__(self, problem, env, preload_databases=PRELOAD_DATABASES, debug=DEBUG, active_arms=None, reset_robot=True):
    self.problem = problem
    self.env = env

    #if problem.rearrangement:
    #  self.max_grasps = 1
    #else:
    #  self.max_grasps = 4

    if debug: print SEPARATOR

    self.setup_env()
    self.setup_robot(debug, active_arms, reset_robot)
    self.setup_bodies()
    self.setup_regions()
    self.setup_bounding_volumes()
    self.setup_caches()
    self.setup_primitives()

    if preload_databases:
      load_ir_database(self)
      if problem.grasps is None:
        for body_name in self.get_objects():
          get_grasps(self, body_name)
      else:
        self.grasp_database = {}
        for obj_name in problem.grasps:
          grasp_key = (self.get_geom_hash(obj_name), USE_GRASP_APPROACH, USE_GRASP_TYPE)
          self.grasp_database[grasp_key] = problem.grasps[obj_name]

    if debug: print SEPARATOR, '\n', self

    ###############

    # NOTE - can use links similarly to KinBodies. Changing the link transform doesn't affect full body, but it remains relatively attached
    self.base = self.robot.GetLink('base_link')
    self.torso = self.robot.GetLink('torso_lift_link')

    #self.necessary_collision = lambda b1, p1, b2, p2: self.aabb_collision(b1, trans_from_pose(p1.value), b2, trans_from_pose(p2.value))
    self.necessary_collision = lambda b1, p1, b2, p2: self.sphere_collision(b1, point_from_pose(p1.value), b2, point_from_pose(p2.value))
    #self.necessary_collision = lambda b1, p1, b2, p2: self.cylinder_collision(b1, point_from_pose(p1.value), b2, point_from_pose(p2.value))

    #self.necessary_collision_current = lambda b1, b2: self.aabb_collision(b1, get_trans(self.get_body(b1)), b2, get_trans(self.get_body(b2)))
    #self.necessary_collision_current = lambda b1, b2: self.sphere_collision(b1, get_point(self.get_body(b1)), b2, get_point(self.get_body(b2)))
    #self.necessary_collision_current = lambda b1, b2: self.cylinder_collision(b1, get_point(self.get_body(b1)), b2, get_point(self.get_body(b2)))

  #################################################################

  def setup_env(self):
    self.env.StopSimulation()
    for sensor in self.env.GetSensors():
      sensor.Configure(Sensor.ConfigureCommand.PowerOff)
      sensor.Configure(Sensor.ConfigureCommand.RenderDataOff)
      sensor.Configure(Sensor.ConfigureCommand.RenderGeometryOff)
      self.env.Remove(sensor)
    self.env.SetCollisionChecker(RaveCreateCollisionChecker(self.env, COLLISION_CHECKER))
    self.env.GetCollisionChecker().SetCollisionOptions(0)
    self.locked = False

  def setup_robot(self, debug, active_arms=None, reset=True):
    self.robot = self.env.GetRobots()[0] # TODO - multiple robots and store names instead of bodies
    self.robot_name = get_name(self.robot)
    self.cd_model = databases.convexdecomposition.ConvexDecompositionModel(self.robot)
    if not self.cd_model.load():
      print 'Creating convex decomposition'
      self.cd_model.autogenerate()
    if debug: print 'Loaded convex decomposition'
    self.l_model = databases.linkstatistics.LinkStatisticsModel(self.robot)
    if not self.l_model.load():
      print 'Creating link statistics'
      self.l_model.autogenerate()
    if debug: print 'Loaded link statistics'
    self.l_model.setRobotWeights()
    self.l_model.setRobotResolutions(xyzdelta=MIN_DELTA) # xyzdelta is the minimum Cartesian distance of an object
    #self.robot.SetDOFResolutions(5*self.robot.GetDOFResolutions())
    #print 'Robot resolutions:', repr(self.robot.GetDOFResolutions()), '| Robot weights:', repr(self.robot.GetDOFWeights())

    self.active_manipulators = [] #['leftarm', 'rightarm'] # []
    if active_arms is None:
      if ACTIVE_LEFT:
        self.active_manipulators.append('leftarm')
      if ACTIVE_RIGHT:
        self.active_manipulators.append('rightarm')
    else:
      for arm in active_arms:
        if arm in ['left', 'right']:
          self.active_manipulators.append(arm + 'arm')
        else:
          raise ValueError(arm)

    if ACTIVE_TORSO:
      self.active_manipulators = [name + '_torso' for name in self.active_manipulators]
    #self.active_manipulators = ['leftarm', 'rightarm'] # [] # TODO - only the first manipulator works for IK...
    assert self.active_manipulators

    robot_radius = get_radius2D(self.robot)
    self.default_left_arm_config = HOLDING_LEFT_ARM if 'leftarm' in self.active_manipulators else REST_LEFT_ARM
    self.default_right_arm_config = mirror_arm_config(HOLDING_LEFT_ARM if 'rightarm' in self.active_manipulators else REST_LEFT_ARM)
    if reset:
      self.robot.SetDOFValues(self.default_left_arm_config, self.robot.GetManipulator('leftarm').GetArmIndices())
      self.robot.SetDOFValues(self.default_right_arm_config, self.robot.GetManipulator('rightarm').GetArmIndices())
      self.robot.SetDOFValues([.15], [self.robot.GetJointIndex('torso_lift_joint')])
    for name in self.active_manipulators:
      self.robot.SetActiveManipulator(name)
      open_gripper(self)
    self.initial_config = self.get_robot_config() # TODO - the right arm has a strange wrist position

    # TODO - depdenent on the current robot config
    self.robot_links = {get_name(link): link for link in self.robot.GetLinks() if has_geometry(link)}
    #self.robot_link_radii = {name: get_radius(link)**2 for name, link in self.robot_links.iteritems()}
    self.robot_link_meshes = {name: link_mesh(self, link) for name, link in self.robot_links.iteritems()}
    self.robot_link_convex_meshes = {name: convex_mesh(mesh_vertices(self.robot_link_meshes[name])) for name in self.robot_links}
    #self.robot_link_oobbs = {name: oobb_from_points(mesh_vertices(self.robot_link_convex_meshes[name])) for name in self.robot_links}

    self.robot_parts = {
      'left_hand': ['l_wrist_roll_link', 'l_gripper_r_finger_link', 'l_gripper_l_finger_tip_link',
          'l_gripper_r_finger_tip_link', 'l_gripper_palm_link',
          'l_gripper_l_finger_link'], #[get_name(link) for link in get_hand_links(self.robot.GetManipulator('leftarm')) if has_geometry(link)],
      'left_forearm': ['l_forearm_link', 'l_wrist_flex_link', 'l_elbow_flex_link'],
      'left_upper_arm': ['l_upper_arm_link', 'l_upper_arm_roll_link'],
      'left_shoulder': ['l_shoulder_lift_link', 'l_shoulder_pan_link'],

      'right_hand': ['r_gripper_palm_link', 'r_gripper_l_finger_tip_link', 'r_gripper_r_finger_link',
          'r_wrist_roll_link', 'r_gripper_l_finger_link',
          'r_gripper_r_finger_tip_link'], #[get_name(link) for link in get_hand_links(self.robot.GetManipulator('rightarm')) if has_geometry(link)],
      'right_forearm': ['r_forearm_link', 'r_wrist_flex_link', 'r_elbow_flex_link'],
      'right_upper_arm': ['r_upper_arm_link', 'r_upper_arm_roll_link'],
      'right_shoulder': ['r_shoulder_lift_link', 'r_shoulder_pan_link'],

      'body': ['base_link', 'br_caster_r_wheel_link', 'br_caster_rotation_link', 'br_caster_l_wheel_link',
          'bl_caster_r_wheel_link', 'bl_caster_rotation_link', 'head_tilt_link', 'fl_caster_r_wheel_link',
          'fr_caster_rotation_link', 'fr_caster_l_wheel_link', 'bl_caster_l_wheel_link', 'head_pan_link',
          'fr_caster_r_wheel_link', 'torso_lift_link', 'fl_caster_rotation_link', 'fl_caster_l_wheel_link',
          'laser_tilt_mount_link'] #[get_name(link) for link in get_non_manipulator_links([self.robot.GetManipulator(m) for m in ['leftarm', 'rightarm']]) if has_geometry(link)]
    }
    self.robot_parts['not_left'] = list(flatten(self.robot_parts[part]
        for part in ['body', 'right_hand', 'right_forearm', 'right_upper_arm', 'right_shoulder']))
    self.robot_parts['not_right'] = list(flatten(self.robot_parts[part]
        for part in ['body', 'left_hand', 'left_forearm', 'left_upper_arm', 'left_shoulder']))

    self.robot_meshes = {}
    self.robot_convex_meshes = {}
    for name, link_names in self.robot_parts.iteritems():
      reference_link = link_names[0]
      refrence_trans = np.linalg.inv(get_trans(self.robot_links[reference_link]))
      self.robot_meshes[name] = merge_meshes([mesh_apply_trans(self.robot_link_meshes[link_name],
          np.dot(refrence_trans, get_trans(self.robot_links[link_name]))) for link_name in link_names])
    self.robot_convex_meshes = {name: convex_mesh(mesh_vertices(self.robot_meshes[name])) for name in self.robot_parts}

    extrema = aabb_extrema(aabb_union([aabb_from_body(body) for body in self.env.GetBodies()])).T
    if not any('floor' in get_name(body) for body in self.env.GetBodies()):
      extrema += WORKSPACE_ROBOT_LENGTHS*robot_radius*np.array([[-1, -1, 0], [1, 1, 0]])
    self.env_min, self.env_max = extrema
    self.robot.SetAffineTranslationLimits(self.env_min, self.env_max)
    #self.robot.SetAffineRotationAxisLimits(-np.array([PI, PI, PI]), np.array([PI, PI, PI])) # NOTE - doesn't change SubtractDOFValues or SetActiveDOFValues
    #self.robot.SetAffineRotation3DLimits(np.array([0, 0, 0]), np.array([2*PI, 2*PI, 2*PI]))

    # http://openrave.org/docs/0.8.2/openravepy/examples.tutorial_iklookat_multiple/
    self.robot.SetActiveManipulator(self.robot.GetManipulator('head'))
    self.look_model = databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=IkParameterization.Type.Lookat3D,freeindices=None)
    if not self.look_model.load():
      print 'Creating openrave look model'
      self.look_model.autogenerate()

    #from manipulation.primitives.look import look_at, look_at_ik
    #print self.env.GetBodies()
    #print look_at_ik(self, self.env.GetKinBody('green_box')) # blue_box | green_box
    #print look_at(self.robot, self.env.GetKinBody('green_box')) # blue_box | green_box
    #raw_input('Pause')
    # NOTE - maybe the manipulator needs to be active to do this

    self.ikmodels = {}
    for name in self.active_manipulators: # TODO - I could just load for all manipulators
      self.robot.SetActiveManipulator(name)
      self.ikmodels[name] = databases.inversekinematics.InverseKinematicsModel(iktype=IkParameterization.Type.Transform6D,
          forceikfast=True, freeindices=None, freejoints=None, manip=self.robot.GetManipulator(name))
      #self.ikmodels[name] = databases.inversekinematics.InverseKinematicsModel(robot=self.robot, iktype=IkParameterization.Type.Transform6D,
      #    forceikfast=True, freeindices=None, freejoints=None, manip=None)
      #self.robot.Enable(True)
      #self.ikmodels[name].setrobot()
      if not self.ikmodels[name].load():
        print 'Creating %s openrave inverse kinematics model'%name
        self.ikmodels[name].autogenerate()
        if debug: print 'Saved %s openrave inverse kinematics model'%name
      else:
        if debug: print 'Loaded %s openrave inverse kinematics model'%name
     # if name == 'leftarm':
     #   #self.ikmodels[name].__del__()
     #   self.env.Remove(self.ikmodels[name].ikfastproblem)
    self.robot.SetActiveManipulator(self.active_manipulators[0]) # TODO - IK only works for the first manipulator

    self.base_manip = interfaces.BaseManipulation(self.robot, plannername=None, maxvelmult=None)
    self.task_manip = interfaces.TaskManipulation(self.robot, plannername=None, maxvelmult=None, graspername=None)

  def setup_bodies(self):
    self.bodies = {str(get_name(body)): body for body in self.env.GetBodies()}
    self.active = {body_name for body_name in self.bodies if self.env.GetKinBody(body_name) is not None}
    self.body_name_to_geom_hash = {body_name: geometry_hash(body) for body_name, body in self.bodies.items()}
    self.geom_hash_to_body_name = {geometry_hash(body): body_name for body_name, body in self.bodies.items()}
    self.geom_hash_to_body_names = defaultdict(list)
    for body_name, body in self.bodies.items():
      self.geom_hash_to_body_names[geometry_hash(body)].append(body_name)

    self.objects = self.problem.object_names
    self.floor_objects = self.problem.floor_object_names
    self.obstacles = [name for name, body in self.bodies.items() if name not in self.get_objects() and not body.IsRobot()]
    self.initial_poses = {object_name: self.get_pose(object_name) for object_name in self.get_objects()}

  def setup_regions(self):
    floors = [AASurface('floor', zip(self.env_min, self.env_max)[:2], BODY_Z_OFFSET, color=FLOOR_COLOR)] # TODO - is it correct to use env_min/env_max for placements?
    self.floors = [region.name for region in floors]
    self.tables = self.problem.table_names
    self.sinks = self.problem.sink_names
    self.stoves = self.problem.stove_names
    self.goal_regions = [region.name for region in self.problem.regions]
    self.regions = merge_dicts({region.name: region for region in self.problem.regions},
                               {name: AARegion.create_on_body(self.bodies[name], color=TABLE_COLOR) for name in self.tables},
                               {name: AARegion.create_on_body(self.bodies[name], color=SINK_COLOR) for name in self.sinks},
                               {name: AARegion.create_on_body(self.bodies[name], color=STOVE_COLOR) for name in self.stoves},
                               {region.name: region for region in floors})

  def setup_bounding_volumes(self):
    self.radii2D = {name: get_radius2D(body)**2 for name, body in self.bodies.iteritems()}
    self.radii = {name: get_radius(body)**2 for name, body in self.bodies.iteritems()}
    self.aabbs = {}
    self.meshes = {}
    for name, body in self.bodies.iteritems(): # TODO - use geom_hash instead of object names
      with self.body_saver(name): # NOTE - aabb_from_body(body) != aabb_apply_trans(self.aabbs[name], get_trans(body)) is okay because constructed differently
        set_trans(body, unit_trans()) # - doesn't encompass different configurations of bodies
        self.aabbs[name] = aabb_from_body(body)
        self.meshes[name] = mesh_from_body(body)
    self.convex_meshes = {name: convex_mesh(self.get_vertices(name)) for name in self.bodies}
    self.oobbs = {name: oobb_from_points(self.convex_meshes[name].vertices.T) for name in self.bodies} # NOTE - self.get_aabb(name, trans) = aabb_from_body(self.get_body(name))
    # TODO - make these all caches
    self.get_geometries = DeterminisiticCache(lambda name: geometries(self.get_body(name)))
    self.get_geometry_types = DeterminisiticCache(lambda name: geometry_types(self.get_body(name)))

  def setup_caches(self):
    self.object_collision = ObjectCollisionCache(self)
    self.holding_collision = HoldingCollisionCache(self)

    self.traj_collision = TrajCollisionCache(self)
    self.traj_holding_collision = TrajHoldingCollisionCache(self)

    self.edge_collision = EdgeCollisionCache(self)
    self.edge_holding_collision = EdgeHoldingCollisionCache(self)

    self.random_placement_cache = RandomPlacementsCache(self)
    self.grid_placement_cache = GridPlacementsCache(self)

    self.region_contains = RegionContainsCache(self)
    self.are_stacked = AreStackedCache(self)
    self.on_edge = OnEdgeCache(self)
    #self.get_aabb = GetAABBCache(self)
    self.extract_base = DeterminisiticCache(lambda q: Config(base_values_from_full_config(q.value)))

    #self.gripper_collision = GripperCollisionCache(self)
    self.approach_collision = ApproachCollisionCache(self)
    self.obj_approach_collision = ObjApproachCollisionCache(self)
    self.caches = [value for value in self.__dict__.values() if isinstance(value, DeterminisiticCache)]

  def setup_primitives(self):
    #self.base_roadmap = StarRoadmap(self.initial_config, lambda q1, q2: plan_base_traj(self, q1, q2))
    self.base_roadmap = MultiBiRRT(self.extract_base(self.initial_config),
      q_distance_fn(self.robot), q_sample_fn(self.robot), q_extend_fn(self.robot), q_collision_fn(self.env, self.robot))
    self.pick_and_places = defaultdict(list)
    self.pushes = defaultdict(Graph)

  def reset(self):
    self.setup_caches()
    self.setup_primitives()

  #################################################################

  def get_counter_objects(self): # TODO - replace objecets with this
    return self.objects

  def get_floor_objects(self):
    return self.floor_objects

  def get_objects(self):
    return self.objects + self.floor_objects

  def get_static_bodies(self): # NOTE - really just means not robot
    return self.objects() + self.obstacles

  def get_counters(self):
    return self.tables + self.sinks + self.stoves

  def get_floors(self):
    return self.floors

  def get_surfaces(self):
    return self.get_counters() + self.get_floors()

  #################################################################

  def is_active(self, body_name):
    return body_name in self.active

  def set_active(self, body_name, active):
    if active and not self.is_active(body_name):
      self.env.Add(self.bodies[body_name])
      self.active.add(body_name)
    if not active and self.is_active(body_name):
      self.env.Remove(self.bodies[body_name])
      self.active.remove(body_name)

  def set_active_state(self, active_robot=True, active_obstacles=True, active_objects=set()):
    self.set_active(get_name(self.robot), active_robot)
    for obst_name in self.obstacles:
      self.set_active(obst_name, active_obstacles)
    for obj_name in self.get_objects():
      self.set_active(obj_name, obj_name in active_objects)

  def get_active_objects(self):
    return [object_name for object_name in self.objects if self.is_active(object_name)]

  #################################################################

  def compute_part_aabb(self, part_name):
    return aabb_from_points(trans_transform_points(get_trans(self.robot_links[self.robot_parts[part_name][0]]),
        mesh_vertices(self.robot_convex_meshes[part_name])))

  #################################################################

  def get_body(self, body_name):
    return self.bodies[body_name]

  def get_region(self, region_name):
    return self.regions[region_name]

  def get_geom_hash(self, body_name):
    return self.body_name_to_geom_hash[body_name]

  def get_body_name(self, geometry_hash):
    return self.geom_hash_to_body_name[geometry_hash]

  def get_aabb(self, body_name, trans=None):
    return aabb_from_oobb(self.get_oobb(body_name, trans))
    #if trans is None: return self.aabbs[body_name]
    #return aabb_apply_trans(self.aabbs[body_name], trans)

  def get_oobb(self, body_name, trans=None):
    if trans is None: return self.oobbs[body_name]
    return OOBB(self.oobbs[body_name].aabb, np.dot(trans, self.oobbs[body_name].trans))

  def get_mesh(self, body_name, trans=None):
    if trans is None: return self.meshes[body_name]
    return mesh_apply_trans(self.meshes[body_name], trans)

  def get_vertices(self, body_name, trans=None):
    if trans is None: return mesh_vertices(self.meshes[body_name])
    return mesh_vertices(mesh_apply_trans(self.meshes[body_name], trans))

  def get_radius2D2(self, body_name):
    return self.radii2D[body_name]

  def get_radius2(self, body_name):
    return self.radii[body_name]

  #################################################################

  def sphere_collision(self, body_name1, point1, body_name2, point2):
    return length2(point1 - point2) <= self.get_radius2(body_name1) + self.get_radius2(body_name2)

  def cylinder_collision(self, body_name1, point1, body_name2, point2): # NOTE - does not account for out of plane rotations
    return length2((point1 - point2)[:2]) <= self.get_radius2D2(body_name1) + self.get_radius2D2(body_name2)

  def aabb_collision(self, body_name1, trans1, body_name2, trans2):
    return aabb_overlap(self.get_aabb(body_name1, trans1), self.get_aabb(body_name2, trans2))

  #################################################################

  def get_robot_config(self):
    return Config(get_full_config(self.robot))

  def set_robot_config(self, config):
    set_full_config(self.robot, config.value)

  def get_pose(self, body_name):
    if not self.is_active(body_name): return None
    return Pose(get_pose(self.bodies[body_name]))

  def set_pose(self, body_name, object_pose=None):
    if object_pose is not None:
      self.set_active(body_name, True)
      set_pose(self.bodies[body_name], object_pose.value)
    else:
      self.set_active(body_name, False)

  def set_poses(self, poses):
    for body_name, pose in poses:
      self.set_pose(body_name, poses)

  def set_all_object_poses(self, object_poses={}):
    for object_name in self.get_objects():
      self.set_pose(object_name, object_pose=object_poses.get(object_name, None))

  #################################################################

  def lock(self):
    if not self.locked and is_viewer_active(self.env):
      self.env.Lock()
      self.locked = True

  def unlock(self):
    if self.locked and is_viewer_active(self.env):
      self.env.Unlock()
      self.locked = False

  def body_saver(self, body_name):
    return body_saver(self.bodies[body_name])

  def robot_saver(self):
    return robot_saver(self.robot)

  def state_saver(self):
    return ManipulationOracleStateSaver(self)

  #################################################################

  def grow_base_roadmap(self, q):
    base_q = self.extract_base(q)
    if base_q in self.base_roadmap: return True
    with self.robot:
      try:
        CSpace.robot_base(self.robot).set_active()
        set_robot_config(self.robot, self.initial_config)
        with collision_saver(self.env, openravepy_int.CollisionOptions.ActiveDOFs):
          return self.base_roadmap.grow(base_q) is not None
      except openrave_exception: # TODO - extremely infrequently, (int)values0.size() != GetActiveDOF()?
        return False
    #return self.base_roadmap.grow(q) is not None

  def plan_base_roadmap(self, q1, q2):
    if not COMPUTE_BASE_TRAJECTORY: return tuple()

    base_q1, base_q2 = self.extract_base(q1), self.extract_base(q2)
    cspace = CSpace.robot_base(self.robot)
    if base_q1 in self.base_roadmap and base_q2 in self.base_roadmap:
      path = self.base_roadmap(base_q1, base_q2)
    else:
      with self.robot:
        cspace.set_active()
        set_robot_config(self.robot, self.initial_config)
        with collision_saver(self.env, openravepy_int.CollisionOptions.ActiveDOFs):
          path = self.base_roadmap(base_q1, base_q2)
    if path is None: return None
    return (PathTrajectory(cspace, [q.value for q in path]),)
    #return self.base_roadmap(q1, q2)
    #return (Trajectory.join(self.base_roadmap(q1, q2)),)

  #################################################################

  def get_paps(self, body_name):
    return self.pick_and_places[self.get_geom_hash(body_name)]

  def add_pap(self, body_name, pap):
    self.pick_and_places[self.get_geom_hash(body_name)].append(pap)

  #################################################################

  def draw_regions(self):
    for region in self.regions.values():
      region.draw(self.env)
    self.draw_goals()
  def clear_regions(self):
    for region in self.regions.values():
      region.clear()
  def draw_goals(self):
    self.goal_handles = []
    for object_name, pose in self.problem.goal_poses.items():
      self.goal_handles.append(draw_point(self.env, point_from_pose(pose.value), get_color(self.get_body(object_name))))
    for object_name, (surface, point) in self.problem.goal_constrained.items():
      self.goal_handles.append(draw_point(self.env, point.value, get_color(self.get_body(object_name))))
    for region in self.goal_regions:
      self.get_region(region).draw(self.env)
  def clear_goals(self):
    self.goal_handles = []
    for region in self.goal_regions:
      self.get_region(region).clear()

  def __str__(self): # TODO - only print lists if not empty
    return '%s\n' \
        'Collision checker: %s\n' \
        'Robot: %s\n' \
        'Counter objects: %s\n' \
        'Floor objects: %s\n' \
        'Obstacles: %s\n' \
        'Tables: %s\n' \
        'Sinks: %s\n' \
        'Stoves: %s\n' \
        'Goal regions: %s'%(self.__class__.__name__, self.env.GetCollisionChecker(), get_name(self.robot), self.objects,
                              self.floor_objects, self.obstacles, self.tables, self.sinks, self.stoves, self.goal_regions)
  __repr__ = __str__