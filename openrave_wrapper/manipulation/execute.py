from manipulation.bodies.robot import set_gripper, grab, open_gripper, release
from manipulation.constants import EXECUTE_TRAJECTORIES, SMOOTH_TRAJECTORIES
from manipulation.primitives.utils import reverse_trajectories

STEP_ENDS = True
STEP_VECTOR = True

class StepExecutable:
  def step(self, oracle):
    raise NotImplementedError()

class Executable:
  def execute(self, oracle):
    raise NotImplementedError()

class MovePickable(StepExecutable, Executable):
  def step(self, oracle):
    oracle.set_robot_config(self.pap.approach_config)
    yield
    if STEP_VECTOR:
      oracle.set_robot_config(self.pap.vector_config)
      yield
    oracle.set_robot_config(self.pap.grasp_config)
    set_gripper(oracle, self.pap.grasp)
    grab(oracle, self.object_name)
    yield
    if STEP_VECTOR:
      oracle.set_robot_config(self.pap.vector_config)
      yield
    oracle.set_robot_config(self.pap.approach_config)
    yield
  def execute(self, oracle):
    EXECUTE_TRAJECTORIES(SMOOTH_TRAJECTORIES(self.base_trajs))
    EXECUTE_TRAJECTORIES(reverse_trajectories(self.pap.trajs))
    self.pap.grasp.gripper_traj.execute()
    grab(oracle, self.object_name)
    EXECUTE_TRAJECTORIES(self.pap.trajs)

class MovePlaceable(StepExecutable, Executable):
  def step(self, oracle):
    oracle.set_robot_config(self.pap.approach_config)
    yield
    if STEP_VECTOR:
      oracle.set_robot_config(self.pap.vector_config)
      yield
    oracle.set_robot_config(self.pap.grasp_config)
    open_gripper(oracle)
    release(oracle, self.object_name)
    yield
    if STEP_VECTOR:
      oracle.set_robot_config(self.pap.vector_config)
      yield
    oracle.set_robot_config(self.pap.approach_config)
    yield
  def execute(self, oracle):
    EXECUTE_TRAJECTORIES(SMOOTH_TRAJECTORIES(self.base_trajs))
    EXECUTE_TRAJECTORIES(reverse_trajectories(self.pap.trajs))
    release(oracle, self.object_name)
    self.pap.grasp.gripper_traj.reverse().execute()
    EXECUTE_TRAJECTORIES(self.pap.trajs)

class Moveable(StepExecutable, Executable):
  def step(self, oracle):
    if STEP_ENDS:
      path = [self.end_config] # [action.start_config, action.end_config]
      for config in path:
        oracle.set_robot_config(config)
        yield
    else:
      for traj in self.base_trajs:
        traj.cspace.set_active()
        path = [traj.end()]
        #path = traj.waypoints()[1:]
        #path = traj.path()[1:]
        for config in path:
          traj.cspace.body.SetActiveDOFValues(config)
          yield
  def execute(self, oracle):
    EXECUTE_TRAJECTORIES(SMOOTH_TRAJECTORIES(self.base_trajs))

class Pickable(StepExecutable, Executable):
  def step(self, oracle):
    oracle.set_robot_config(self.pap.approach_config)
    yield
    if STEP_VECTOR:
      oracle.set_robot_config(self.pap.vector_config)
      yield
    oracle.set_robot_config(self.pap.grasp_config)
    set_gripper(oracle, self.pap.grasp)
    grab(oracle, self.object_name)
    yield
    if STEP_VECTOR:
      oracle.set_robot_config(self.pap.vector_config)
      yield
    oracle.set_robot_config(self.pap.approach_config)
    yield
  def execute(self, oracle):
    EXECUTE_TRAJECTORIES(reverse_trajectories(self.pap.trajs))
    self.pap.grasp.gripper_traj.execute()
    grab(oracle, self.object_name)
    EXECUTE_TRAJECTORIES(self.pap.trajs)

class Placeable(StepExecutable, Executable):
  def step(self, oracle):
    oracle.set_robot_config(self.pap.approach_config)
    yield
    if STEP_VECTOR:
      oracle.set_robot_config(self.pap.vector_config)
      yield
    oracle.set_robot_config(self.pap.grasp_config)
    open_gripper(oracle)
    release(oracle, self.object_name)
    yield
    if STEP_VECTOR:
      oracle.set_robot_config(self.pap.vector_config)
      yield
    oracle.set_robot_config(self.pap.approach_config)
    yield
  def execute(self, oracle):
    EXECUTE_TRAJECTORIES(reverse_trajectories(self.pap.trajs))
    release(oracle, self.object_name)
    self.pap.grasp.gripper_traj.reverse().execute()
    EXECUTE_TRAJECTORIES(self.pap.trajs)
