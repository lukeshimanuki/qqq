class RewardFunction:
    def __init__(self, problem_env):
        self.problem_env = problem_env
        self.robot = problem_env.robot
        self.infeasible_reward = -2

    def apply_operator_instance_and_get_reward(self, state, operator_instance, is_op_feasible):
        raise NotImplementedError

    def apply_operator_skeleton_and_get_reward(self, state, operator_skeleton):
        return 0

    def is_goal_reached(self):
        raise NotImplemented
