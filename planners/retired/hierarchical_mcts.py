import pickle

from planners.flat_mcts.mcts import MCTS
from problem_environments.reward_function import AbstractProblemRewardFunction, ObstacleClearingProblemRewardFunction
from planners.subplanners.minimum_constraint_planner import OperatorMinimumConstraintPlanner, MinimumConstraintPlanner
from trajectory_representation.swept_volume import PickAndPlaceSweptVolume

from manipulation.primitives.savers import DynamicEnvironmentStateSaver

from trajectory_representation.operator import Operator
from generators.uniform import UniformGenerator
from manipulation.bodies.bodies import set_color
from mover_library.utils import visualize_path, two_arm_pick_object, two_arm_place_object


class HierarchicalMCTS:
    def __init__(self, goal_predicate, problem_env, domain_name, mcts_parameters):
        self.goal_predicate = goal_predicate  # dummy variable for now
        self.problem_env = problem_env
        self.domain_name = domain_name

        self.widening_parameter = None
        self.uct_parameter = None
        self.sampling_stategy = None
        self.task_plan_idx = 0
        self.obj_plan_idx = 0

        self.mcts_parameters = mcts_parameters
        self.recursion_depth = 0
        self.robot = self.problem_env.robot

    @staticmethod
    def make_plan_pklable(plan):
        for p in plan:
            if 'object' in p.discrete_parameters.keys() and type(p.discrete_parameters['object']) != str:
                p.discrete_parameters['object'] = str(p.discrete_parameters['object'].GetName())
            elif 'region' in p.discrete_parameters.keys() and type(p.discrete_parameters['region']) != str:
                p.discrete_parameters['region'] = str(p.discrete_parameters['region'].name)
        return plan

    def load_plan(self, filename):
        savepath = './test_results/mcts_results_on_mover_domain/' + filename + '.pkl'
        plan = pickle.load(open(savepath, 'r'))
        return plan

    def save_plan(self, plan, filename):
        pklable_plan = self.make_plan_pklable(plan)
        savepath = './test_results/mcts_results_on_mover_domain/' + filename + '.pkl'
        pickle.dump(pklable_plan, open(savepath, 'wb'))

    @staticmethod
    def extract_operator_instances(abstract_plan):
        return abstract_plan[1::2]

    def solve_namo(self, target_pick_skeleton, target_place_skeleton, swept_volume_to_clear,
                   objects_moved_in_higher_level_plan, mcts):

        # define constraint on applicable ops
        self.problem_env.set_action_constraint(target_pick_skeleton, target_place_skeleton)

        # define the constraints
        mcts.set_place_operator_swept_volume_constraint(swept_volume_to_clear)
        self.problem_env.reward_function.set_swept_volume(swept_volume_to_clear)
        self.problem_env.reward_function.set_plan_skeleton(target_pick_skeleton, target_place_skeleton)

        # define the motion planner to use
        target_object = target_pick_skeleton.discrete_parameters['object']
        if type(target_object) is str:
            target_object = self.problem_env.env.GetKinBody(target_object)
        motion_planner = OperatorMinimumConstraintPlanner(self.problem_env, target_object,
                                                          objects_moved_in_higher_level_plan)
        self.problem_env.set_motion_planner(motion_planner)

        # set objs already moved as an exception
        self.problem_env.set_exception_objs_when_disabling_objects_in_region(objects_moved_in_higher_level_plan)

        rwds, best_traj, best_leaf_node = mcts.search(n_iter=1, iteration_for_tree_logging=len(mcts.search_time_to_reward))

        if not best_leaf_node.is_goal_node:
            # todo we need to reset the mcts
            # set the initial root node to the mcts.original root
            mcts.original_s0_node.state_saver = mcts.original_s0_node_saver
            mcts.switch_init_node(mcts.original_s0_node)
            mcts.tree.set_root_node(mcts.original_s0_node)
            self.recursion_depth = 0
            return None

        if self.recursion_depth != 0:
            objects_moved_in_higher_level_plan.append(target_object)

        self.recursion_depth += 1
        print objects_moved_in_higher_level_plan

        # update the swept volume
        new_actions = best_traj[-4:]
        best_pick = new_actions[1]
        best_place = new_actions[3]
        swept_volume_to_clear.add_swept_volume(best_pick, best_place)

        pick_collisions = swept_volume_to_clear.get_objects_in_collision_with_given_pick_swept_volume(
                                                                                            best_pick.low_level_motion)
        place_collisions = swept_volume_to_clear.get_objects_in_collision_with_given_place_swept_volume(best_pick,
                                                                                            best_place.low_level_motion)
        objs_in_collision = pick_collisions + [o for o in place_collisions if not(o in pick_collisions)]
        print 'Objects in collision', len(objs_in_collision)

        for oidx, o in enumerate(objs_in_collision):
            next_pick_skeleton = Operator('two_arm_pick', {'object': o})
            next_place_skeleton = Operator('two_arm_place', {'region': self.problem_env.get_region_containing(o)})

            best_leaf_node.state_saver = mcts.tree.root.state_saver
            del best_leaf_node.A[0]
            best_leaf_node.add_actions([next_pick_skeleton])
            best_leaf_node.is_goal_node = False
            mcts.switch_init_node(best_leaf_node)

            if oidx > 0:
                print oidx, objs_in_collision
                import pdb;pdb.set_trace()

            best_leaf_node = self.solve_namo(next_pick_skeleton, next_place_skeleton, swept_volume_to_clear,
                                             objects_moved_in_higher_level_plan, mcts)
            if best_leaf_node is None:
                return None

        # execute the action
        two_arm_pick_object(best_pick.discrete_parameters['object'], self.problem_env.robot,
                            best_pick.continuous_parameters)
        two_arm_place_object(self.problem_env.robot.GetGrabbed()[0], self.problem_env.robot,
                             best_place.continuous_parameters)

        # remove the swept volume constraint
        swept_volume_to_clear.remove_swept_volumes(best_pick, best_place)

        # change the statesaver
        mcts.tree.root.state_saver = DynamicEnvironmentStateSaver(self.problem_env.env)
        return best_leaf_node

    def plan_with_reachability_precondition(self, abstract_plan):
        abstract_plan = self.extract_operator_instances(abstract_plan)
        picks = abstract_plan[::2]  # this should be a part of MCTS
        places = abstract_plan[1::2]
        reward_function = ObstacleClearingProblemRewardFunction(self.problem_env)
        self.problem_env.set_reward_function(reward_function)

        mcts = MCTS(self.mcts_parameters.widening_parameter, self.mcts_parameters.uct,
                    self.mcts_parameters.n_feasibility_checks, self.problem_env, True, 4, 1)

        plan = []
        namo_plan = None
        for abstract_pick, abstract_place in zip(picks, places):
            for i in range(1, 100):
                swept_volumes_to_clear = PickAndPlaceSweptVolume(self.problem_env)
                namo_plan = self.solve_namo(abstract_pick, abstract_place, swept_volumes_to_clear, [], mcts)
                if i % 10 == 0:
                    # switch
                    best_child_node = mcts.choose_child_node_to_descend_to()
                    mcts.switch_init_node(best_child_node)
                    mcts.original_s0_node = best_child_node
                    mcts.original_s0_node_saver = best_child_node.state_saver
                    mcts.log_current_tree_to_dot_file(len(mcts.search_time_to_reward))
                    leaves = mcts.tree.get_leaf_nodes()
                    for l in leaves:
                        l.is_goal_node = False
                    import pdb; pdb.set_trace()

                if namo_plan is not None:
                    break

            if namo_plan is not None:
                plan += namo_plan
            else:
                print "Problem not solvable within the given resource limit"
                return -1

    def solve(self):
        # abstract_plan = self.plan_without_reachability_precondition()
        abstract_plan = self.load_plan('abstract_plan')
        concrete_plan = self.plan_with_reachability_precondition(abstract_plan)

        # return the solution trajectory
        return concrete_plan

    def simulate_trajectory(self, trajectory):
        root_node = self.mcts.s0_node
        self.problem_env.reset_to_init_state(root_node)
        self.problem_env.env.SetViewer('qtcoin')

        for operator_instance in trajectory:
            if operator_instance.continuous_parameters is None:
                continue
            reward = self.problem_env.apply_operator_instance(operator_instance, False)
            print reward
