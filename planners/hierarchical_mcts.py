import pickle
import numpy as np

from planners.flat_mcts.mcts import MCTS
from problem_environments.reward_function import AbstractProblemRewardFunction, ObstacleClearingProblemRewardFunction
from planners.subplanners.minimum_constraint_planner import OperatorMinimumConstraintPlanner, MinimumConstraintPlanner
from planners.subplanners.motion_planner import ArmBaseMotionPlanner
from trajectory_representation.swept_volume import PickAndPlaceSweptVolume

from trajectory_representation.operator import Operator
from mover_library.utils import visualize_path, two_arm_pick_object, two_arm_place_object, CustomStateSaver


class HierarchicalMCTS:
    def __init__(self, goal_predicate, problem_env, domain_name, mcts_parameters, qinit=None):
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
        self.node_to_swept_volumes = {}
        self.current_state = None
        self.update_current_state()
        self.n_switch = mcts_parameters.n_switch

        self.mcts = MCTS(self.mcts_parameters.w,
                         self.mcts_parameters.uct,
                         self.mcts_parameters.n_feasibility_checks,
                         self.problem_env,
                         depth_limit=4,
                         discount_rate=0.95,
                         check_reachability=True,
                         use_progressive_widening=mcts_parameters.pw,
                         use_ucb=mcts_parameters.use_ucb,
                         qinit=qinit)

    def update_current_state(self):
        # this updates the current state as we recurse down in the hierarchy
        self.current_state = CustomStateSaver(self.problem_env.env)

    @staticmethod
    def make_plan_pklable(plan):
        for p in plan:
            if 'object' in p.discrete_parameters.keys() and type(p.discrete_parameters['object']) != str:
                p.discrete_parameters['object'] = str(p.discrete_parameters['object'].GetName())
            elif 'region' in p.discrete_parameters.keys() and type(p.discrete_parameters['region']) != str:
                p.discrete_parameters['region'] = str(p.discrete_parameters['region'].name)
        return plan

    @staticmethod
    def load_plan(filename):
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

    @staticmethod
    def is_curr_node_equal_to_node_to_search_from(curr_node, node_to_search_from):
        if node_to_search_from is None:
            return True

        is_pick_node = node_to_search_from.A[0].type == 'two_arm_pick'

        if is_pick_node:
            return curr_node == node_to_search_from
        else:
            corresponding_pick_skeleton_node = node_to_search_from.parent.parent
            return curr_node == corresponding_pick_skeleton_node

    @staticmethod
    def get_child_using_plan(node, plan):
        return node.children[plan[node]]

    @staticmethod
    def get_objects_in_collision_with_pick_and_place(pick, place, swept_volume_to_clear):
        pick_collisions = swept_volume_to_clear.get_objects_in_collision_with_given_pick_swept_volume(
            pick.low_level_motion)
        place_collisions = swept_volume_to_clear.get_objects_in_collision_with_given_place_swept_volume(pick,
                                                                                                        place.low_level_motion)
        objs_in_collision = pick_collisions + [o for o in place_collisions if not (o in pick_collisions)]
        return objs_in_collision

    def is_time_to_search(self, curr_node, node_to_search_from):
        is_curr_node_leaf = curr_node is None or len(curr_node.children) == 0
        if is_curr_node_leaf:
            return True

        if curr_node.is_descendent_of(node_to_search_from):
            # why would this ever happen?
            if curr_node != node_to_search_from:
                import pdb;
                pdb.set_trace()
            return True

        is_pick_node = node_to_search_from.A[0].type == 'two_arm_pick'
        if is_pick_node:
            return curr_node == node_to_search_from
        else:
            corresponding_pick_skeleton_node = node_to_search_from.parent.parent
            return curr_node == corresponding_pick_skeleton_node

    def solve_namo(self, target_pick_skeleton, target_place_skeleton, parent_object_pick_op,
                   swept_volume_to_clear, objects_moved_in_higher_level_plan,
                   curr_node, best_trajs_at_nodes, node_to_search_from,
                   entire_plan, is_last_object_to_clear=False):
        self.problem_env.set_action_constraint(target_pick_skeleton, target_place_skeleton)
        self.mcts.set_place_operator_swept_volume_constraint(swept_volume_to_clear)
        self.problem_env.reward_function.set_swept_volume(swept_volume_to_clear)
        self.problem_env.reward_function.set_plan_skeleton(target_pick_skeleton, target_place_skeleton)

        target_object = target_pick_skeleton.discrete_parameters['object']
        if type(target_object) is str:
            target_object = self.problem_env.env.GetKinBody(target_object)

        motion_planner = OperatorMinimumConstraintPlanner(self.problem_env, target_object,
                                                          objects_moved_in_higher_level_plan,
                                                          'rrt',
                                                          parent_object_pick_op,
                                                          is_last_object_to_clear)
        self.problem_env.set_motion_planner(motion_planner)
        self.problem_env.set_exception_objs_when_disabling_objects_in_region(objects_moved_in_higher_level_plan)

        pap_traj = []
        if self.is_time_to_search(curr_node, node_to_search_from):
            is_pick_node = node_to_search_from is None or node_to_search_from.A[0].type == 'two_arm_pick'
            if not (curr_node is None) and not (node_to_search_from is None):
                print curr_node.idx, node_to_search_from.idx
            rwds, pap_traj, next_node = self.mcts.search(n_iter=1,
                                                         iteration_for_tree_logging=len(
                                                             self.mcts.search_time_to_reward),
                                                         node_to_search_from=node_to_search_from)
            if not next_node.is_goal_node:
                self.recursion_depth = 0
                return None, None

            if is_pick_node:
                pass
            else:
                pap_traj.insert(0, node_to_search_from.parent_action)
                pap_traj.insert(0, node_to_search_from.parent.parent_action)
            node_to_search_from = next_node  # after we got to the node, we would like to search from there
        else:
            print "Traversing to node_to_search_from"
            try:
                pap_traj = best_trajs_at_nodes[curr_node]  # this is the pick-and-place from curr_node
                next_node = curr_node.children[pap_traj[0]].children[pap_traj[1]].children[pap_traj[2]].children[
                    pap_traj[3]]
                assert next_node.is_goal_node
            except KeyError:
                import pdb;
                pdb.set_trace()

        if self.recursion_depth != 0:
            objects_moved_in_higher_level_plan.append(target_object)

        self.recursion_depth += 1

        is_curr_node_already_cleared_desired_obstacle = len(pap_traj) == 0
        if not is_curr_node_already_cleared_desired_obstacle:
            new_actions = pap_traj[-4:]
            best_pick = new_actions[1]
            best_place = new_actions[3]
            swept_volume_to_clear.add_swept_volume(best_pick, best_place)
        else:
            # this happens because we already cleared the obstacle
            self.current_state.Restore()
            print "This probably should not happen?"
            return [], next_node

        objs_in_collision = self.get_objects_in_collision_with_pick_and_place(best_pick, best_place,
                                                                              swept_volume_to_clear)

        if curr_node is not None:
            print "Curr node idx", curr_node.idx
            print "Objects in collision: ", objs_in_collision

        oidx = 0
        while len(objs_in_collision) > 0:
            o = objs_in_collision[0]
            is_next_node_leaf = len(next_node.children) == 0
            if is_next_node_leaf:
                next_pick_skeleton = Operator('two_arm_pick', {'object': o})
                next_place_skeleton = Operator('two_arm_place', {'region': self.problem_env.get_region_containing(o)})
                next_node.state_saver = self.current_state
                assert next_node.A[0].type == 'two_arm_pick'
                del next_node.A[0]
                next_node.add_actions([next_pick_skeleton])
            else:
                next_pick_skeleton = next_node.A[0]
                try:
                    assert next_pick_skeleton.is_discrete_parameters_eq_to(o)
                except:
                    import pdb;
                    pdb.set_trace()
                next_place_skeleton = Operator('two_arm_place', {'region': self.problem_env.get_region_containing(o)})

            plan_for_moving_o, next_node = self.solve_namo(next_pick_skeleton, next_place_skeleton,
                                                           best_pick, swept_volume_to_clear,
                                                           objects_moved_in_higher_level_plan,
                                                           next_node, best_trajs_at_nodes,
                                                           node_to_search_from, entire_plan,
                                                           oidx == len(objs_in_collision) - 1)
            if plan_for_moving_o is None:
                return None, None
            else:
                if len(next_node.children) == 0:
                    # this is to make sure that, when we revisit next node, we keep what we moved before in
                    # its A
                    node_to_search_from = next_node
                else:
                    node_to_search_from = next_node

                objs_in_collision \
                    = self.get_objects_in_collision_with_pick_and_place(best_pick, best_place, swept_volume_to_clear)

        # execute the action
        two_arm_pick_object(best_pick.discrete_parameters['object'], self.problem_env.robot,
                            best_pick.continuous_parameters)
        two_arm_place_object(self.problem_env.robot.GetGrabbed()[0], self.problem_env.robot,
                             best_place.continuous_parameters)
        self.update_current_state()

        # remove the swept volume constraint for the executed action
        swept_volume_to_clear.remove_swept_volumes(best_pick, best_place)

        # what if I took a different route to get here?
        # when it is not time to switch, I think I always have to reset the entire plan
        if best_pick not in entire_plan:
            entire_plan.append(best_pick)
        if best_place not in entire_plan:
            entire_plan.append(best_place)

        print "Moved:", best_pick.discrete_parameters['object']
        if curr_node is not None:
            print "Curr node idx ", curr_node.idx
            print "Next node idx ", next_node.idx

        return [best_pick, best_place], next_node

    def get_plan_up_to_given_node(self, node):
        curr_node = node
        plan = []
        while curr_node.parent is not None:
            plan.append(curr_node.parent_action)
            curr_node = curr_node.parent
        return plan[::-1]

    def get_best_trajs_up_to_given_node(self, node):
        # This function returns the list of best_trajs associated with each pick-and-place *before* node.
        if node is None:
            return []

        plan = self.get_plan_up_to_given_node(node)
        is_node_place_skeleton = node.is_operator_skeleton_node and node.A[0].type == 'two_arm_place'
        is_node_pick_skeleton = node.is_operator_skeleton_node and node.A[0].type == 'two_arm_pick'

        if is_node_place_skeleton:
            necessary_plan = plan[:-2]
        elif is_node_pick_skeleton:
            necessary_plan = plan
        else:
            raise ValueError("node_to_search_from should be of type pick or place skeleton")

        best_trajs = {}
        curr_node = self.mcts.tree.root
        for p in necessary_plan:
            is_begin_of_best_traj = curr_node.is_operator_skeleton_node and curr_node.A[0].type == 'two_arm_pick'
            if is_begin_of_best_traj:
                best_traj_begin_node = curr_node
                best_trajs[best_traj_begin_node] = [p]
            else:
                best_trajs[best_traj_begin_node].append(p)
            curr_node = curr_node.children[p]

        return best_trajs

    def get_swept_volume_to_clear_at_node(self):
        swept_volume_to_clear = PickAndPlaceSweptVolume(self.problem_env)
        return swept_volume_to_clear

    def is_time_to_switch(self, node):
        if node.is_operator_skeleton_node:
            node = self.mcts.get_best_child_node(node)

        is_pick_node = node.A[0].type == 'two_arm_pick'

        if len(node.Q) == 0:
            n_feasible_actions = 0
        else:
            root_node_reward_history = node.reward_history.values()
            root_node_reward_history = np.array([np.max(R) for R in root_node_reward_history])
            n_feasible_actions = np.sum(root_node_reward_history > self.problem_env.reward_function.infeasible_reward)

        print "n feasible actions is", n_feasible_actions
        if is_pick_node:
            we_evaluated_the_node_enough = n_feasible_actions >= self.n_switch  # and self.s0_node.Nvisited > 15
        else:
            we_evaluated_the_node_enough = n_feasible_actions >= self.n_switch  # and self.s0_node.Nvisited > self.n_switch

        return we_evaluated_the_node_enough

    def get_child_node_to_descend_to(self, node):
        child = self.mcts.get_best_child_node(node)
        if node.is_operator_skeleton_node:
            child = self.mcts.get_best_child_node(child)
        assert child.is_operator_skeleton_node

        return child

    def get_operator_skeletons_for_node(self, node, abstract_target_object):
        if type(abstract_target_object) == str:
            abstract_target_object = self.problem_env.env.GetKinBody(abstract_target_object)

        is_pick_node = 'object' in node.A[0].discrete_parameters.keys()
        if is_pick_node:
            target_object = node.A[0].discrete_parameters['object']
            pick_skeleton = Operator('two_arm_pick', {'object': target_object})
        else:
            target_object = node.parent_action.discrete_parameters['object']
            pick_skeleton = Operator('two_arm_pick', {'object': target_object})

        if target_object == abstract_target_object:
            place_skeleton = Operator('two_arm_place', {'region': self.problem_env.regions['home_region']})
        else:
            place_skeleton = Operator('two_arm_place', {'region': self.problem_env.regions['loading_region']})

        return pick_skeleton, place_skeleton

    def resolve_preconditions(self, abstract_pick, abstract_place):

        initial_state_saver = CustomStateSaver(self.problem_env.env)
        node_to_search_from = None
        # self.mcts.load_tree()
        # node_to_search_from = self.mcts.get_node_at_idx(40)
        # self.mcts.s0_node = self.mcts.get_node_at_idx(1)
        for i in range(100):
            print "NAMO Iteration ", i
            swept_volumes_to_clear = self.get_swept_volume_to_clear_at_node()
            best_trajs_at_nodes = self.get_best_trajs_up_to_given_node(node_to_search_from)
            initial_state_saver.Restore()
            self.update_current_state()

            # if I save the tree, then I can skip path planning and swept volume collision checking
            entire_plan = []
            namo_plan, _ = self.solve_namo(abstract_pick, abstract_place, None, swept_volumes_to_clear, [],
                                           self.mcts.s0_node, best_trajs_at_nodes, node_to_search_from,
                                           entire_plan, False)
            # print "Saving tree"
            # self.mcts.save_tree()

            if node_to_search_from is None:
                node_to_search_from = self.mcts.s0_node

            if self.is_time_to_switch(node_to_search_from):
                print 'Switching node!'
                node_to_search_from = self.get_child_node_to_descend_to(node_to_search_from)

            if namo_plan is not None:
                print "Found plan!"
                # self.mcts.save_tree()
                """
                for action in entire_plan:
                    action.execute()
                """
                break

        if namo_plan is not None:
            return entire_plan
        else:
            return None

    def plan_with_reachability_precondition(self, abstract_plan):
        abstract_plan = self.extract_operator_instances(abstract_plan)
        picks = abstract_plan[::2]  # this should be a part of MCTS
        places = abstract_plan[1::2]
        reward_function = ObstacleClearingProblemRewardFunction(self.problem_env)
        self.problem_env.set_reward_function(reward_function)
        self.update_current_state()

        """
        plan = []
        for abstract_pick, abstract_place in zip(picks, places):
            resolving_plan = self.resolve_preconditions(abstract_pick, abstract_place)
            if resolving_plan is not None:
                plan.append(resolving_plan)
            else:
                print "Problem not solvable within the given resource limit"
                return -1
        return plan
        """

        # just resolve the first thing for now.
        resolving_plan = self.resolve_preconditions(picks[0], places[0])
        return resolving_plan

    def plan_without_reachability_precondition(self):
        # todo plan a path to the target object
        reward_function = AbstractProblemRewardFunction(self.problem_env)

        motion_planner = ArmBaseMotionPlanner(self.problem_env, 'rrt')
        self.problem_env.set_motion_planner(motion_planner)
        self.problem_env.set_reward_function(reward_function)
        rwds, pap_traj, next_node = self.mcts.search(n_iter=10)
        import pdb;pdb.set_trace()

    def solve(self):
        abstract_plan = self.plan_without_reachability_precondition()
        # abstract_plan = self.load_plan('abstract_plan')
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
