from problem_environments.mover_env import Mover
from planners.resolve_spatial_constraints import ResolveSpatialConstraints

import os
import sys
import argparse
import pickle
import numpy as np
import random
import time


def make_and_get_save_dir(parameters):
    save_dir = './test_results/rsc_results_on_mover_domain/'
    save_dir += str(parameters.n_objs_pack) + '/'

    if not os.path.isdir(save_dir):
        os.makedirs(save_dir)

    return save_dir


def quit_if_already_tested(file_path):
    if os.path.isfile(file_path):
        finfo = pickle.load(open(file_path, 'r'))
        is_done = finfo['n_nodes'] == 1000 or finfo['n_remaining_objs'] == 0
        if is_done:
            print "Already done"
            sys.exit(-1)


def parse_parameters():
    parser = argparse.ArgumentParser(description='MCTS parameters')

    # mcts parameters
    parser.add_argument('-uct', type=float, default=1.0)
    parser.add_argument('-w', type=float, default=5)
    parser.add_argument('-sampling_strategy', type=str, default='unif')
    parser.add_argument('-problem_idx', type=int, default=0)
    parser.add_argument('-domain', type=str, default='mover')
    parser.add_argument('-planner', type=str, default='mcts')
    parser.add_argument('-v', action='store_true', default=False)
    parser.add_argument('-debug', action='store_true', default=False)
    parser.add_argument('-mcts_iter', type=int, default=1000)
    parser.add_argument('-n_feasibility_checks', type=int, default=500)
    parser.add_argument('-use_learned_q', action='store_true', default=True)
    parser.add_argument('-n_switch', type=int, default=5)
    parser.add_argument('-use_ucb', action='store_true', default=False)
    parser.add_argument('-pw', action='store_true', default=False)
    parser.add_argument('-n_parameters_to_test_each_sample_time', type=int, default=10)
    parser.add_argument('-n_motion_plan_trials', type=int, default=10)
    parser.add_argument('-n_objs_pack', type=int, default=1)
    parameters = parser.parse_args()
    return parameters


def find_plan_for_obj(obj_name, environment):
    rsc = ResolveSpatialConstraints(problem_env=environment,
                                    goal_object_name=obj_name,
                                    goal_region_name='home_region',
                                    misc_region_name='loading_region')
    plan_found = False
    stime = time.time()
    plan = None
    status = 'NoSolution'
    while not plan_found and rsc.get_num_nodes() < 100:
        plan, status = rsc.search(obj_name,
                                  parent_swept_volumes=None,
                                  obstacles_to_remove=[],
                                  objects_moved_before=[],
                                  plan=[])
        plan_found = status == 'HasSolution'
        if plan_found:
            print "Solution found"
        else:
            print "Restarting..."
    print "Time taken: %.2f" % (time.time() - stime)
    if plan_found:
        return plan, rsc.get_num_nodes(), status
    else:
        return [], rsc.get_num_nodes(), status


def execute_plan(plan):
    for p in plan:
        p.execute()


def save_plan(total_plan, total_n_nodes, n_remaining_objs, file_path):
    [p.make_pklable() for p in total_plan]
    pickle.dump({"plan": total_plan, 'n_nodes': total_n_nodes, 'n_remaining_objs': n_remaining_objs},
                open(file_path, 'wb'))


def run_rsc_on_problem(environment, goal_object_names, file_path):
    total_n_nodes = 0
    total_plan = []
    idx = 0
    while len(goal_object_names) > 0 or (total_n_nodes > 1000):
        goal_obj_name = goal_object_names[idx]
        plan, n_nodes, status = find_plan_for_obj(goal_obj_name, environment)
        total_n_nodes += n_nodes
        print goal_obj_name, goal_object_names, total_n_nodes

        if status == 'HasSolution':
            execute_plan(plan)
            goal_object_names = np.delete(goal_object_names, idx)
            idx -= 1  # because we delete an element
            total_plan += plan

        save_plan(plan, total_n_nodes, len(goal_object_names), file_path)
        if len(goal_object_names) == 0:
            break
        idx += 1
        idx %= len(goal_object_names)
        print 'plan saved'


def main():
    parameters = parse_parameters()

    save_dir = make_and_get_save_dir(parameters)
    file_path = save_dir + '/' + str(parameters.problem_idx) + '.pkl'
    quit_if_already_tested(file_path)

    np.random.seed(parameters.problem_idx)
    random.seed(parameters.problem_idx)

    environment = Mover(parameters.problem_idx)
    goal_object_names = np.random.permutation([obj.GetName() for obj in environment.objects[:parameters.n_objs_pack]])

    if parameters.v:
        environment.env.SetViewer('qtcoin')

    run_rsc_on_problem(environment, goal_object_names, file_path)


if __name__ == '__main__':
    main()
