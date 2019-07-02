from problem_environments.one_arm_mover_env import OneArmMover

import argparse
import pickle
import numpy as np
import random
import os


def parse_parameters():
    parser = argparse.ArgumentParser(description='HPN parameters')

    parser.add_argument('-pidx', type=int, default=0)
    parser.add_argument('-planner_seed', type=int, default=0)
    parser.add_argument('-timelimit', type=int, default=1000)
    parser.add_argument('-v', action='store_true', default=False)
    parser.add_argument('-f', action='store_true', default=False)
    parser.add_argument('-n_feasibility_checks', type=int, default=500)
    parser.add_argument('-n_parameters_to_test_each_sample_time', type=int, default=10)
    parser.add_argument('-n_motion_plan_trials', type=int, default=10)
    parser.add_argument('-n_objs_pack', type=int, default=1)
    parser.add_argument('-domain', type=str, default='one_arm_mover')

    # dummy variables
    parser.add_argument('-loss', type=str, default='asdf')
    parser.add_argument('-train_seed', type=int, default=1000)
    parser.add_argument('-num_train', type=int, default=1000)
    parameters = parser.parse_args()
    return parameters


def verify_plan(plan, problem_env):
    env = problem_env.env
    robot = problem_env.robot

    for action in plan:
        action.execute_pick()
        assert not env.CheckCollision(robot)
        action.execute()

    print "Test passed"


def main():
    parameters = parse_parameters()
    np.random.seed(parameters.pidx)
    random.seed(parameters.pidx)
    is_one_arm_env = parameters.domain.find('two_arm') != -1
    if is_one_arm_env:
        raise NotImplementedError
    else:
        environment = OneArmMover(parameters.pidx)

    if parameters.v:
        environment.env.SetViewer('qtcoin')

    filename = 'test_results/prm_mcr_hpn_results_on_mover_domain/1/test_purpose/seed_%d_pidx_%d.pkl' \
               % (parameters.planner_seed, parameters.pidx)
    assert os.path.isfile(filename), "No solution file"
    plan = pickle.load(open(filename, 'r'))['plan']

    verify_plan(plan, environment)




if __name__ == '__main__':
    main()
