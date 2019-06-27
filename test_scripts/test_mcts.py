from problem_environments.mover_env import Mover, PaPMoverEnv

from problem_environments.one_arm_mover_env import OneArmMover
from problem_environments.reward_functions.packing_problem.single_object_packing_reward_function \
    import ObjectPackingRewardFunction
from planners.flat_mcts.mcts import MCTS
from planners.subplanners.motion_planner import OperatorBaseMotionPlanner, ArmBaseMotionPlanner
from learn.pap_gnn import PaPGNN
from manipulation.bodies.bodies import set_color

import numpy as np
import random
import os
import argparse
import pickle
import time
import sys


def make_and_get_save_dir(parameters):
    if parameters.use_learned_q:
        save_dir = './test_results/mcts_results_on_mover_domain/widening_' + str(parameters.w) \
                   + '/uct_' + str(parameters.uct) + '/learned_q_test/' + str(parameters.domain) + '/' \
                   + str(parameters.n_objs_pack) + '/'
    else:
        save_dir = './test_results/mcts_results_on_mover_domain/widening_' + str(parameters.w) \
                   + '/uct_' + str(parameters.uct) + '/' + str(parameters.domain) + '/n_objs_to_pack_' \
                   + str(parameters.n_objs_pack) + '/'
    problem_idx = parameters.problem_idx

    if not os.path.isdir(save_dir):
        os.makedirs(save_dir)
    if os.path.isfile(save_dir + '/' + str(problem_idx) + '.pkl'):
        print "Already done"
        if not parameters.f:
            sys.exit(-1)

    return save_dir


def load_learned_q_functions(parameters, entities):
    num_entities = len(entities)
    dim_nodes = 10
    dim_edges = 16
    parameters.operator = 'two_arm_pick_two_arm_place'
    parameters.n_msg_passing = 1
    parameters.top_k = 1
    parameters.n_layers = 2
    parameters.mse_weight = 1.0
    parameters.loss = 'largemargin'
    m = PaPGNN(num_entities, dim_nodes, dim_edges, parameters, entities)
    m.weight_file_name = './learn/q-function-weights/Q_weight_n_msg_passing_1_mse_weight_1.0_optimizer_adam_seed_1_lr_0.0001_operator_two_arm_pick_two_arm_place_n_layers_2_n_hidden_32_top_k_1_num_train_2000_loss_largemargin.hdf5'
    m.load_weights()
    return m


def parse_parameters():
    parser = argparse.ArgumentParser(description='MCTS parameters')

    # mcts parameters
    parser.add_argument('-uct', type=float, default=1.0)
    parser.add_argument('-w', type=float, default=5)
    parser.add_argument('-sampling_strategy', type=str, default='unif')
    parser.add_argument('-problem_idx', type=int, default=0)
    parser.add_argument('-domain', type=str, default='pap_mover')
    parser.add_argument('-planner', type=str, default='mcts')
    parser.add_argument('-v', action='store_true', default=False)
    parser.add_argument('-debug', action='store_true', default=False)
    parser.add_argument('-mcts_iter', type=int, default=1000)
    parser.add_argument('-n_feasibility_checks', type=int, default=100)
    parser.add_argument('-use_learned_q', action='store_true', default=False)
    parser.add_argument('-n_switch', type=int, default=5)
    parser.add_argument('-use_ucb', action='store_true', default=False)
    parser.add_argument('-pw', action='store_true', default=False)
    parser.add_argument('-n_parameters_to_test_each_sample_time', type=int, default=10)
    parser.add_argument('-n_motion_plan_trials', type=int, default=10)
    parser.add_argument('-n_objs_pack', type=int, default=1)
    parser.add_argument('-f', action='store_true', default=False)

    # q-function parameters
    parser.add_argument('-n_hidden', type=int, default=32)
    parser.add_argument('-n_layers', type=int, default=2)
    parser.add_argument('-seed', type=int, default=0)
    parser.add_argument('-lr', type=float, default=1e-4)
    parser.add_argument('-optimizer', type=str, default='adam')
    parser.add_argument('-batch_size', type=int, default=32)
    parser.add_argument('-num_test', type=int, default=600)
    parser.add_argument('-num_train', type=int, default=5000)
    parser.add_argument('-val_portion', type=float, default=0.1)
    parser.add_argument('-top_k', type=int, default=1)
    parser.add_argument('-use_mse', action='store_true', default=False)
    parser.add_argument('-donttrain', action='store_true', default=False)
    parser.add_argument('-same_vertex_model', action='store_true', default=False)
    parser.add_argument('-diff_weight_msg_passing', action='store_true', default=False)
    parser.add_argument('-operator', type=str, default='two_arm_pick')
    parser.add_argument('-num_fc_layers', type=int, default=2)
    parser.add_argument('-no_goal_nodes', action='store_true', default=False)
    parser.add_argument('-n_msg_passing', type=int, default=0)
    parser.add_argument('-weight_initializer', type=str, default='glorot_uniform')
    parser.add_argument('-mse_weight', type=float, default=1.0)
    parameters = parser.parse_args()
    return parameters


def main():
    parameters = parse_parameters()
    np.random.seed(parameters.problem_idx)
    random.seed(parameters.problem_idx)

    save_dir = make_and_get_save_dir(parameters)
    if parameters.domain.find('one_arm') != -1:
        environment = OneArmMover(parameters.problem_idx)
        # following illustrates the situation where the entity occuluding the goal entity has the second highest value
        # goal_entities = ['r_obst0', environment.regions['rectangular_packing_box1_region'].name] # seed=0
        goal_entities = ['r_obst0', environment.regions['rectangular_packing_box1_region'].name]
        motion_planner = ArmBaseMotionPlanner(environment, 'rrt')
        reward_function = ObjectPackingRewardFunction(environment,
                                                      ['r_obst0'],
                                                      'rectangular_packing_box1_region')
    else:
        if parameters.domain.find('pap') != -1:
            environment = PaPMoverEnv(parameters.problem_idx)
        else:
            environment = Mover(parameters.problem_idx)

        goal_object_names = [obj.GetName() for obj in environment.objects[:parameters.n_objs_pack]]
        goal_object_names = ['rectangular_packing_box1']
        set_color(obj, [1,0,0])
        goal_region_name = [environment.regions['home_region'].name]
        goal_entities = goal_object_names + goal_region_name
        motion_planner = OperatorBaseMotionPlanner(environment, 'prm')
        reward_function = ObjectPackingRewardFunction(environment,
                                                      goal_object_names,
                                                      goal_region_name[0])
    environment.set_reward_function(reward_function)
    environment.set_motion_planner(motion_planner)

    if parameters.v:
        environment.env.SetViewer('qtcoin')

    if parameters.use_learned_q:
        learned_q_functions = load_learned_q_functions(parameters, environment.entity_names)
    else:
        learned_q_functions = None

    planner = MCTS(parameters.w,
                   parameters.uct,
                   parameters.n_feasibility_checks,
                   environment,
                   n_parameters_to_test_each_sample_time=parameters.n_parameters_to_test_each_sample_time,
                   depth_limit=100,
                   discount_rate=1,
                   check_reachability=True,
                   use_progressive_widening=parameters.pw,
                   use_ucb=parameters.use_ucb,
                   learned_q_function=learned_q_functions,
                   n_motion_plan_trials=parameters.n_motion_plan_trials,
                   goal_entities=goal_entities)

    stime = time.time()
    search_time_to_reward, plan = planner.search(max_time=300)
    print "Time taken: %.2f" % (time.time() - stime)

    # I also need to store the state, but later
    save_dir = make_and_get_save_dir(parameters)
    filename = save_dir + str(parameters.problem_idx) + '.pkl'

    pickle.dump({"search_time_to_reward": search_time_to_reward,
                 "pidx": parameters.problem_idx,
                 'n_nodes': len(planner.tree.get_instance_nodes())}, open(filename, 'wb'))


if __name__ == '__main__':
    main()
