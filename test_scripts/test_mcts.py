from problem_environments.mover_env import Mover, PaPMoverEnv

from problem_environments.one_arm_mover_env import OneArmMover, PaPOneArmMoverEnv
from problem_environments.reward_functions.packing_problem.single_object_packing_reward_function \
    import ObjectPackingRewardFunction
from planners.flat_mcts.mcts import MCTS
from planners.subplanners.motion_planner import OperatorBaseMotionPlanner, ArmBaseMotionPlanner
from learn.pap_gnn import PaPGNN
from manipulation.bodies.bodies import set_color
from mover_library import utils
import numpy as np
import random
import os
import argparse
import pickle
import time
import sys
import socket


def make_and_get_save_dir(parameters):
    hostname = socket.gethostname()
    if hostname == 'dell-XPS-15-9560' or hostname == 'phaedra' or hostname == 'shakey' or hostname == 'lab' or \
            hostname == 'glaucus':
        root_dir = './'
    else:
        root_dir = '/data/public/rw/pass.port/tamp_q_results/'

    save_dir = root_dir + '/test_results/mcts_results_on_mover_domain/' \
               + 'n_objs_pack_' + str(parameters.n_objs_pack) + '/' \
               + 'n_mp_params_' + str(parameters.n_motion_plan_trials) + '/' \
               + 'widening_' + str(parameters.w) \
               + '/uct_' + str(parameters.uct)
    pidx = parameters.pidx

    if not os.path.isdir(save_dir):
        os.makedirs(save_dir)

    if os.path.isfile(save_dir + '/' + str(pidx) + '.pkl'):
        print "Already done"
        if not parameters.f:
            sys.exit(-1)

    return save_dir


def load_learned_q_functions(parameters, entities):
    num_entities = len(entities)
    dim_nodes = 10
    dim_edges = 44
    parameters.operator = 'two_arm_pick_two_arm_place'
    parameters.n_msg_passing = 1
    parameters.top_k = 1
    parameters.n_layers = 2
    parameters.mse_weight = 1.0
    parameters.optimizer = 'adam'
    parameters.loss = 'largemargin'
    parameters.seed = parameters.train_seed

    m = PaPGNN(num_entities, dim_nodes, dim_edges, parameters, entities)
    m.weight_file_name \
        = './learn/q-function-weights/Q_weight_n_msg_passing_1_mse_weight_1.0_optimizer_' \
          'adam_seed_%d_lr_0.0001_operator_two_arm_pick_two_arm_place_n_layers_2_n_hidden_' \
          '32_top_k_1_num_train_%d_loss_%s.hdf5' % (parameters.train_seed, parameters.num_train, parameters.loss)
    m.load_weights()
    return m


def parse_parameters():
    parser = argparse.ArgumentParser(description='MCTS parameters')
    parser.add_argument('-n_hidden', type=int, default=32)
    parser.add_argument('-n_layers', type=int, default=2)
    parser.add_argument('-train_seed', type=int, default=0)
    parser.add_argument('-lr', type=float, default=1e-4)
    parser.add_argument('-optimizer', type=str, default='adam')
    parser.add_argument('-batch_size', type=int, default=32)
    parser.add_argument('-num_test', type=int, default=1882)
    parser.add_argument('-num_train', type=int, default=5000)
    parser.add_argument('-val_portion', type=float, default=0.1)
    parser.add_argument('-top_k', type=int, default=1)
    parser.add_argument('-use_mse', action='store_true', default=False)
    parser.add_argument('-donttrain', action='store_true', default=False)
    parser.add_argument('-same_vertex_model', action='store_true', default=False)
    parser.add_argument('-diff_weight_msg_passing', action='store_true', default=False)
    parser.add_argument('-operator', type=str, default='two_arm_pick_two_arm_place')
    parser.add_argument('-num_fc_layers', type=int, default=2)
    parser.add_argument('-no_goal_nodes', action='store_true', default=False)
    parser.add_argument('-n_msg_passing', type=int, default=1)
    parser.add_argument('-weight_initializer', type=str, default='glorot_uniform')
    parser.add_argument('-loss', type=str, default='largemargin')
    parser.add_argument('-mse_weight', type=float, default=1.0)

    parser.add_argument('-uct', type=float, default=0.1)
    parser.add_argument('-w', type=float, default=3)
    parser.add_argument('-sampling_strategy', type=str, default='unif')
    parser.add_argument('-pidx', type=int, default=0)
    parser.add_argument('-domain', type=str, default='pap_mover')
    parser.add_argument('-planner', type=str, default='mcts')
    parser.add_argument('-v', action='store_true', default=False)
    parser.add_argument('-debug', action='store_true', default=False)
    parser.add_argument('-mcts_iter', type=int, default=1000)
    parser.add_argument('-n_feasibility_checks', type=int, default=200)
    parser.add_argument('-dont_use_learned_q', action='store_false', default=True)
    parser.add_argument('-use_learned_q', action='store_true', default=False)
    parser.add_argument('-n_switch', type=int, default=5)
    parser.add_argument('-use_ucb', action='store_true', default=False)
    parser.add_argument('-pw', action='store_true', default=False)
    parser.add_argument('-n_motion_plan_trials', type=int, default=3)
    parser.add_argument('-n_objs_pack', type=int, default=1)
    parser.add_argument('-timelimit', type=int, default=300)
    parser.add_argument('-f', action='store_true', default=False)
    parser.add_argument('-planner_seed', type=int, default=0)
    parameters = parser.parse_args()
    return parameters


def main():
    parameters = parse_parameters()
    save_dir = make_and_get_save_dir(parameters)

    np.random.seed(parameters.pidx)
    random.seed(parameters.pidx)
    if parameters.domain.find('one_arm') != -1:
        environment = PaPOneArmMoverEnv(parameters.pidx)
        goal_entities = ['r_obst0', environment.regions['rectangular_packing_box1_region'].name]
        motion_planner = ArmBaseMotionPlanner(environment, 'rrt')
        reward_function = ObjectPackingRewardFunction(environment,
                                                      ['r_obst0'],
                                                      'rectangular_packing_box1_region')
    else:
        if parameters.domain.find('pap') != -1:
            environment = PaPMoverEnv(parameters.pidx)
        else:
            environment = Mover(parameters.pidx)
        goal_object_names = [obj.GetName() for obj in environment.objects[:parameters.n_objs_pack]]
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

    np.random.seed(parameters.planner_seed)
    random.seed(parameters.planner_seed)

    planner = MCTS(parameters.w,
                   parameters.uct,
                   parameters.n_feasibility_checks,
                   environment,
                   depth_limit=11,
                   discount_rate=1,
                   check_reachability=True,
                   use_progressive_widening=parameters.pw,
                   use_ucb=parameters.use_ucb,
                   learned_q_function=learned_q_functions,
                   n_motion_plan_trials=parameters.n_motion_plan_trials,
                   goal_entities=goal_entities)

    stime = time.time()
    search_time_to_reward, plan = planner.search(max_time=parameters.timelimit)
    print "Time taken: %.2f" % (time.time() - stime)
    print "Number of nodes explored", len(planner.tree.get_discrete_nodes())

    # I also need to store the state, but later
    save_dir = make_and_get_save_dir(parameters)
    filename = save_dir + str(parameters.pidx) + '.pkl'

    pickle.dump({"search_time_to_reward": search_time_to_reward,
                 "pidx": parameters.pidx,
                 'n_nodes': len(planner.tree.get_discrete_nodes())}, open(filename, 'wb'))


if __name__ == '__main__':
    main()
