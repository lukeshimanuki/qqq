# this file collects data by calling a planner on whatever the environment
from problem_environments.mover_env import Mover
from problem_environments.one_arm_mover_env import OneArmMover
from planners.hierarchical_mcts import HierarchicalMCTS
from trajectory_representation.predicates.in_region import InRegion
from learn import model

import numpy as np
import random
import os
import argparse
import pickle
import time


def make_and_get_save_dir(parameters):
    if parameters.qinit:
        save_dir = './test_results/mcts_results_on_mover_domain/widening_' + str(parameters.w) \
                   + '/uct_' + str(parameters.uct) + '/learned_q_test/'
    else:
        save_dir = './test_results/mcts_results_on_mover_domain/widening_' + str(parameters.w) \
               + '/uct_' + str(parameters.uct) + '/raw_data/'
    problem_idx = parameters.problem_idx

    if not os.path.isdir(save_dir):
        os.makedirs(save_dir)

    if os.path.isfile(save_dir+'/'+str(problem_idx)+'.pkl'):
        print "Already done"
        return None

    return save_dir


def load_qinit(entity_index):
    num_edge_features = 3
    num_node_features = 12
    num_latent_features = 32
    n_layers = 1
    name_to_idx = pickle.load(open('entity_name_to_idx.pkl', 'r'))
    weight_file = 'learn/Q_weight_num_train_5000_optimizer_adadelta_n_layers_1_top_k_5_batch_size_32_seed_0_lr_1.0_val_portion_0.1_num_test_600_n_hidden_64_.hdf5'
    m = model.QModel(num_node_features, num_edge_features, num_latent_features, n_layers, name_to_idx, 5, weight_file)
    return m


def parse_parameters():
    parser = argparse.ArgumentParser(description='MCTS parameters')
    parser.add_argument('-uct', type=float, default=1.0)
    parser.add_argument('-w', type=float, default=5)
    parser.add_argument('-sampling_strategy', type=str, default='unif')
    parser.add_argument('-problem_idx', type=int, default=0)
    parser.add_argument('-domain', type=str, default='mover')
    parser.add_argument('-planner', type=str, default='mcts')
    parser.add_argument('-v', action='store_true', default=False)
    parser.add_argument('-debug', action='store_true', default=False)
    parser.add_argument('-mcts_iter', type=int, default=1000)
    parser.add_argument('-n_feasibility_checks', type=int, default=100)
    parser.add_argument('-qinit', action='store_true', default=False)
    parser.add_argument('-n_switch', type=int, default=5)
    parser.add_argument('-use_ucb', action='store_true', default=False)
    parser.add_argument('-pw', action='store_true', default=False)

    mcts_parameters = parser.parse_args()
    return mcts_parameters


def main():
    mcts_parameters = parse_parameters()
    np.random.seed(mcts_parameters.problem_idx)
    random.seed(mcts_parameters.problem_idx)

    save_dir = make_and_get_save_dir(mcts_parameters)
    #if save_dir is None:
    #    return

    if mcts_parameters.domain.find('one_arm') != -1:
        environment = OneArmMover()
        #goal_predicate = InRegion(environment.objects[0], )
        goal_predicate = None
    else:
        environment = Mover()
        goal_predicate = None

    if mcts_parameters.v:
        environment.env.SetViewer('qtcoin')

    if mcts_parameters.qinit:
        qinit = load_qinit(environment.entity_idx)
    else:
        qinit = None

    planner = HierarchicalMCTS(None, environment, 'mover', mcts_parameters, qinit)
    stime = time.time()
    plan = planner.solve()
    print "Time taken: %.2f" % (time.time()-stime)

    # I also need to store the state, but later
    if plan is not None:
        filename = save_dir + str(mcts_parameters.problem_idx)+'.pkl'
        planner.mcts.tree.make_tree_picklable()
        pickle.dump({"plan": plan, "seed": mcts_parameters.problem_idx, "search_tree": planner.mcts.tree},
                      open(filename, 'wb'))


if __name__ == '__main__':
    main()
