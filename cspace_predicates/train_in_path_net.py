from in_path_net import InPathNet

import argparse
import pickle
import numpy as np
import random
import os


def get_train_test_idxs(n_data, test_portion, labels):
    n_train = int(n_data*(1-test_portion))

    data_idxs = np.random.permutation(range(n_data))
    train_idxs = data_idxs[0:n_train]
    test_idxs = data_idxs[n_train:]

    # todo balance the negative and positive data points
    return train_idxs, test_idxs


def select_rows_with_given_idxs(matrix, idxs_1, idxs_2):
    return matrix[idxs_1, :], matrix[idxs_2, :]


def get_train_test_data(test_portion, n_planning_episodes):
    data_file = './test_results/mcts_results_on_mover_domain/widening_5/uct_1.0/in_path_net_data/data_' \
                + str(n_planning_episodes) +'_loading_region_.pkl'
    data = pickle.load(open(data_file, 'r'))
    n_data = len(data['relative_configs'])
    labels = data['labels']
    train_idxs, test_idxs = get_train_test_idxs(n_data, test_portion, labels)

    rel_konfs = data['relative_configs']
    rel_cgs = data['relative_cgs']

    train_konf, test_konf = select_rows_with_given_idxs(rel_konfs, train_idxs, test_idxs)
    train_cg, test_cg = select_rows_with_given_idxs(rel_cgs, train_idxs, test_idxs)
    train_labels, test_labels = select_rows_with_given_idxs(labels, train_idxs, test_idxs)
    return train_konf, train_cg, train_labels, test_konf, test_cg, test_labels


def set_seed(seed):
    np.random.seed(seed)
    random.seed(seed)


def write_results(predictions, labels, network_config):
    result_dir = './cspace_predicates/test_results/in_path_net_results/'
    if not os.path.isdir(result_dir):
        os.makedirs(result_dir)
    result_fname = ''
    config_variables = vars(network_config)
    for var_name, value in zip(config_variables.keys(), config_variables.values()):
        if var_name == 'seed':
            continue
        result_fname += var_name + '_' + str(value) + '_'
    result_fname += '.txt'

    fopen = open(result_dir+result_fname, 'a')
    values_to_write = '%d, %.3f\n' % (config_variables['seed'], np.mean(predictions == labels))
    fopen.write(values_to_write)


def main():
    parser = argparse.ArgumentParser(description='Process configurations')
    parser.add_argument('-n_hidden', type=int, default=2)
    parser.add_argument('-seed', type=int, default=1)
    parser.add_argument('-dense_num', type=int, default=64)
    parser.add_argument('-lr', type=float, default=1e-4)
    parser.add_argument('-use_dense', action='store_true', default=False)
    parser.add_argument('-optimizer', type=str, default='adam')
    parser.add_argument('-test_portion', type=float, default=0.1)
    parser.add_argument('-n_planning_episodes', type=int, default=800)
    parser.add_argument('-key_config_region', type=str, default='both')
    path_net_config = parser.parse_args()

    set_seed(path_net_config.seed)

    in_path_net = InPathNet(path_net_config)
    train_konf, train_cg, train_labels, test_konf, test_cg, test_labels = get_train_test_data(
        path_net_config.test_portion, path_net_config.n_planning_episodes)

    in_path_net.train(train_cg, train_konf, train_labels)

    # test
    pred = in_path_net.clf.predict([test_cg, test_konf]) > 0.5
    write_results(pred, test_labels, path_net_config)


if __name__ == '__main__':
    main()
