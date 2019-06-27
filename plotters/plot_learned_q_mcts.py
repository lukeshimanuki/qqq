import pickle
import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
import numpy as np


def main():
    result_dir = 'test_results/mcts_results_on_mover_domain/widening_5/uct_1.0/learned_q_test//'
    successes = []
    n_nodes = []
    n_nodes_successes = []
    for fin in os.listdir(result_dir):
        if fin.find('.pkl') == -1:
            continue
        result = pickle.load(open(result_dir + fin, 'r'))

        is_success = result['search_time_to_reward'][-1][-1]
        successes.append(is_success)
        n_nodes.append(result['n_nodes'] / 2.0)
        if is_success:
            n_nodes_successes.append(result['n_nodes'] / 2.0)
    print "N = ", len(n_nodes)
    print 'Success rates', np.mean(successes)
    print 'n_nodes', np.mean(n_nodes)
    print '95% CI', np.std(n_nodes) * 1.96 / np.sqrt(len(n_nodes))
    print 'n_nodes in success episode', np.mean(n_nodes_successes)
    print '95% CI', np.std(n_nodes_successes) * 1.96 / np.sqrt(len(n_nodes_successes))


if __name__ == '__main__':
    main()
