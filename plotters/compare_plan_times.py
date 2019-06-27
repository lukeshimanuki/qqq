import pickle
import os
import numpy as np


def get_plan_times(test_dir, test_files, t_limit):
    successes = []
    time_taken = []
    print "Getting test stats from %d files in %s" % (len(test_files), test_dir)
    for filename in test_files:
        stat = pickle.load(open(test_dir + filename, 'r'))
        time_taken.append(stat['time_taken'])
        if stat['time_taken'] < t_limit:
            successes.append(stat['found_solution'])
        else:
            successes.append(False)
        print filename, stat['time_taken']

    CI95 = 1.96 * np.std(time_taken) / np.sqrt(len(time_taken))
    print "Time taken %.3f +- %.3f" % (np.mean(time_taken), CI95)
    print "Success rate %.3f" % np.mean(successes)


def main():
    test_dir = './test_results/hpn_results_on_mover_domain/results_from_cloud/tamp_q_results/test_results/hpn_results_on_mover_domain/2/test_purpose/'
    test_files = os.listdir(test_dir)
    get_plan_times(test_dir, test_files, 1000)
    """
    hpn_test_dir = './test_results/hpn_results_on_mover_domain/1/test_purpose/'
    mcts_test_dir = './test_results/mcts_results_on_mover_domain/widening_5/uct_1.0/learned_q_test/pap_mover/1/'

    hpn_test_set = set(os.listdir(hpn_test_dir))
    mcts_test_set = set(os.listdir(mcts_test_dir))

    test_files = hpn_test_set.intersection(mcts_test_set)

    hpn_time_taken = []
    mcts_time_taken = []

    n_hpn_success = []
    n_mcts_success = []

    for filename in test_files:
        hpn = pickle.load(open(hpn_test_dir + filename, 'r'))
        t_hpn = hpn['time_taken']
        mcts = pickle.load(open(mcts_test_dir + filename, 'r'))
        t_mcts = mcts['search_time_to_reward'][-1][0]

        t_limit = 300
        if t_hpn > t_limit:
            n_hpn_success.append(False)
            hpn_time_taken.append(t_limit)
        else:
            hpn_time_taken.append(t_hpn)
            if len(hpn['plan']) > 0:
                n_hpn_success.append(True)
            else:
                n_hpn_success.append(False)

        if t_mcts > t_limit:
            n_mcts_success.append(False)
            mcts_time_taken.append(t_limit)
        else:
            mcts_time_taken.append(t_mcts)
            if mcts['search_time_to_reward'][-1][-1]:
                n_mcts_success.append(True)
            else:
                print 'Non time limit failure'
                n_mcts_success.append(False)

        print '%s mcts: %.2f hpn: %.2f' % (filename, mcts['search_time_to_reward'][-1][0], hpn['time_taken'])

    print hpn_time_taken
    print mcts_time_taken
    hpn_time_taken = np.array(hpn_time_taken)
    mcts_time_taken = np.array(mcts_time_taken)
    n_data = len(hpn_time_taken)
    print 'ndata', n_data
    print 'hpn', np.mean(hpn_time_taken), np.std(hpn_time_taken) / np.sqrt(len(hpn_time_taken)) * 1.96
    print 'mcts', np.mean(mcts_time_taken), np.std(mcts_time_taken) / np.sqrt(len(hpn_time_taken)) * 1.96
    print 'hpn-mcts', np.mean(hpn_time_taken-mcts_time_taken), np.std(hpn_time_taken-mcts_time_taken) / np.sqrt(len(hpn_time_taken)) * 1.96
    print 'hpn', np.mean(n_hpn_success)
    print 'mcts', np.mean(n_mcts_success)
    """

if __name__ == '__main__':
    main()