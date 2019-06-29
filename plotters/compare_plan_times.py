import pickle
import os
import numpy as np


def get_time_taken(test_dir, stat):
    if test_dir.find('hpn') != -1:
        return stat['time_taken']
    elif test_dir.find('greedy') != -1:
        return stat.metrics['tottime']


def get_success(test_dir, stat):
    if test_dir.find('hpn') != -1:
        return stat['found_solution']
    elif test_dir.find('greedy') != -1:
        return stat.metrics['success']


def get_plan_times(test_dir, test_files, t_limit):
    successes = []
    time_taken = []
    print "Getting test stats from %d files in %s" % (len(test_files), test_dir)
    for filename in test_files:
        stat = pickle.load(open(test_dir + filename, 'r'))
        ftime_taken = get_time_taken(test_dir, stat)
        fsuccess = get_success(test_dir,stat)

        """
        if filename.find('pidx_1_') !=-1 and test_dir.find('greedy') !=-1:
            print stat.metrics['num_nodes']
            print filename
            print ftime_taken
        """
        time_taken.append(ftime_taken)
        if ftime_taken < t_limit:
            successes.append(fsuccess)
        else:
            successes.append(False)

    CI95 = 1.96 * np.std(time_taken) / np.sqrt(len(time_taken))
    print "Time taken %.3f +- %.3f" % (np.mean(time_taken), CI95)
    print "Success rate %.3f" % np.mean(successes)


def main():
    n_objs = 1
    test_dir = 'test_results/cloud_results/faster_hpn_results_on_mover_domain/%d/test_purpose/' % n_objs
    test_dir = 'test_results/cloud_results/faster_hpn_results_on_mover_domain/%d/test_purpose/' % n_objs
    test_dir = '/home/beomjoon/Documents/github/qqq/test_results/hpn_results_on_mover_domain/results_from_cloud/tamp_q_results/test_results/hpn_results_on_mover_domain/%d/test_purpose/' % n_objs
    test_files = os.listdir(test_dir)
    get_plan_times(test_dir, test_files, 1000)

    test_dir = 'test_results/cloud_results/greedy_results_on_mover_domain/n_objs_pack_%d/test_purpose/num_train_5000/' % n_objs
    test_files = os.listdir(test_dir)
    get_plan_times(test_dir, test_files, 1000)


if __name__ == '__main__':
    main()