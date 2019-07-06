import pickle
import os
import numpy as np
import sys
import socket


def get_time_taken(test_dir, stat):
    if test_dir.find('hpn') != -1:
        return stat['time_taken']
    elif test_dir.find('greedy') != -1:
        return stat.metrics['tottime']
    elif test_dir.find('mcts') != -1:
        return stat['search_time_to_reward'][-1][0]


def get_success(test_dir, stat):
    if test_dir.find('hpn') != -1:
        return stat['found_solution']
    elif test_dir.find('greedy') != -1:
        return stat.metrics['success']
    elif test_dir.find('mcts') != -1:
        return stat['search_time_to_reward'][-1][-1]


def get_num_nodes(test_dir, stat):
    if test_dir.find('hpn') != -1:
        return stat['n_nodes']
    elif test_dir.find('greedy') != -1:
        return stat.metrics['num_nodes']


def get_plan_length(test_dir, stat):
    if test_dir.find('hpn') != -1:
        return len(stat['plan'])
    elif test_dir.find('greedy') != -1:
        return stat.metrics['plan_length']


def get_pidx(test_dir, filename):
    if test_dir.find('hpn') != -1:
        return int(filename.split('pidx_')[1].split('.pkl')[0])
    else:
        return int(filename.split('pidx_')[1].split('_planner')[0])


def get_plan_times(test_dir, test_files, t_limit):
    successes = []
    time_taken = []
    print "Getting test stats from %d files in %s" % (len(test_files), test_dir)
    for filename in test_files:
        pidx = get_pidx(test_dir, filename)
        if pidx < 20000:
            continue

        stat = pickle.load(open(test_dir + filename, 'r'))
        ftime_taken = get_time_taken(test_dir, stat)
        fsuccess = get_success(test_dir, stat)

        if ftime_taken < t_limit:
            time_taken.append(ftime_taken)
            successes.append(fsuccess)
        else:
            time_taken.append(t_limit)
            successes.append(False)

    CI95 = 1.96 * np.std(time_taken) / np.sqrt(len(time_taken))
    print "Time taken %.3f +- %.3f" % (np.mean(time_taken), CI95)
    print "Success rate %.3f" % np.mean(successes)


def get_summary_stat_file_name(test_dir, n_data, n_objs):
    if test_dir.find('hpn') != -1:
        fname = 'hpn_n_objs_%d.pkl' % n_objs
    elif test_dir.find('greedy') != -1:
        if test_dir.find('no_gnn') != -1:
            if test_dir.find('num_goals') != -1:
                fname = 'greedy_n_objs_%d_no_gnn_num_goals.pkl' % n_objs
            else:
                fname = 'greedy_n_objs_%d_no_gnn.pkl' % n_objs
        elif test_dir.find('hcount') !=-1:
            fname = 'hcount_n_objs_%d.pkl' % n_objs
        else:
            if test_dir.find('helps_goal') != -1:
                fname = 'greedy_helps_goal_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data)
            elif test_dir.find('num_goals') != -1:
                fname = 'greedy_num_goals_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data)
            else:
                fname = 'greedy_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data)
    if test_dir.find('one_arm') != -1:
        fname = 'one_arm_' + fname

    if test_dir.find('dql') != -1:
        fname = 'dql_' + fname

    return fname


def get_metrics(test_dir, test_files, n_objs, n_data=None):
    successes = []
    time_taken = []
    num_nodes = []
    plan_lengths = []
    pidxs = []

    has = []
    for fidx, filename in enumerate(test_files):
        print "%d / %d" % (fidx, len(test_files))
        pidx = get_pidx(test_dir, filename)
        if pidx < 20000:
            continue
        #seed = int(filename.split('seed_')[1].split('_')[0])
        #train_seed = int(filename.split('train_seed_')[1].split('_')[0])
        #has.append((seed,train_seed,pidx))
        #print filename, seed, train_seed,pidx


        stat = pickle.load(open(test_dir + filename, 'r'))
        ftime_taken = get_time_taken(test_dir, stat)
        fsuccess = get_success(test_dir, stat)
        fnodes = get_num_nodes(test_dir, stat)
        pidxs.append(pidx)
        if fsuccess:
            plan_length = get_plan_length(test_dir, stat)
            plan_lengths.append(plan_length)
        time_taken.append(ftime_taken)
        successes.append(fsuccess)
        num_nodes.append(fnodes)
        """
    for pidx in range(20000,20100):
        for planner_seed in [0,1,2,3,4]:
            train_seed=0
            if (planner_seed,train_seed,pidx) not in has:
                print planner_seed, train_seed, pidx

        """

    stat_summary = {'pidxs': pidxs, 'times': time_taken, 'successes': successes, 'num_nodes': num_nodes, 'plan_length': plan_lengths}
    fname = get_summary_stat_file_name(test_dir, n_data, n_objs)
    pickle.dump(stat_summary, open('./plotters/stats/' + fname, 'wb'))


def get_dir(algo, n_objs, n_train=5000, domain='two_arm_mover'):
    root = '/home/beomjoon/Dropbox (MIT)/cloud_results/'

    if algo == 'hpn':
        fdir = root + 'prm_mcr_hpn_results_on_mover_domain/'
        fdir += domain
        fdir += "/%d/test_purpose/" % n_objs
    elif algo == 'hcount':
        fdir = root + 'greedy_results_on_mover_domain/'
        fdir += 'domain_%s/' % domain
        fdir += 'n_objs_pack_%d/' % n_objs
        fdir += "/hcount/"
    elif algo.find('greedy') != -1:
        fdir = root + 'greedy_results_on_mover_domain/' \
                      'domain_%s/' \
                      'n_objs_pack_%d/' \
                      '/' % (domain, n_objs)
        if algo.find('no_gnn') != -1:
            fdir += 'no_gnn/'
        elif algo.find('dql') == -1:
            fdir += 'gnn/loss_largemargin/num_train_%d/' % n_train
        else:
            fdir += 'gnn/loss_dql/num_train_%d/' % n_train
    else:
        raise NotImplementedError
    return fdir, os.listdir(fdir)


def main():
    n_objs = int(sys.argv[1])
    n_train = int(sys.argv[2])
    algo = sys.argv[3]
    test_dir, test_files = get_dir(algo, n_objs, n_train, domain='two_arm_mover')
    get_metrics(test_dir, test_files, n_objs, n_train)


if __name__ == '__main__':
    main()
