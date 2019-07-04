import pickle
import os
import numpy as np


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


def save_summary(stat_summary, test_dir, n_data, n_objs):

    if test_dir.find('hpn') != -1:
        fname = 'hpn_n_objs_%d.pkl' % n_objs
    elif test_dir.find('greedy') != -1:
        if test_dir.find('no_gnn') != -1:
            if test_dir.find('num_goals') != -1:
                fname = 'greedy_n_objs_%d_no_gnn_num_goals.pkl' % n_objs
            else:
                fname = 'greedy_n_objs_%d_no_gnn.pkl' % n_objs
        else:
            if test_dir.find('helps_goal') != -1:
                fname = 'greedy_helps_goal_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data)
            elif test_dir.find('num_goals') != -1:
                fname = 'greedy_num_goals_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data)
            else:
                fname = 'greedy_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data)
    if test_dir.find('one_arm') != -1:
        fname = 'one_arm_' + fname

    pickle.dump(stat_summary, open('./plotters/stats/' + fname, 'wb'))



def get_metrics(test_dir, test_files, n_objs, n_data=None):
    successes = []
    time_taken = []
    num_nodes = []
    plan_lengths = []
    pidxs = []

    for fidx, filename in enumerate(test_files):
        print "%d / %d" % (fidx, len(test_files))
        pidx = get_pidx(test_dir, filename)
        if pidx < 20000:
            continue

        stat = pickle.load(open(test_dir + filename, 'r'))
        ftime_taken = get_time_taken(test_dir, stat)
        fsuccess = get_success(test_dir, stat)
        fnodes = get_num_nodes(test_dir, stat)
        pidxs.append(pidx)
        if pidx == 20022:
            import pdb;pdb.set_trace()
        if fsuccess:
            plan_length = get_plan_length(test_dir, stat)
        time_taken.append(ftime_taken)
        successes.append(fsuccess)
        num_nodes.append(fnodes)
        plan_lengths.append(plan_length)

    stat_summary = {'pidxs': pidxs, 'times': time_taken, 'successes': successes, 'num_nodes': num_nodes, 'plan_length': plan_lengths}
    save_summary(stat_summary, test_dir, n_data, n_objs)


def get_dir(algo, n_objs, domain='two_arm_mover'):
    root = '/home/beomjoon/cloud_results/'
    if algo == 'hpn':
        fdir = root + 'prm_mcr_hpn_results_on_mover_domain/'
        fdir += "/%d/test_purpose/" % n_objs
    elif algo == 'greedy':
        fdir = root + 'greedy_results_on_mover_domain/domain_%s/n_objs_pack_%d/test_purpose/no_goal_obj_same_region/num_goals/' \
                      '/num_train_5000/' % (domain, n_objs)
    return fdir, os.listdir(fdir)


def main():
    n_objs = 1
    n_train = 5000
    test_dir, test_files = get_dir('hpn', n_objs)
    #test_dir, test_files = get_dir('greedy', n_objs)

    get_metrics(test_dir, test_files, n_objs, n_train)


if __name__ == '__main__':
    main()
