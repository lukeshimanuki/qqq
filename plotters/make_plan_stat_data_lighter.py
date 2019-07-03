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
        pickle.dump(stat_summary, open('./plotters/stats/hpn_n_objs_%d.pkl' % n_objs, 'wb'))
    elif test_dir.find('greedy') != -1:
        if test_dir.find('no_h') != -1:
            pickle.dump(stat_summary, open('./plotters/stats/greedy_n_objs_%d_no_h.pkl' % n_objs, 'wb'))
        elif test_dir.find('no_gnn') != -1:
            if test_dir.find('num_goals') != -1:
                pickle.dump(stat_summary, open('./plotters/stats/greedy_n_objs_%d_no_gnn_num_goals.pkl' % n_objs, 'wb'))
            else:
                pickle.dump(stat_summary, open('./plotters/stats/greedy_n_objs_%d_no_gnn.pkl' % n_objs, 'wb'))
        elif test_dir.find('helps_goal') != -1:
            pickle.dump(stat_summary, open('./plotters/stats/greedy_helps_goal_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data), 'wb'))
        elif test_dir.find('num_goals') != -1:
            pickle.dump(stat_summary,
                        open('./plotters/stats/greedy_num_goals_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data), 'wb'))
        else:
            pickle.dump(stat_summary, open('./plotters/stats/greedy_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data), 'wb'))


def get_metrics(test_dir, test_files, n_objs, n_data=None):
    successes = []
    time_taken = []
    num_nodes = []
    plan_lengths = []

    for fidx, filename in enumerate(test_files):
        print "%d / %d" % (fidx, len(test_files))
        pidx = get_pidx(test_dir, filename)
        if pidx < 20000:
            continue

        stat = pickle.load(open(test_dir + filename, 'r'))
        ftime_taken = get_time_taken(test_dir, stat)
        fsuccess = get_success(test_dir, stat)
        fnodes = get_num_nodes(test_dir, stat)
        if fsuccess:
            plan_length = get_plan_length(test_dir, stat)
        time_taken.append(ftime_taken)
        successes.append(fsuccess)
        num_nodes.append(fnodes)
        plan_lengths.append(plan_length)

    stat_summary = {'times': time_taken, 'successes': successes, 'num_nodes': num_nodes, 'plan_length': plan_lengths}
    save_summary(stat_summary, test_dir, n_data, n_objs)


def main():
    n_objs = 1

    test_dir = '/home/beomjoon/cloud_results/prm_mcr_hpn_results_on_mover_domain/%d/test_purpose/' % n_objs
    #test_files = os.listdir(test_dir)
    #get_metrics(test_dir, test_files, n_objs)


    test_dir = '/home/beomjoon/cloud_results/greedy_results_on_mover_domain/domain_two_arm_mover/n_objs_pack_%d/test_purpose/no_gnn/no_goal_obj_same_region/num_goals/' % n_objs
    test_files = os.listdir(test_dir)
    #get_metrics(test_dir, test_files, n_objs)

    n_train = 5000
    test_dir = '/home/beomjoon/cloud_results/greedy_results_on_mover_domain/n_objs_pack_%d/' \
               'test_purpose/num_train_%d/' % (n_objs, n_train)
    #test_files = os.listdir(test_dir)
    #get_metrics(test_dir, test_files, n_objs, n_train)

    n_train = 5000
    test_dir = '/home/beomjoon/cloud_results/greedy_results_on_mover_domain/n_objs_pack_%d/' \
               'test_purpose/no_h/num_train_%d/' % (n_objs, n_train)
    test_files = os.listdir(test_dir)
    #get_metrics(test_dir, test_files, n_objs, n_train)

    n_train = 5000
    test_dir = '/home/beomjoon/cloud_results/greedy_results_on_mover_domain/domain_two_arm_mover/n_objs_pack_%d' \
               '/test_purpose/no_goal_obj_same_region/num_goals/num_train_5000/' % n_objs
    test_files = os.listdir(test_dir)
    get_metrics(test_dir, test_files, n_objs, n_train)

if __name__ == '__main__':
    main()
