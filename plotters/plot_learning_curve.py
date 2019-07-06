import pickle
import argparse
import os
import numpy as np
import copy

import sys
import matplotlib.pyplot as plt
import seaborn as sns

from make_plan_stat_data_lighter import get_summary_stat_file_name, get_dir


def load_data(algo, n_objs, domain, n_data=5000):
    test_dir, _ = get_dir(algo, n_objs, n_train=n_data, domain=domain)
    fname = get_summary_stat_file_name(test_dir, n_data, n_objs)
    #print fname
    return pickle.load(open('./plotters/stats/' + fname, 'r'))


def get_success_rate_at(time_data, success_data, interval):
    # success_data = copy.copy(success_data)
    rates = []
    for tlimit in interval:
        rates.append(np.sum(success_data[time_data <= tlimit]) / float(len(success_data)))
    return rates


def print_plan_time(statfile, max_time=None, max_nodes=None, algo_name=None):
    plantimes = np.array(statfile['times'])
    successes = np.array(statfile['successes'])
    num_nodes = np.array(statfile['num_nodes'])
    planlength = np.array(statfile['plan_length'])

    if max_nodes is not None:
        successes[num_nodes > max_nodes] = False
        num_nodes[num_nodes > max_nodes] = max_nodes
        num_nodes[~successes] = max_nodes
    else:
        successes[plantimes > max_time] = False
        plantimes[plantimes > max_time] = max_time
        plantimes[~successes] = max_time


    if algo_name is not None:
        print algo_name
    print "Numb data", len(successes)
    print "Success rate", np.mean(successes)
    print "Plan times", np.mean(plantimes), np.std(plantimes) * 1.96 / np.sqrt(len(plantimes))
    print "Nodes expandes", np.mean(num_nodes), np.std(num_nodes) * 1.96 / np.sqrt(len(num_nodes))
    print "Plan length", np.mean(planlength), np.std(planlength) * 1.96 / np.sqrt(len(planlength))
    print "=="
    return np.mean(successes)


def plot_success_vs_time(n_objs, n_data=5000, domain='two_arm_mover'):
    if n_objs == 8:
        max_time = 2400  # 300*n_objs
    elif n_objs == 1:
        max_time = 300
    else:
        raise NotImplementedError

    print "HPN"
    hpn = load_data('hpn', n_objs, domain)
    print_plan_time(hpn, max_time)
    print "Greedy"
    greedy = load_data('greedy', n_objs, domain=domain, n_data=n_data)
    print_plan_time(greedy, max_time)
    print "Greedy DQL"
    greedy_dql = load_data('greedy_dql', n_objs, domain=domain, n_data=n_data)
    print_plan_time(greedy_dql, max_time)
    print "Hcount"
    greedy_hcount = load_data('hcount', n_objs, domain=domain, n_data=n_data)
    print_plan_time(greedy_hcount, max_time)


def savefig(xlabel, ylabel, fname=''):
    plt.legend(loc='best', prop={'size': 13})
    plt.xlabel(xlabel, fontsize=14, fontweight='bold')
    plt.ylabel(ylabel, fontsize=14, fontweight='bold')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)

    print 'Saving figure ', fname + '.png'
    plt.tight_layout()
    plt.savefig(fname + '.png', dpi=100)


def plot_learning_curve():
    time_limit = 300
    n_objs = 1

    hpn = load_data('hpn', n_objs, 'two_arm_mover')

    hpn_times = np.array(hpn['times'])
    hpn_successes = np.array(hpn['successes'])
    hpn_successes[hpn_times > time_limit] = False
    hpn_rate = np.mean(hpn_successes)

    data_ranges = [100, 1000, 3000, 5000]
    rates = []
    dqn_rates = []
    for n_data in data_ranges:
        greedy = load_data('greedy', 1, 'two_arm_mover', n_data)
        greedy_dqn = load_data('greedy_dql', 1, 'two_arm_mover', n_data)
        greedy_rate = print_plan_time(greedy, time_limit,'greedy')
        dqn_rate = print_plan_time(greedy_dqn, time_limit,'greedy_dql')

        rates.append(greedy_rate)
        dqn_rates.append(dqn_rate)

    # todo avg over planner seeds
    plt.plot(data_ranges, [hpn_rate] * len(data_ranges), label='RSC', color=[0, 0, 1], marker='o')
    plt.plot(data_ranges, rates, label='GreedyLM', color=[1, 0, 0], marker='o')
    plt.plot(data_ranges, dqn_rates, label='GreedyDQN', color=[0, 0.5, 0], marker='o')
    plt.xticks(data_ranges)
    savefig("Number of training data", "Success rates within 300s", './plotters/learning_curve')


def get_sorted_pidxs_and_plan_times_sorted_according_to_pidxs(stat, max_time):
    plan_times = {}
    nfail = 0
    for pidx, plantime,success in zip(stat['pidxs'], stat['times'], stat['successes']):
        if pidx < 20000:
            continue
        if pidx == 20011 or pidx == 20023 or pidx == 20024 or pidx == 20049 or pidx == 20059 or pidx == 20094 or pidx == 20097:
            # we don't have data for these - fill them in later
            continue
        if pidx in plan_times:
            if plantime >= max_time:
                nfail += 1
                plantime = max_time
            else:
                try:
                    assert success
                except:
                    print pidx
            plan_times[pidx].append(plantime)
        else:
            if plantime >= max_time:
                nfail += 1
                plantime = max_time
            plan_times[pidx] = [plantime]
    print nfail / float(len(stat['pidxs']))
    return plan_times


def get_avg_time_per_pidx(stat, max_time):
    stattimes = get_sorted_pidxs_and_plan_times_sorted_according_to_pidxs(stat, max_time)
    avg_times = [np.mean(v) for v in stattimes.values()]
    CI = [np.std(v) * 1.96 / np.sqrt(len(v)) for v in stattimes.values()]
    return stattimes.keys(), avg_times, CI


def plot_scatter_plot(n_objs,domain):
    max_time = n_objs * 300
    print 'greedy'
    stat = load_data('greedy_num_goals', n_objs, n_data=5000, domain=domain)
    greedy_idxs, greedy_times, greedy_ci = get_avg_time_per_pidx(stat, max_time)

    print 'hpn'
    stat = load_data('hpn', n_objs, domain)
    hpn_idxs, hpn_times, hpn_ci = get_avg_time_per_pidx(stat, max_time)

    idxs = set(greedy_idxs).intersection(set(hpn_idxs))
    idxs = np.array(list(idxs))


    hpn_times_ = []
    greedy_times_ = []
    for idx in idxs:
        hidx = np.where(np.array(hpn_idxs) == idx)[0][0]
        gidx = np.where(np.array(greedy_idxs) == idx)[0][0]
        hpn_times_.append(hpn_times[hidx])
        greedy_times_.append(greedy_times[gidx])

    plt.plot(greedy_times_, hpn_times_, 'o', alpha=0.3)
    plt.plot(range(max_time+30), range(max_time+30), 'r')
    plt.xlim(0, max_time+30)
    plt.ylim(0, max_time+30)

    savefig("GreedyLM average times", "RSC times", './plotters/scatter_%d' % n_objs)


def main():
    # plot_learning_curve()
    n_objs = int(sys.argv[1])
    n_data = int(sys.argv[2])

    #plot_success_vs_time(n_objs, n_data, domain='one_arm_mover')
    plot_scatter_plot(n_objs, domain='two_arm_mover')
    #plot_learning_curve()
    pass


if __name__ == '__main__':
    main()
