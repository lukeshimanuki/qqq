import pickle
import argparse
import os
import numpy as np
import copy

import matplotlib.pyplot as plt
import seaborn as sns


def load_data(algo, n_objs, domain=None, n_data=0):
    if algo == 'hpn':
        fname = 'hpn_n_objs_' + str(n_objs) + '.pkl'
    elif algo == 'greedy':
        fname = 'greedy_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data)
    elif algo == 'greedy_num_goals':
        fname = 'greedy_num_goals_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data)
    elif algo == 'greedy_helps_goal':
        fname = 'greedy_helps_goal_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data)
    elif algo == 'greedy_no_gnn':
        fname = 'greedy_n_objs_%d_no_gnn.pkl' % n_objs
    elif algo == 'greedy_no_gnn_num_goals':
        fname = 'greedy_n_objs_%d_no_gnn_num_goals.pkl' % n_objs
    elif algo == 'greedy_no_h':
        fname = 'greedy_n_objs_%d_no_h.pkl' % n_objs
    else:
        raise NotImplementedError

    if domain is not None:
        fname = domain + "_" + fname

    return pickle.load(open('./plotters/stats/' + fname, 'r'))


def get_success_rate_at(time_data, success_data, interval):
    # success_data = copy.copy(success_data)
    rates = []
    for tlimit in interval:
        rates.append(np.sum(success_data[time_data <= tlimit]) / float(len(success_data)))
    return rates


def print_plan_time(statfile, max_time):
    plantimes = np.array(statfile['times'])
    successes = np.array(statfile['successes'])
    num_nodes = np.array(statfile['num_nodes'])
    planlength = np.array(statfile['plan_length'])
    successes[plantimes > max_time] = False
    plantimes[plantimes > max_time] = max_time

    plantimes[~successes] = max_time
    print "Success rate", np.mean(successes)
    print "Plan times", np.mean(plantimes), np.std(plantimes) * 1.96 / np.sqrt(len(plantimes))
    print "Nodes expandes", np.mean(num_nodes), np.std(num_nodes) * 1.96 / np.sqrt(len(num_nodes))
    print "Plan length", np.mean(planlength), np.std(planlength) * 1.96 / np.sqrt(len(planlength))


def plot_success_vs_time(n_objs):
    if n_objs == 8:
        max_time = 2400  # 300*n_objs
    elif n_objs == 1:
        max_time = 300
    else:
        raise NotImplementedError

    hpn = load_data('hpn', n_objs)
    # greedy = load_data('greedy', n_objs, n_data=5000)
    greedy_helps_goal = load_data('greedy_num_goals', n_objs, n_data=5000)
    # noh = load_data('greedy_no_h', n_objs)
    """

    print "=="
    print "gnn"
    print_plan_time(greedy, max_time)
    print "=="
    """
    print "hpn"
    print_plan_time(hpn, max_time)
    print '=='
    print "gnn num goals"
    print_plan_time(greedy_helps_goal, max_time)
    print "=="
    print "no gnn"
    nognn = load_data('greedy_no_gnn_num_goals', n_objs)
    print_plan_time(nognn, max_time)
    print "=="


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

    hpn = load_data('hpn', n_objs)

    hpn_times = np.array(hpn['times'])
    hpn_successes = np.array(hpn['successes'])
    hpn_successes[hpn_times > time_limit] = False
    hpn_rate = np.mean(hpn_successes)

    data_ranges = [50, 100, 1000, 3000, 4000, 5000]
    rates = []
    for n_data in data_ranges:
        greedy = load_data('greedy', 1, n_data)
        greedy_times = np.array(greedy['times'])
        greedy_successes = np.array(greedy['successes'])
        overlimit_idxs = greedy_times > time_limit
        greedy_successes[overlimit_idxs] = False
        rates.append(np.mean(greedy_successes))

    plt.plot(data_ranges, [hpn_rate] * len(data_ranges), label='HPN', color=[0, 0, 1], marker='o')
    plt.plot(data_ranges, rates, label='GreedyQ', color=[1, 0, 0], marker='o')
    plt.xticks(data_ranges)
    savefig("Number of training data", "Success rates within 300s", './plotters/learning_curve.png')



def get_sorted_pidxs_and_plan_times_sorted_according_to_pidxs(stat, max_time):
    plan_times = {}
    nfail = 0
    for pidx, plantime in zip(stat['pidxs'], stat['times']):
        if pidx < 20000:
            continue
        if pidx in plan_times:
            if plantime >= max_time:
                nfail += 1
                plantime = max_time
            plan_times[pidx].append(plantime)
        else:
            plan_times[pidx] = []
    for r in range(20000, 20100):
        if r not in plan_times:
            plan_times[r] = [-100]
    return plan_times


def get_avg_time_per_pidx(stat, max_time):
    stattimes = get_sorted_pidxs_and_plan_times_sorted_according_to_pidxs(stat, max_time)
    avg_times = [np.mean(v) for v in stattimes.values()]
    CI = [np.std(v) * 1.96 / np.sqrt(len(v)) for v in stattimes.values()]
    return avg_times, CI


def plot_scatter_plot(n_objs):
    max_time = n_objs * 300

    stat = load_data('hpn', n_objs)
    hpn_times, hpn_ci = get_avg_time_per_pidx(stat, max_time)

    stat = load_data('greedy_num_goals', n_objs, n_data=5000)
    greedy_times, greedy_ci = get_avg_time_per_pidx(stat, max_time)

    pidxs = range(100)
    plt.errorbar(pidxs, hpn_times, hpn_ci, fmt='o', color='r', label='RSC')
    plt.errorbar(pidxs, greedy_times, greedy_ci, fmt='o', color='blue', label='GreedyQ')
    savefig("Problem instances", "Average times", './plotters/scatter')



def main():
    # plot_learning_curve()
    plot_success_vs_time(1)
    plot_scatter_plot(1)
    pass


if __name__ == '__main__':
    main()
