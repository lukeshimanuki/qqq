import pickle
import argparse
import os
import numpy as np
import copy

import matplotlib.pyplot as plt
import seaborn as sns


def load_data(algo, n_objs, n_data=0):
    if algo == 'hpn':
        return pickle.load(open('./plotters/stats/hpn_n_objs_' + str(n_objs) + '.pkl', 'r'))
    elif algo == 'greedy':
        return pickle.load(open('./plotters/stats/greedy_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data), 'r'))
    else:
        raise NotImplementedError


def get_success_rate_at(time_data, success_data, interval):
    #success_data = copy.copy(success_data)
    rates = []
    for tlimit in interval:
        rates.append(np.sum(success_data[time_data <= tlimit]) / float(len(success_data)))
    return rates


def plot_success_vs_time(n_objs):
    max_time = n_objs * 62.5
    interval = np.linspace(10, max_time, 500)

    hpn = load_data('hpn', n_objs)
    greedy = load_data('greedy', n_objs, n_data=5000)

    hpn_times = np.array(hpn['times'])
    hpn_successes = np.array(hpn['successes'])
    hpn_rates = get_success_rate_at(hpn_times, hpn_successes, interval)

    greedy_times = np.array(greedy['times'])
    greedy_successes = np.array(greedy['successes'])
    greedy_rates = get_success_rate_at(greedy_times, greedy_successes, interval)

    """
    sns.tsplot(hpn_rates, interval, condition='HPN', color=[0,1,0])
    sns.tsplot(greedy_rates, interval, condition='GreedyQ', color=[1, 0, 0])
    plt.show()
    hpn_successes[hpn_times > max_time] = False
    print np.mean(hpn_successes)
    """

    greedy_successes[greedy_times > max_time] = False
    print np.mean(greedy_successes)

    hpn_times[hpn_times > max_time] = max_time
    print np.mean(hpn_times), np.std(hpn_times) * 1.96 /np.sqrt(len(hpn_times))
    greedy_times[greedy_times > max_time] = max_time
    print np.mean(greedy_times), np.std(greedy_times) * 1.96 /np.sqrt(len(greedy_times))


    """
    sns.tsplot(rates, interval)
    sns.tsplot(greedy_rates, interval)
    plt.show()
    """


def savefig(xlabel, ylabel, fname=''):
    plt.legend(loc='best', prop={'size': 13})
    plt.xlabel(xlabel, fontsize=14, fontweight='bold')
    plt.ylabel(ylabel, fontsize=14, fontweight='bold')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    print 'Saving figure ', fname + '.png'
    plt.savefig(fname + '.png', dpi=100, format='png')


def plot_learning_curve():
    time_limit = 300
    n_objs = 1

    hpn = load_data('hpn', n_objs)

    hpn_times = np.array(hpn['times'])
    hpn_successes = np.array(hpn['successes'])
    hpn_successes[hpn_times > time_limit] = False
    hpn_rate = np.mean(hpn_successes)

    data_ranges = [100, 1000, 3000, 4000, 5000]

    rates = []
    for n_data in data_ranges:
        greedy = load_data('greedy', 1, n_data)
        greedy_times = np.array(greedy['times'])
        greedy_successes = np.array(greedy['successes'])
        overlimit_idxs = greedy_times > time_limit
        greedy_successes[overlimit_idxs] = False
        rates.append(np.mean(greedy_successes))

    plt.plot(data_ranges, [hpn_rate]*len(data_ranges), label='HPN', color=[0, 0, 1], marker='o')
    plt.plot(data_ranges, rates, label='GreedyQ', color=[1, 0, 0], marker='o')
    savefig("Number of training data", "Success rates within 300s", './plotters/learning_curve.png')


    import pdb;pdb.set_trace()



def main():
    #plot_success_vs_time(8)
    plot_learning_curve()
    pass


if __name__ == '__main__':
    main()
