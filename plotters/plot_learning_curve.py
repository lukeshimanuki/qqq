import pickle
import argparse
import os
import numpy as np

import matplotlib.pyplot as plt
import seaborn as sns


def load_data(algo, n_objs, n_data=None):
    if algo == 'hpn':
        return pickle.load(open('./plotters/stats/hpn_n_objs_' + str(n_objs) + '.pkl', 'r'))
    elif algo == 'greedy':
        return pickle.load(open('./plotters/stats/greedy_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data), 'r'))
    else:
        raise NotImplementedError


def get_success_rate_at(time_success_data, interval):
    time_success_data = np.array(time_success_data)
    result =[]
    for tlimit in interval:
        successes = []
        for tsuccess in time_success_data:
            time_taken = tsuccess[1]
            if time_taken <= tlimit:
                successes.append(tsuccess[0])
            else:
                successes.append(False)
        result.append(np.mean(successes))


        """
        idxs = time_success_data[:, 1] < tlimit

        if len(idxs) == 0:
            successes.append(False)
        else:
            successes_within_tlimit = np.sum(time_success_data[idxs, 0]) / float(len(time_success_data))
        rates.append(successes_within_tlimit)
        """
    return result


def plot_success_vs_time(n_objs):
    max_time = n_objs * 50
    interval = np.linspace(10, max_time, 500)

    hpn = load_data('hpn', n_objs)
    greedy = load_data('greedy', n_objs, n_data=5000)

    hpn_times = np.array(hpn['times'])
    greedy_times = np.array(greedy['times'])

    hpn_successes = np.array(hpn['successes'])
    greedy_successes = np.array(greedy['successes'])
    hpn_successes[hpn_times > max_time] = False
    greedy_successes[greedy_times > max_time] = False

    print np.mean(hpn_successes)
    print np.mean(greedy_successes)

    hpn_times[hpn_times>max_time] = max_time
    greedy_times[greedy_times > max_time] = max_time

    print np.mean(hpn_times)
    print np.mean(greedy_times)

    import pdb;pdb.set_trace()

    """
    hpn_success_rates = get_success_rate_at(zip(hpn['successes'], hpn['times']), interval)
    greedy_success_rates = get_success_rate_at(zip(greedy['successes'], greedy['times']), interval)

    sns.tsplot(hpn_success_rates, interval, ci=95, condition='HPN',
               color=[1,0,0])
    sns.tsplot(greedy_success_rates, interval, ci=95, condition='GreedyQ',
               color=[0, 1, 0])
    plt.show()
    import pdb;
    pdb.set_trace()
    """


def main():
    plot_success_vs_time(8)
    pass


if __name__ == '__main__':
    main()
