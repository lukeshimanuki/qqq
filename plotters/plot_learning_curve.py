import pickle
import argparse
import os
import numpy as np

import matplotlib.pyplot as plt
import seaborn as sns


def load_data(algo, n_objs, n_data=None):
    if algo == 'hpn':
        return pickle.load(open('./plotters/stats/hpn_n_objs_' + str(n_objs) + '.pkl', 'r'))
    elif algo == 'gredy':
        return pickle.load(open('./plotters/stats/greedy_n_objs_%d_n_data_%d.pkl' % (n_objs, n_data), 'r'))

def get_success_rate_at(time_success_data, interval):
    import pdb;pdb.set_trace()
    time_success_data = np.array(time_success_data)
    for tlimit in interval:
        idxs = np.where(time_success_data[:, 1] < tlimit)
        successes_within_tlimit = np.mean(time_success_data[idxs, 0])
        import pdb;pdb.set_trace()






def plot_success_vs_time(n_objs):
    hpn = load_data('hpn', n_objs)
    max_time = n_objs*300
    interval = range(10, max_time, 100)
    success_rates = get_success_rate_at(zip(hpn['successes'], hpn['times']),interval)

    import pdb;pdb.set_trace()

def main():
    plot_success_vs_time(8)
    pass


if __name__ == '__main__':
    main()
