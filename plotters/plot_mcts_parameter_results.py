import pickle
import argparse
import os
import numpy as np

import matplotlib.pyplot as plt


def savefig(xlabel, ylabel, fname=''):
    plt.legend(loc='best', prop={'size': 13})
    plt.xlabel(xlabel, fontsize=14, fontweight='bold')
    plt.ylabel(ylabel, fontsize=14, fontweight='bold')
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    print 'Saving figure ', fname + '.png'
    plt.savefig(fname + '.png', dpi=100, format='png')


def get_result_dir(algo_name, dimension, obj_fcn):
    if obj_fcn != 'shekel':
        result_dir = './test_results/function_optimization/' + str(obj_fcn) + '/dim_' + str(
            dimension) + '/' + algo_name + '/'
    else:
        result_dir = './test_results/function_optimization/' + 'dim_' + str(dimension) + '/' + algo_name + '/'
        if algo_name == 'gpucb' and dimension == 10:
            result_dir = './test_results/function_optimization/' + 'dim_' + str(
                dimension) + '/' + algo_name + '/' + 'n_eval_200/'
    result_dir = './test_results/function_optimization/' + str(obj_fcn) + '/dim_' + str(
        dimension) + '/' + algo_name + '/'
    return result_dir


def get_results(algo_name, dimension, obj_fcn):
    result_dir = get_result_dir(algo_name, dimension, obj_fcn)
    search_times = []
    max_y_values = []
    time_takens = []
    for fin in os.listdir(result_dir):
        # for fin in os.listdir('./test_results//function_optimization/shekel/'+'dim_'+str(dimension)+'/gpucb/'):
        if fin.find('.pkl') == -1:
            continue
        result = pickle.load(open(result_dir + fin, 'r'))
        max_ys = np.array(result['max_ys'])
        if algo_name == 'doo':
            if obj_fcn != 'griewank':
                idxs = [0, 4, 10, 11, 12]
                optimal_epsilon_idx = np.argmax(max_ys[idxs, -1])
            else:
                optimal_epsilon_idx = np.argmax(max_ys[:, -1])
        else:
            optimal_epsilon_idx = np.argmax(max_ys[:, -1])
        max_y = max_ys[optimal_epsilon_idx, :]
        if len(max_y) < 500:
            continue
        if dimension == 2 and obj_fcn == 'shekel':
            max_y_values.append(max_y[:100])
            time_takens.append(result['time_takens'][optimal_epsilon_idx][:100])
        else:
            max_y_values.append(max_y)

            # time_takens.append(result['time_takens'][optimal_epsilon_idx])
    print 'number of functions tested ', len(max_y_values)
    return np.array(max_y_values)  # , np.array(time_takens)


def get_max_rwds_wrt_time(search_rwd_times):
    max_time = 10000
    organized_times = range(100, max_time, 100)

    all_episode_data = []
    for rwd_time in search_rwd_times:
        episode_max_rwds_wrt_organized_times = []
        for organized_time in organized_times:
            if isinstance(rwd_time, dict):
                rwd_time_temp = rwd_time['namo']
                episode_times = np.array(rwd_time_temp)[:, 0]
                episode_rwds = np.array(rwd_time_temp)[:, 2]
            else:
                episode_times = np.array(rwd_time)[:, 0]
                episode_rwds = np.array(rwd_time)[:, 2]
            idxs = episode_times < organized_time
            if np.any(idxs):
                max_rwd = np.max(episode_rwds[idxs])
            else:
                max_rwd = 0
            episode_max_rwds_wrt_organized_times.append(max_rwd)
        all_episode_data.append(episode_max_rwds_wrt_organized_times)

    return np.array(all_episode_data), organized_times


def get_max_rwds_wrt_samples(search_rwd_times):
    organized_times = range(10, 1000, 10)

    all_episode_data = []
    for rwd_time in search_rwd_times:
        episode_max_rwds_wrt_organized_times = []
        for organized_time in organized_times:
            if isinstance(rwd_time, dict):
                rwd_time_temp = rwd_time['namo']
                episode_times = np.array(rwd_time_temp)[:, 1]
                # episode_rwds = np.array(rwd_time_temp)[:, -1]
                episode_rwds = np.array(rwd_time_temp)[:, 2]
            else:
                episode_times = np.array(rwd_time)[:, 1]
                episode_rwds = np.array(rwd_time)[:, 2]
            idxs = episode_times <= organized_time
            if np.any(idxs):
                max_rwd = np.max(episode_rwds[idxs])
            else:
                max_rwd = 0
            episode_max_rwds_wrt_organized_times.append(max_rwd)
        all_episode_data.append(episode_max_rwds_wrt_organized_times)
    return np.array(all_episode_data), organized_times


def print_performance_for_widening_values_averaged_over_uct_values(widening_values, uct_values):
    for widening_value in widening_values:
        performances = []
        for uct_value in uct_values:
            fdir = './test_results/mcts_results_on_mover_domain/widening_' + str(widening_value) + '/uct_' \
                   + str(uct_value) + '/'

            max_rewards = []
            max_progresses = []
            for fin in os.listdir(fdir):
                result = pickle.load(open(fdir + fin, 'r'))
                max_progress = max(result['progress_list'])
                max_reward = max(np.array(result['search_time_to_reward'])[:, 2])
                max_rewards.append(max_reward)
                max_progresses.append(max_progress)

            performances.append(np.mean(max_rewards))

        print 'Widening %d mean performance: %.2f' % (widening_value, np.mean(performances))
        print 'Widening %d max performance: %.2f' % (widening_value, np.max(performances))


def print_performance_for_uct_values_averaged_over_widening_values(widening_values, uct_values):
    for uct_value in uct_values:
        performances = []
        for widening_value in widening_values:
            fdir = './test_results/mcts_results_on_mover_domain/widening_' + str(widening_value) + '/uct_' \
                   + str(uct_value) + '/'

            max_rewards = []
            max_progresses = []
            for fin in os.listdir(fdir):
                result = pickle.load(open(fdir + fin, 'r'))
                max_progress = max(result['progress_list'])
                max_reward = max(np.array(result['search_time_to_reward'])[:, 2])
                max_rewards.append(max_reward)
                max_progresses.append(max_progress)

            performances.append(np.mean(max_rewards))

        print 'UCT %.2f mean performance %.2f' % (uct_value, np.mean(performances))
        print 'UCT %.2f max performance %.2f' % (uct_value, np.max(performances))


def print_all_performances(widening_values, uct_values):
    for widening_value in widening_values:
        for uct_value in uct_values:
            fdir = './test_results/mcts_results_on_mover_domain/widening_' + str(widening_value) + '/uct_' \
                   + str(uct_value) + '/'

            max_rewards = []
            max_progresses = []
            for fin in os.listdir(fdir):
                result = pickle.load(open(fdir + fin, 'r'))
                max_progress = max(result['progress_list'])
                max_reward = max(np.array(result['search_time_to_reward'])[:, 2])
                max_rewards.append(max_reward)
                max_progresses.append(max_progress)

            print "UCT %.2f, Widening value %d, reward (mean,var): %.2f %.2f progress (mean,var): %.2f %.2f " \
                  % (uct_value, widening_value,
                     np.mean(max_rewards),
                     1.96 / np.sqrt(len(max_rewards)) * np.std(max_rewards),
                     np.mean(max_progresses),
                     1.96 / np.sqrt(len(max_progresses)) * np.std(max_progresses),
                     )


def plot_across_algorithms():
    parser = argparse.ArgumentParser(description='parameters')
    parser.add_argument('-uct', type=float, default=5.0)
    parser.add_argument('-widening_parameter', type=float, default=10)
    parser.add_argument('-sampling_strategy', type=str, default='unif')
    parser.add_argument('-problem_idx', type=int, default=0)
    parser.add_argument('-domain', type=str, default='mover')
    parser.add_argument('-planner', type=str, default='mcts')
    parser.add_argument('-v', action='store_true', default=False)
    parser.add_argument('-debug', action='store_true', default=False)
    parser.add_argument('-mcts_iter', type=int, default=500)
    parser.add_argument('-n_feasibility_checks', type=int, default=50)
    parser.add_argument('-random_seed', type=int, default=-1)
    args = parser.parse_args()

    widening_values = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0]
    uct_values = [0.1, 0.5, 1.0, 5.0, 10.0, 100.0]

    print 'Across different UCT values:'
    print_performance_for_uct_values_averaged_over_widening_values(widening_values, uct_values)

    print 'Across different widening values:'
    print_performance_for_widening_values_averaged_over_uct_values(widening_values, uct_values)


if __name__ == '__main__':
    plot_across_algorithms()
