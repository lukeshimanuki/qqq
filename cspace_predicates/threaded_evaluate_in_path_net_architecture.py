import os
import sys
from multiprocessing.pool import ThreadPool  # dummy is nothing but multiprocessing but wrapper around threading
import argparse
import time


def worker_p(config):
    command = 'python -m cspace_predicates.train_in_path_net'

    for key, value in zip(config.keys(), config.values()):
        option = ' -' + str(key) + ' ' + str(value)
        command += option

    print command
    os.system(command)


def worker_wrapper_multi_input(multi_args):
    time.sleep(1)
    return worker_p(multi_args)


def main():
    parser = argparse.ArgumentParser(description='Process configurations')
    parser.add_argument('-n_hidden', nargs='+', type=int, default=[2])
    parser.add_argument('-seed', nargs='+', type=int, default=[1])
    parser.add_argument('-dense_num', nargs = '+', type=int, default=[64])
    parser.add_argument('-lr', nargs='+', type=float, default=[1e-4])
    parser.add_argument('-optimizer', nargs='+', type=str, default=['adam'])
    parser.add_argument('-test_portion', nargs='+', type=float, default=[0.1])
    parser.add_argument('-use_dense', action='store_true', default=False)
    parser.add_argument('-n_planning_episodes', type=int, default=100)

    args = parser.parse_args()
    configs = []
    for optimizer in args.optimizer:
        for lr in args.lr:
            for dense_num in args.dense_num:
                for n_hidden in args.n_hidden:
                    for random_seed in range(10):
                        configs.append({'dense_num': dense_num,
                                        'n_hidden': n_hidden,
                                        'optimizer': optimizer,
                                        'lr': lr,
                                        'n_planning_episodes': args.n_planning_episodes,
                                        'seed': random_seed})

    n_workers = int(200)
    print configs
    pool = ThreadPool(n_workers)
    results = pool.map(worker_wrapper_multi_input, configs)


if __name__ == '__main__':
    main()
