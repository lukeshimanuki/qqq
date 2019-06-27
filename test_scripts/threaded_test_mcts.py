import os
import sys
from multiprocessing.pool import ThreadPool  # dummy is nothing but multiprocessing but wrapper around threading
import argparse
import time
import multiprocessing


def worker_p(config):
    command = 'python ./test_scripts/test_mcts.py -use_learned_q -f'

    for key, value in zip(config.keys(), config.values()):
        option = ' -'+str(key)+' ' + str(value)
        command += option

    print command
    os.system(command)


def worker_wrapper_multi_input(multi_args):
    time.sleep(1)
    return worker_p(multi_args)


def main():
    configs = []
    for pidx in range(2020, 2100):
        config = {'problem_idx': pidx}
        configs.append(config)

    n_workers = multiprocessing.cpu_count()
    n_workers = 1
    print configs
    pool = ThreadPool(n_workers)
    results = pool.map(worker_wrapper_multi_input, configs)


if __name__ == '__main__':
    main()
