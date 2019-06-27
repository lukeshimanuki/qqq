import os
import sys
from multiprocessing.pool import ThreadPool  # dummy is nothing but multiprocessing but wrapper around threading
import argparse
import time


def worker_p(config):
    command = 'python ./test_scripts/test_hpn.py'

    for key, value in zip(config.keys(), config.values()):
        option = ' -'+str(key)+' ' + str(value)
        command += option

    print command
    os.system(command)


def worker_wrapper_multi_input(multi_args):
    #time.sleep(1)
    return worker_p(multi_args)


def main():
    parser = argparse.ArgumentParser(description='MCTS parameters')
    parser.add_argument('-n_objs_pack', type=int, default=2)
    parser.add_argument('-seed', type=int, default=0)
    parser.add_argument('-time_limit', type=int, default=1000)
    parser.add_argument('-pidxs', nargs='+', type=int)
    parameters = parser.parse_args()

    pidx_begin = parameters.pidxs[0]
    pidx_end = parameters.pidxs[1]

    configs = []
    for pidx in range(pidx_begin, pidx_end):
        config = {'problem_idx': pidx,
                  'seed': parameters.seed,
                  'n_objs_pack': parameters.n_objs_pack,
                  'time_limit': parameters.time_limit}
        configs.append(config)

    n_workers = 1
    print configs
    pool = ThreadPool(n_workers)
    results = pool.map(worker_wrapper_multi_input, configs)


if __name__ == '__main__':
    main()
