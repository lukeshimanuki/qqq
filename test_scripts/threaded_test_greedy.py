import os
import sys
from multiprocessing.pool import ThreadPool  # dummy is nothing but multiprocessing but wrapper around threading
import argparse
import time

def worker_p(config):
    command = 'python ./planners/mover_stripstream/greedy.py'

    for key, value in zip(config.keys(), config.values()):
        if value is not None:
            option = ' -'+str(key)+' ' + str(value)
        else:
            option = ' -' + str(key)
        command += option

    print command
    os.system(command)

def worker_wrapper_multi_input(multi_args):
    #time.sleep(1)
    return worker_p(multi_args)

def main():
    parser = argparse.ArgumentParser(description='Greedy Planner parameters')
    parser.add_argument('-n_objs_pack', type=int, default=2)
    parser.add_argument('-seed', type=int, default=0)
    parser.add_argument('-train_seed', type=int, default=0)
    parser.add_argument('-time_limit', type=int, default=1000)
    parser.add_argument('-loss', type=str, default='largemargin')
    parser.add_argument('-pidxs', nargs=2, type=int, default=[0,1])
    parameters = parser.parse_args()

    pidx_begin = parameters.pidxs[0]
    pidx_end = parameters.pidxs[1]

    configs = []
    for pidx in range(pidx_begin, pidx_end):
        config = {
            'seed': pidx,
            'planner_seed': parameters.seed,
            'train_seed': parameters.train_seed,
            'num_objects': parameters.n_objs_pack,
            'timelimit': parameters.time_limit,
            'loss': parameters.loss,
            #'plan' : None,
        }
        configs.append(config)

    n_workers = 1
    print configs
    pool = ThreadPool(n_workers)
    results = pool.map(worker_wrapper_multi_input, configs)

if __name__ == '__main__':
    main()

