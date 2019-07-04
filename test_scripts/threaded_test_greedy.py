import os
from multiprocessing.pool import ThreadPool  # dummy is nothing but multiprocessing but wrapper around threading
from threaded_test_utils import get_configs
import multiprocessing


def worker_p(config):
    command = 'python ./planners/mover_stripstream/greedy.py'

    for key, value in zip(config.keys(), config.values()):
        if value is not None:
            option = ' -' + str(key) + ' ' + str(value)
        else:
            option = ' -' + str(key)
        command += option

    print command
    os.system(command)


def worker_wrapper_multi_input(multi_args):
    return worker_p(multi_args)


def main():
    configs = get_configs()
    n_workers = 1  # multiprocessing.cpu_count()
    pool = ThreadPool(n_workers)
    results = pool.map(worker_wrapper_multi_input, configs)


if __name__ == '__main__':
    main()
