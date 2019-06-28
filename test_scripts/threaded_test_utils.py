import argparse


def parse_options():
    parser = argparse.ArgumentParser(description='Greedy Planner parameters')
    parser.add_argument('-n_objs_pack', type=int, default=2)
    parser.add_argument('-planner_seed', type=int, default=0)
    parser.add_argument('-train_seed', type=int, default=0)
    parser.add_argument('-time_limit', type=int, default=1000)
    parser.add_argument('-loss', type=str, default='largemargin')
    parser.add_argument('-pidxs', nargs=2, type=int, default=[0, 1])
    parser.add_argument('-num_train', type=int, default=5000)
    parameters = parser.parse_args()
    return parameters


def get_configs():
    parameters = parse_options()
    pidx_begin = parameters.pidxs[0]
    pidx_end = parameters.pidxs[1]

    configs = []
    for pidx in range(pidx_begin, pidx_end):
        config = {
            'pidx': pidx,
            'planner_seed': parameters.planner_seed,
            'train_seed': parameters.train_seed,
            'num_objects': parameters.n_objs_pack,
            'timelimit': parameters.time_limit,
            'loss': parameters.loss,
            'num_train': parameters.num_train
        }
        configs.append(config)

    return configs
