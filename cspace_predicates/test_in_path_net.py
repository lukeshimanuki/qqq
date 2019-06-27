from problem_environments.mover_env import Mover
from mover_library.utils import visualize_path, get_body_xytheta, get_place_domain
from cspace_predicates.in_path_net import InPathNet

from cspace_predicates.data_creators.data_preparation_utils import compute_relative_config, c_outside_threshold

import pickle
import numpy as np
import argparse

konf_file = './test_results/mcts_results_on_mover_domain/widening_5/uct_1.0/key_config_data/mover_key_configs_in_loading_region.pkl'
key_configs = np.array(pickle.load(open(konf_file, 'r')))
n_key_configs = len(key_configs)


def sample_from_uniform(domain):
    dim_parameters = domain.shape[-1]
    domain_min = domain[0]
    domain_max = domain[1]
    return np.random.uniform(domain_min, domain_max, (1, dim_parameters)).squeeze()


def select_relevant_configs(preds):
    configs = []
    konfs_to_visualize = key_configs[preds.squeeze() > 0.5, :]
    sorted_detection = np.sort(preds.squeeze())[::-1]
    xy_threshold = 0.7  # size of the base - 0.16
    th_threshold = 35 * np.pi / 180  # adhoc
    for c, v in zip(konfs_to_visualize, sorted_detection):
        if v < 0.5: # or len(configs) > 100:
            break
        if c_outside_threshold(c, configs, xy_threshold, th_threshold):
            configs.append(c)
        else:
            continue

    if len(konfs_to_visualize) > 100:
        import pdb;pdb.set_trace()
        konfs_to_visualize =  konfs_to_visualize[::2]
    return konfs_to_visualize



def main():
    parser = argparse.ArgumentParser(description='Process configurations')
    parser.add_argument('-n_hidden', type=int, default=2)
    parser.add_argument('-seed', type=int, default=1)
    parser.add_argument('-dense_num', type=int, default=64)
    parser.add_argument('-lr', type=float, default=1e-4)
    parser.add_argument('-use_dense', action='store_true', default=False)
    parser.add_argument('-optimizer', type=str, default='adam')
    parser.add_argument('-test_portion', type=float, default=0.1)
    parser.add_argument('-n_planning_episodes', type=int, default=800)
    path_net_config = parser.parse_args()

    problem_env = Mover()
    env = problem_env.env
    env.SetViewer('qtcoin')
    robot = problem_env.robot
    in_path_net = InPathNet(path_net_config)
    in_path_net.load_trained_weight('optimizer_adam_use_dense_False_seed_1_lr_0.0001_dense_num_64_test_portion_0.1_n_planning_episodes_800_n_hidden_2_.hdf5')

    c0 = get_body_xytheta(problem_env.robot)
    domain = get_place_domain(problem_env.regions['loading_region'])

    while True:
        inp = ''
        while inp.find('y') == -1:
            cg = sample_from_uniform(domain)
            visualize_path(robot, [cg])
            inp = raw_input("Good?")

        rel_cg = compute_relative_config(c0, cg)
        rel_konfs = compute_relative_config(c0, key_configs)
        rel_cgs = np.tile(rel_cg, (len(rel_konfs), 1))

        preds = in_path_net.clf.predict([rel_cgs, rel_konfs])
        konfs_to_visualize = select_relevant_configs(preds)

        in_path_key_configs = np.vstack([konfs_to_visualize, cg])
        visualize_path(problem_env.robot, in_path_key_configs)

    """
    relative_cg = compute_relative_config(c0, cg)
    detection = in_path_net.clf.predict([relative_cg, compute_relative_config(c0, c0)])
    print detection
    """

    relative_cg = np.repeat(relative_cg, n_key_configs, 0)
    relative_konf = compute_relative_config(c0, key_configs)

    detection = in_path_net.clf.predict([relative_cg, relative_konf])
    sorted_detection = np.sort(detection.squeeze())[::-1]
    args_of_sorted_detection = np.argsort(detection.squeeze())[::-1]

    in_path_key_configs = key_configs[args_of_sorted_detection, :]
    configs = []
    xy_threshold = 0.7  # size of the base - 0.16
    th_threshold = 35 * np.pi / 180  # adhoc
    for c, v in zip(in_path_key_configs, sorted_detection):
        if v < 0.5 or len(configs) > 100:
            break
        if c_outside_threshold(c, configs, xy_threshold, th_threshold):
            configs.append(c)
        else:
            continue

    print v, len(configs)
    import pdb;pdb.set_trace()
    in_path_key_configs = np.vstack([configs, cg])
    visualize_path(problem_env.robot, in_path_key_configs)
    import pdb;pdb.set_trace()


if __name__ == '__main__':
    main()

