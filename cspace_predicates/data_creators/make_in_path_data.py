import os
import pickle
import numpy as np

from data_preparation_utils import reduce_path_length_to_hundred, c_outside_threshold, compute_relative_config, \
    get_paths_from
from problem_environments.mover_env import Mover
from mover_library.utils import visualize_path

konf_file = './test_results/mcts_results_on_mover_domain/widening_5/uct_1.0/key_config_data/mover_key_configs_in_loading_region.pkl'
key_configs = pickle.load(open(konf_file, 'r'))

#penv = Mover()
#penv.env.SetViewer('qtcoin')


def get_path_label(path):
    # returns the label of the path computed by the planner
    n_label = len(path)
    return np.ones((n_label, 1))


def get_konf_path_label(path):
    xy_threshold = 0.3  # size of the base - 0.16
    th_threshold = 20 * np.pi / 180  # adhoc

    labels = []
    for k in key_configs:
        label = 0
        for c in path:
            if not c_outside_threshold(c, [k], xy_threshold, th_threshold):
                label = 1
                break
        labels.append(label)
    return np.array(labels)


def subsample_neg_konf_data(konf_path_label, n_neg_konfs_per_path):
    global key_configs
    pos = np.array(key_configs)[konf_path_label == 1, :]
    neg = np.array(key_configs)[konf_path_label == 0, :]

    labels = np.vstack([np.ones((len(pos), 1)),
                        np.zeros((n_neg_konfs_per_path, 1))])

    # subsample negative konfs
    neg_idxs = np.random.randint(0, len(neg), n_neg_konfs_per_path)
    konfs = np.vstack([pos, neg[neg_idxs, :]])
    return konfs, labels


def get_konf_data_for_path(c0, path):
    path_reduced = reduce_path_length_to_hundred(path)
    konf_path_label = get_konf_path_label(path_reduced)

    n_pos_data = np.sum(konf_path_label)
    konfs, konf_labels = subsample_neg_konf_data(konf_path_label, n_pos_data+len(path))  # make sure  n_pos = n_neg
    relative_konfs = compute_relative_config(c0, konfs)
    init_config_idx = np.where(np.all(np.isclose(relative_konfs, [0, 0, 0]), axis=-1))[0]
    if len(init_config_idx) > 0:
        assert konf_labels[init_config_idx[0]] == 1
    return np.array(relative_konfs), np.array(konf_labels)


def save_data(relative_configs, relative_cgs, labels, fname_suffix=''):
    data_dir = './test_results/mcts_results_on_mover_domain/widening_5/uct_1.0/in_path_net_data/'
    pickle.dump({'relative_configs': np.vstack(relative_configs),
                 'relative_cgs': np.vstack(relative_cgs),
                 'labels': np.vstack(labels)},
                open(data_dir + 'data_' + fname_suffix + '.pkl', 'wb'))


def main():
    relative_cgs = []
    relative_configs = []
    labels = []

    raw_data_dir = './test_results/mcts_results_on_mover_domain/widening_5/uct_1.0/raw_data/'
    raw_data_files = os.listdir(raw_data_dir)
    print "Key config file name ", konf_file

    for idx, raw_data in enumerate(raw_data_files):
        plan = pickle.load(open(raw_data_dir+raw_data, 'r'))['plan']
        paths = get_paths_from(plan)
        for path in paths:
            c0 = path[0][None, :]
            cg = path[-1][None, :]

            path_label = get_path_label(path)  # we definitely include these
            relative_path = compute_relative_config(c0, path)
            relative_konf, konf_label = get_konf_data_for_path(c0, path)

            init_config_idx = np.where(np.all(np.isclose(relative_path, [0, 0, 0]), axis=-1))[0]
            assert len(init_config_idx) > 0
            assert path_label[init_config_idx[0]] == 1
            relative_config = np.vstack([relative_path, relative_konf])
            labels.append(np.vstack([path_label, konf_label]))
            relative_configs.append(relative_config)

            n_konf = len(relative_config)
            relative_cg = compute_relative_config(c0, cg)
            relative_cg = np.repeat(relative_cg, n_konf, 0)
            relative_cgs.append(relative_cg)

        print len(np.vstack(relative_cgs)), len(np.vstack(relative_configs)), len(np.vstack(labels))
        if idx % 100 == 0:
            save_data(relative_configs, relative_cgs, labels, fname_suffix=str(idx)+"_loading_region_")

        print "Finished %d / %d files" % (idx, len(raw_data_files))

    save_data(relative_configs, relative_cgs, labels)


if __name__ == "__main__":
    main()
