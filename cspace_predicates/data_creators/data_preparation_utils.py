######## Base-planning domains ########

import numpy as np


def compute_relative_config(src_config, end_config):
    src_config = np.array(src_config)
    end_config = np.array(end_config)

    if len(src_config.shape) == 1:
        src_config = src_config[None, :]

    rel_config = end_config - src_config
    neg_idxs_to_fix = rel_config[:, -1] < -np.pi
    pos_idxs_to_fix = rel_config[:, -1] > np.pi

    # making unique rel angles; keep the range to [-pi,pi]
    rel_config[neg_idxs_to_fix, -1] = rel_config[neg_idxs_to_fix, -1] + 2 * np.pi
    rel_config[pos_idxs_to_fix, -1] = rel_config[pos_idxs_to_fix, -1] - 2 * np.pi

    return rel_config


def compute_relative_key_config(src_config, key_config):
    rels = []  # outputs konfs relative to each src_config
    for src in src_config:
        rels.append(key_config - src)
    return np.array(rels)


def reduce_path_length_to_hundred(path):
    path_len = 100  # should this depend on the length of traj?
    if len(path) > path_len:
        path_idxs = np.linspace(0, len(path) - 1, path_len).astype(int)
        path_reduced = np.array(path)[path_idxs]
    else:
        path_reduced = path
    return path_reduced


def c_outside_threshold(c, configs, xy_threshold, th_threshold):
    min_dist = np.inf
    c = np.array(c)
    for cprime in configs:
        cprime = np.array(cprime)
        xy_dist = np.linalg.norm(c[0:2] - cprime[0:2])
        # th_dist = abs(c[2]-cprime[2])
        th_dist = abs(c[2] - cprime[2]) if abs(c[2] - cprime[2]) < np.pi else 2 * np.pi - abs(c[2] - cprime[2])
        assert (th_dist < np.pi)

        if xy_dist < xy_threshold and th_dist < th_threshold:
            return False

    return True


def get_configs_from_paths_outside_threshold_from_configs(paths, configs, xy_threshold=0.3, th_threshold=20*np.pi/180):
    for path in paths:
        for c in path:
            if c[-1] < 0:
                c[-1] += 2 * np.pi
            if c[-1] > 2 * np.pi:
                c[-1] -= 2 * np.pi
            if c_outside_threshold(c, configs, xy_threshold, th_threshold):
                configs.append(c)


def get_paths_from(plan):
    paths = []
    for action in plan:
        paths.append(action.low_level_motion)
    return paths



