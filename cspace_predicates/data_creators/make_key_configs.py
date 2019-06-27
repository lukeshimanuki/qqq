import pickle
import os
import numpy as np

from data_preparation_utils import get_configs_from_paths_outside_threshold_from_configs, get_paths_from
from manipulation.regions import create_region, AARegion

from problem_environments.mover_env import Mover

from mover_library.utils import visualize_path


def get_configs_in_both_regions(paths):
    r1, r2 = make_regions_of_interest()
    configs = []
    for path in paths:
        for c in path:
            point = np.hstack((c[0:2], 0.5))
            if r1.contains_point(point) or r2.contains_point(point):
                configs.append(c)
    return configs


def get_configs_in_loading_region(paths):
    # todo refactor
    _, loading_region = make_regions_of_interest()
    configs = []
    for path in paths:
        for c in path:
            point = np.hstack((c[0:2], 0.5))
            if loading_region.contains_point(point):
                configs.append(c)
    return configs


def make_key_configs(raw_data_dir, konf_file):
    plan_files = os.listdir(raw_data_dir)
    key_configs = []

    for idx, plan_file in enumerate(plan_files):
        plan = pickle.load(open(raw_data_dir + plan_file, 'r'))['plan']
        paths = get_paths_from(plan)

        #configs_in_region = get_configs_in_both_regions(paths)
        configs_in_region = get_configs_in_loading_region(paths)
        # todo exclude the ones outside the home and loading region

        get_configs_from_paths_outside_threshold_from_configs([configs_in_region], key_configs)
        print "Number of key configs", len(key_configs)
        print "Key config generation %d / %d done" % (idx, len(plan_files))

    pickle.dump(key_configs, open(konf_file, 'wb'))
    print "Key config generation done"


def augment_data_with_konf_collisions(data_dir, konf_file_name, domain_name):
    # augment the state data with collision info
    pass


def make_regions_of_interest():
    x_extents = 3.5
    y_extents = 3.16
    home_region = AARegion('home_region',
                           ((-x_extents + x_extents / 2.0, x_extents + x_extents / 2.0), (-y_extents, y_extents)),
                           z=0.135, color=np.array((1, 1, 0, 0.25)))

    loading_region_xy = [1.8, -6.7]
    loading_region_xy_extents = [2.5, 1.85]
    loading_region = AARegion('loading_region', ((loading_region_xy[0] - loading_region_xy_extents[0],
                                                  loading_region_xy[0] + loading_region_xy_extents[0]),
                                                 (loading_region_xy[1] - loading_region_xy_extents[1],
                                                  loading_region_xy[1] + loading_region_xy_extents[1])),
                              z=0.138, color=np.array((1, 1, 0, 0.25)))
    return home_region, loading_region


def main():
    konf_dir = './test_results/mcts_results_on_mover_domain/widening_5/uct_1.0/key_config_data/'
    konf_file = 'mover_key_configs_in_non_connecting_region.pkl'
    konf_file = 'mover_key_configs_in_loading_region.pkl'
    raw_data_dir = './test_results/mcts_results_on_mover_domain/widening_5/uct_1.0/raw_data/'

    inp = 'y'
    if os.path.isfile(konf_file):
        inp = raw_input("Overwrite the existing key config?")

    if inp.find('y') == -1:
        return

    make_key_configs(raw_data_dir, konf_file)


if __name__ == '__main__':
    main()
