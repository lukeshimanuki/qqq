import pickle
import os
import numpy as np
import sys
import argparse

sys.path.append('../mover_library/')
from key_config_utils import get_configs_from_paths, c_outside_threshold
from path_detectors.PathDetector import PathDetector
from data_preparation_utils import *


class DatasetCreator:
    # todo divide this class into two
    def __init__(self):

        konf_data = pickle.load(open('rrt_no_node_lim_key_configs.pkl', 'r'))
        if type(konf_data) is list:
            self.key_configs = konf_data
            konf_data = {}
            knn_dists, knn_idxs = self.get_knn_dists_and_idxs_for_each_konf(k=10)
            konf_data['key_configs'] = self.key_configs
            self.knn_dists = konf_data['knn_dists'] = knn_dists
            self.knn_idxs = konf_data['knn_idxs'] = knn_idxs
            pickle.dump(konf_data, open('rrt_no_node_lim_key_configs.pkl', 'wb'))
        else:
            self.key_configs = konf_data['key_configs']
            self.knn_idxs = konf_data['knn_idxs']
            self.knn_dists = konf_data['knn_dists']

        self.n_konf = len(self.key_configs)
        self.n_data_per_c0cg_pair = 100

    def get_konf_path_label(self, path):
        xy_threshold = 0.3  # size of the base - 0.16
        th_threshold = 20 * np.pi / 180  # adhoc

        labels = []
        for k in self.key_configs:
            label = 0
            for c in path:
                if not c_outside_threshold(c, [k], xy_threshold, th_threshold):
                    label = 1
                    break
            labels.append(label)
        return np.array(labels)

    def select_konfs_and_labels(self, konf_path_label):
        pos = np.array(self.key_configs)[konf_path_label == 1, :]
        neg = np.array(self.key_configs)[konf_path_label == 0, :]

        labels = np.vstack([np.ones((len(pos), 1)),
                            np.zeros((self.n_data_per_c0cg_pair, 1))])

        # subsample negative konfs, because there are too many
        neg_idxs = np.random.randint(0, len(neg), self.n_data_per_c0cg_pair)
        konfs = np.vstack([pos, neg[neg_idxs, :]])
        return konfs, labels

    def get_konf_data(self, path):
        konf_path_label = self.get_konf_path_label(path)
        konfs, konf_labels = self.select_konfs_and_labels(konf_path_label)
        return np.array(konfs), np.array(konf_labels)

    def make_path_detector_dataset(self):
        cgs = []
        c0s = []
        konfs = []
        paths = []
        konf_labels = []
        path_labels = []

        relative_konfs = []
        relative_paths = []
        relative_cgs = []
        konf_collisions = []

        n_data = 0
        for dataf in os.listdir('./data/'):
            if dataf.find('rrt') == -1: continue

            data = pickle.load(open('./data/' + dataf, 'r'))
            for c0, cg, label, path in zip(data['c0'], data['cg'], data['label'], data['path']):
                if label == False:
                    continue

                path_reduced = reduce_path(path)
                path_label = get_path_label(path_reduced)
                konf, konf_label = self.get_konf_data(path_reduced)

                c0 = np.array(c0).reshape((1, 3))
                cg = np.array(cg).reshape((1, 3))

                c0s.append(c0)
                cgs.append(cg)

                konfs.append(konf)
                konf_labels.append(konf_label)
                paths.append(path_reduced)
                path_labels.append(path_label)
                konf_collisions.append(data['konf'])

                relative_cg = compute_relative_config(c0, cg)
                relative_konf = compute_relative_config(c0, konf)
                relative_path = compute_relative_config(c0, path_reduced)

                relative_cgs.append(relative_cg)
                relative_konfs.append(relative_konf)
                relative_paths.append(relative_path)
                """
                # Debugging purpose
                active_konfs = konf[(konf_label==1).squeeze(),:]
                active_rel_konfs = relative_konf[(konf_label==1).squeeze(),:]
                # TODO Write a test function for relative configs
                import pdb;pdb.set_trace()
                """
            print 'Finished a file ', len(c0s)
            if len(c0s) > 5000:
                break
        relative_cgs = np.vstack(relative_cgs)
        pickle.dump({'rel_cgs': relative_cgs,
                     'rel_paths': relative_paths,
                     'rel_konfs': relative_konfs,
                     'konf_collisions': konf_collisions,
                     'konf_labels': konf_labels,
                     'paths': paths,
                     'path_labels': path_labels},
                    open('./path_detector_data/data.pkl', 'wb'))

    def make_reachability_dataset(self):
        c0s = []
        cgs = []
        labels = []
        konfs = []
        k_vals = []
        rel_konfs = []

        p_model = PathDetector(n_hidden=2)
        p_model.load_trained_weight()
        for dataf in os.listdir('./data/'):
            if dataf.find('rrt') == -1:
                continue

            data = pickle.load(open('./data/' + dataf, 'r'))

            dataf_c0 = np.array(data['c0'])
            dataf_cg = np.array(data['cg'])
            dataf_konf = np.array(data['konf'])
            labels.append(np.array(data['label']).squeeze())
            c0s.append(dataf_c0)
            cgs.append(dataf_cg)

            rel_cg = compute_relative_config(dataf_c0, dataf_cg)
            rel_konf = compute_relative_key_config(dataf_c0, self.key_configs)

            # Make path_detection dataset, called path_detection, without collision info - i.e. assume no collision at all
            # Set no collision at everywhere except at path_detection > 0.5. Call this p_detected_collision
            # Predict reachability from path_detection * p_detected_collision

            dataf_kval = []
            for each_cg, each_rel_konf in zip(rel_cg, rel_konf):
                each_cg = np.tile(each_cg, (self.n_konf, 1))
                each_kval = p_model.clf.predict([each_cg, each_rel_konf, dataf_konf.squeeze()])
                dataf_kval.append(each_kval)

            k_vals.append(dataf_kval)
            konfs.append(np.tile(data['konf'], (len(data['c0']), 1, 1)))
            rel_konfs.append(rel_konf)

            print 'Finished a file ', len(c0s)
        c0s = np.vstack(c0s)
        cgs = np.vstack(cgs)
        labels = np.hstack(labels)
        konfs = np.vstack(konfs)
        k_vals = np.vstack(k_vals)
        rel_konfs = np.vstack(rel_konfs)
        pickle.dump({'c0s': c0s, 'cgs': cgs, 'konfs': konfs, 'labels': labels,
                     'k_vals': k_vals, 'rel_konfs': rel_konfs},
                     open('./reachability_data/data.pkl', 'wb'))

    def compute_konf_relevance(self, rel_cg, rel_konf):
        dataf_kval = []
        for each_cg, each_rel_konf in zip(rel_cg, rel_konf):
            each_cg = np.tile(each_cg, (self.n_konf, 1))
            each_kval = self.p_model.clf.predict([each_cg, each_rel_konf])
            dataf_kval.append(each_kval)
        return dataf_kval

    def get_knn_dists_and_idxs_for_each_konf(self, k=10):
        knn_idxs = []
        knn_dists = []
        for konf in self.key_configs:
            dists = [dist_between_configs(konf, konf2) for konf2 in self.key_configs]
            knn_dists.append(np.sort(dists)[0:k])
            knn_idxs.append(np.argsort(dists)[0:k])
        return knn_dists, knn_idxs

    def augment_knn_data(self, original_data):
        # augments the original data with the knn data;
        augmented = []
        for data_point in original_data:
            augmented.append(np.array(data_point[self.knn_idxs]).squeeze())

        for i in range(len(augmented)):
            for j in range(10):
                assert np.all(augmented[i][j] == original_data[i][self.knn_idxs[j]].squeeze())
        return np.array(augmented)

    def combine_relevance_and_collision(self, relevance, collision):
        combined = [np.hstack((r, z)) for r, z in zip(relevance, collision)]
        return combined

    def make_reachability_dataset_with_knn(self):
        labels = []
        konf_collision = []
        konf_relevance = []
        konf_combined = []

        self.p_model = PathDetector(n_hidden=2)
        self.p_model.load_trained_weight()

        for dataf in os.listdir('./data/'):
            if dataf.find('rrt') == -1: continue

            data = pickle.load(open('./data/' + dataf, 'r'))

            dataf_c0 = np.array(data['c0'])
            dataf_cg = np.array(data['cg'])

            rel_cg = compute_relative_config(dataf_c0, dataf_cg)
            rel_konf = compute_relative_key_config(dataf_c0, self.key_configs)

            dataf_konf_relevance = self.compute_konf_relevance(rel_cg, rel_konf)
            dataf_konf_collision = np.tile(data['konf'], (len(data['c0']), 1, 1))

            dataf_combined = self.combine_relevance_and_collision( \
                dataf_konf_relevance, dataf_konf_collision)

            knn_combined = self.augment_knn_data( \
                dataf_combined).reshape((len(data['c0']), self.n_konf, 30))

            konf_combined.append(knn_combined)
            labels.append(np.array(data['label']).squeeze())

            print len(labels)
        konf_combined = np.vstack(konf_combined)
        labels = np.hstack(labels)
        pickle.dump({'konf_combined': konf_combined, 'labels': labels}, \
                    open('./reachability_data/knn_data.pkl', 'wb'))

    def make_path_detector_dataset_with_collision_at_detected_path(self):
        c0s = []
        cgs = []
        labels = []
        konfs = []
        k_vals = []
        rel_konfs = []

        rel_cgs = []

        p_model = PathDetector(n_hidden=2)
        p_model.load_trained_weight()
        for dataf in os.listdir('./data/'):
            if dataf.find('rrt') == -1:
                continue

            data = pickle.load(open('./data/' + dataf, 'r'))

            dataf_c0 = np.array(data['c0'])
            dataf_cg = np.array(data['cg'])
            dataf_key_config_collisions = np.array(data['konf'])
            labels.append(np.array(data['label']).squeeze())

            c0s.append(dataf_c0)
            cgs.append(dataf_cg)
            rel_cg = compute_relative_config(dataf_c0, dataf_cg)
            rel_konf = compute_relative_key_config(dataf_c0, self.key_configs)

            # Predict path_detection without collision (i.e. set no collision everywhere)
            # Give collision information at path_detection > 0.5. Call this p_detected_collision
            # Predict reachability from path_detection * p_detected_collision

            # Alternatively, in case the threshold 0.5 might not work well, we just save path_detection prediction
            # and the collision information. During the training of reachability network, we can give
            # test_prediction * collision[test_prediction>threshold] as an input

            # So in this script, just save path_detection with collision=0 everywhere

            no_collision_at_key_config = np.zeros(dataf_key_config_collisions.squeeze().shape)
            no_collision_at_key_config[:, 0] = 1  # no collilsion
            dataf_kval = []
            i = 0
            for each_cg, each_rel_konf in zip(rel_cg, rel_konf):
                each_cg = np.tile(each_cg, (self.n_konf, 1))
                each_kval = p_model.clf.predict([each_cg, each_rel_konf, no_collision_at_key_config])
                dataf_kval.append(each_kval)
                i += 1

            k_vals.append(dataf_kval)
            konfs.append(np.tile(data['konf'], (len(data['c0']), 1, 1)))
            rel_konfs.append(rel_konf)
            rel_cgs.append(rel_cg)

            print 'Finished a file %d/%d' % (len(c0s), len(os.listdir('./data/')))

        c0s = np.vstack(c0s)
        cgs = np.vstack(cgs)
        labels = np.hstack(labels)
        konfs = np.vstack(konfs)
        k_vals = np.vstack(k_vals)
        rel_konfs = np.vstack(rel_konfs)
        pickle.dump({'rel_cgs': rel_cgs, 'labels': labels, 'path_detection': k_vals, 'key_config_collisions': konfs},
                    open('./reachability_data/no_collision_data.pkl', 'wb'))


def get_train_test_data(data):
    n_train = len(data['cg']) - 100
    data_idxs = np.random.permutation(range(len(data['cg'])))

    train_idxs = data_idxs[0:n_train]
    test_idxs = data_idxs[n_train:]
    c0s = np.array(data['c0'])[train_idxs, :]
    cgs = np.array(data['cg'])[train_idxs, :]
    labels = np.array(data['labels'])[train_idxs]
    konfs = np.array(data['konfs']).squeeze()[train_idxs, :]

    test_c0s = np.array(data['c0'])[test_idxs, :]
    test_cgs = np.array(data['cg'])[test_idxs, :]
    test_labels = np.array(data['labels'])[test_idxs]
    test_konfs = np.array(data['konfs']).squeeze()[test_idxs, :]

    return c0s, cgs, konfs, labels, test_c0s, test_cgs, test_konfs, test_labels


def preprocess_data(data):
    c0s = np.array(data['c0'])[train_idxs, :]
    cgs = np.array(data['cg'])[train_idxs, :]
    labels = np.array(data['labels'])[train_idxs]
    konfs = np.array(data['konfs']).squeeze()[train_idxs, :]


def main():
    """
    parser = argparse.ArgumentParser(description='Process configurations')
    parser.add_argument('-dataset_type', type=int, default=64)
    parser.add_argument('-problem', type=str, default='dynamic_wall')
    parser.add_argument('-normalize', action='store_true', default=False)
    args = parser.parse_args()

    if args.normalize and args.problem == 'dynamic_wall':
        normalize_configurations('./dynamic_wall_data/')
    """

    creator = DatasetCreator()
    if sys.argv[1] == 'p':
        creator.make_path_detector_dataset()
    elif sys.argv[1] == 'r':
        creator.make_reachability_dataset()
    elif sys.argv[1] == 'p_with_path_detected_collision':
        creator.make_path_detector_dataset_with_collision_at_detected_path()
    else:
        print "Wrong dataset name"


if __name__ == '__main__':
    main()
