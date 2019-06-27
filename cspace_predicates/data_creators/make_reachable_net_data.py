import os
import pickle
import numpy as np
from cspace_predicates.in_path_net import InPathNet

from data_preparation_utils import compute_relative_config, compute_relative_key_config, get_paths_from
from mover_library.utils import set_robot_config, release_obj, get_body_xytheta, visualize_path
from problem_environments.mover_env import Mover

konf_file = './test_results/mcts_results_on_mover_domain/widening_5/uct_1.0/key_config_data/mover_key_configs.pkl'
key_configs = pickle.load(open(konf_file, 'r'))
problem_env = Mover()
openrave_env = problem_env.env
#openrave_env.SetViewer('qtcoin')

# ReachableNet takes the relative goal config and collision information at the key configurations,
# and produces whether the goal config is reachable. It does not use the p_model output, but I could.
# So let's save the following information:
#   1) Relative cg to c0
#   2) Key config collisions
#   3) p_model prediction on key configs


def get_all_continuous_nodes(tree):
    return [n for n in tree.nodes if not n.is_operator_skeleton_node]


def get_relative_cg_label_tuple_in_node(node):
    assert not node.is_operator_skeleton_node
    cgs = []
    labels = []
    relative_cgs = []
    for op_instance, R in zip(node.reward_history.keys(), node.reward_history.values()):
        base_pose = op_instance.continuous_parameters['base_pose']
        if base_pose is None:
            continue
        cgs.append(base_pose)
        if np.max(R) == 0:
            labels.append(True)
        else:
            labels.append(False)

    if len(cgs) > 0:
        node.state_saver.Restore()
        c0s = np.array([get_body_xytheta(problem_env.robot)]*len(labels)).squeeze()
        relative_cgs = compute_relative_config(c0s, cgs)

    return relative_cgs, labels


def get_key_config_collisions_at_node(node):
    # returns one-hot-encoded collision information at node
    node.state_saver.Restore()

    not_holding = len(problem_env.robot.GetGrabbed()) == 0
    is_holding = len(problem_env.robot.GetGrabbed()) > 0
    is_place_node = node.A[0].type == 'two_arm_place'
    if is_place_node:
        assert is_holding
    else:
        assert not_holding

    collision_info = []
    for k in key_configs:
        set_robot_config(k, problem_env.robot)
        if openrave_env.CheckCollision(problem_env.robot):
            collision_info.append([0, 1])
        else:
            collision_info.append([1, 0])

    return np.array(collision_info).reshape((1, 1897, 2))


def get_paths_key_config_collisions_and_reachability_label(tree):
    continuous_nodes = get_all_continuous_nodes(tree)
    cgs = []
    labels = []
    konf_collisions = []
    dataset = []
    for n in continuous_nodes:
        node_cgs, node_labels = get_relative_cg_label_tuple_in_node(n)
        if len(node_cgs) > 0:
            node_dataset = {'konf_collisions': get_key_config_collisions_at_node(n),
                            'cgs': node_cgs,
                            'labels': node_labels}
            """
            node_konf_collisions = np.repeat(node_konf_collisions, len(node_cgs), 0)
            cgs.append(node_cgs)
            labels.append(node_labels)
            konf_collisions.append(node_konf_collisions)
            """
            dataset.append(node_dataset)
    """
    cgs = np.vstack(cgs)
    labels = np.hstack(labels)
    konf_collisions = np.vstack(konf_collisions)
    """

    return dataset


#def save_data(relative_cgs, konf_collisions, labels, fname_suffix=''):
def save_data(dataset, fname_suffix=''):
    data_dir = './test_results/mcts_results_on_mover_domain/widening_5/uct_1.0/reachability_data/'
    #pickle.dump({'relative_cgs': np.vstack(relative_cgs),
    #             'key_config_collisions': np.vstack(konf_collisions),
    #             'labels': np.hstack(labels)},
    #            open(data_dir + '_data_' + fname_suffix + '.pkl', 'wb'))

    pickle.dump(dataset, open(data_dir + '_compressed_data_' + fname_suffix + '.pkl', 'wb'))


def main():
    #p_model = InPathNet(n_hidden=2) # todo
    #p_model.load_trained_weight()

    raw_data_dir = './test_results/mcts_results_on_mover_domain/widening_5/uct_1.0/raw_data/'
    raw_data_files = os.listdir(raw_data_dir)

    dataset = []
    for idx, raw_data in enumerate(raw_data_files):
        raw_data = pickle.load(open(raw_data_dir + raw_data, 'r'))
        if 'search_tree' not in raw_data.keys():
            continue

        epsisode_dataset \
            = get_paths_key_config_collisions_and_reachability_label(raw_data['search_tree'])

        dataset = dataset + epsisode_dataset
        print "Finished %d / %d files" % (idx, len(raw_data_files))

        if idx % 100 == 0:
            save_data(epsisode_dataset, fname_suffix=str(idx))

    save_data(epsisode_dataset)


if __name__ == '__main__':
    main()

