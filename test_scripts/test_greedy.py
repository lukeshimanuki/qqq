from problem_environments.mover_env import Mover
from problem_environments.one_arm_mover_env import OneArmMover
from planners.subplanners.motion_planner import BaseMotionPlanner
from manipulation.primitives.savers import DynamicEnvironmentStateSaver
from trajectory_representation.trajectory import Trajectory
from planners.greedy_search import GreedySearch

import socket
import os
import pickle
import time
import argparse
import numpy as np
import random
import collections
import tensorflow as tf

from learn.pap_gnn import PaPGNN


def create_model(config, mover):
    mconfig_type = collections.namedtuple('mconfig_type',
                                          'operator n_msg_passing n_layers num_fc_layers n_hidden no_goal_nodes top_k optimizer lr use_mse batch_size seed num_train val_portion num_test mse_weight diff_weight_msg_passing same_vertex_model weight_initializer loss')

    assert config.num_train <= 5000
    pap_mconfig = mconfig_type(
        operator='two_arm_pick_two_arm_place',
        n_msg_passing=1,
        n_layers=2,
        num_fc_layers=2,
        n_hidden=32,
        no_goal_nodes=False,

        top_k=1,
        optimizer='adam',
        lr=1e-4,
        use_mse=True,

        batch_size='32',
        seed=config.train_seed,
        num_train=config.num_train,
        val_portion=.1,
        num_test=1882,
        mse_weight=1.0,
        diff_weight_msg_passing=False,
        same_vertex_model=False,
        weight_initializer='glorot_uniform',
        loss=config.loss,
    )
    if config.domain == 'two_arm_mover':
        num_entities = 11
        n_regions = 2
    elif config.domain == 'one_arm_mover':
        num_entities = 17
        n_regions = 3
    else:
        raise NotImplementedError
    num_node_features = 10
    num_edge_features = 44
    entity_names = mover.entity_names

    with tf.variable_scope('pap'):
        pap_model = PaPGNN(num_entities, num_node_features, num_edge_features, pap_mconfig, entity_names, n_regions)
    pap_model.load_weights()

    return pap_model

def run(config):
    np.random.seed(config.pidx)
    random.seed(config.pidx)
    if config.domain == 'two_arm_mover':
        mover = Mover(config.pidx)
    elif config.domain == 'one_arm_mover':
        mover = OneArmMover(config.pidx)
    else:
        raise NotImplementedError
    np.random.seed(config.planner_seed)
    random.seed(config.planner_seed)
    mover.set_motion_planner(BaseMotionPlanner(mover, 'prm'))
    mover.seed = config.pidx

    mover.init_saver = DynamicEnvironmentStateSaver(mover.env)

    hostname = socket.gethostname()
    if hostname in {'dell-XPS-15-9560', 'phaedra', 'shakey', 'lab', 'glaucus', 'luke-laptop-1'}:
        root_dir = './'
    else:
        root_dir = '/data/public/rw/pass.port/tamp_q_results/'

    if config.dont_use_gnn:
        solution_file_dir = root_dir + '/test_results/greedy_results_on_mover_domain/' \
                            + '/domain_' + config.domain \
                            + '/n_objs_pack_' + str(config.n_objs_pack) \
                            + '/test_purpose/no_gnn/no_goal_obj_same_region/num_goals/'
    elif config.dont_use_h:
        solution_file_dir = root_dir + '/test_results/greedy_results_on_mover_domain/' \
                            + '/domain_' + config.domain \
                            + '/n_objs_pack_' + str(config.n_objs_pack) \
                            + '/test_purpose/no_h/' \
                            + '/num_train_' + str(config.num_train) + '/'
    else:
        solution_file_dir = root_dir + '/test_results/greedy_results_on_mover_domain/' \
                            + '/domain_' + config.domain \
                            + '/n_objs_pack_' + str(config.n_objs_pack) \
                            + '/test_purpose/no_goal_obj_same_region/num_goals/' \
                            + '/num_train_' + str(config.num_train) + '/'

    solution_file_name = 'pidx_' + str(config.pidx) + \
                         '_planner_seed_' + str(config.planner_seed) + \
                         '_train_seed_' + str(config.train_seed) + \
                         '_domain_' + str(config.domain) + '.pkl'
    if not os.path.isdir(solution_file_dir):
        os.makedirs(solution_file_dir)

    solution_file_name = solution_file_dir + solution_file_name

    is_problem_solved_before = os.path.isfile(solution_file_name)
    if is_problem_solved_before and not config.plan:
        with open(solution_file_name, 'rb') as f:
            trajectory = pickle.load(f)
            success = trajectory.metrics['success']
            tottime = trajectory.metrics['tottime']
    else:
        pap_model = create_model(config, mover)
        greedy_search = GreedySearch(mover, pap_model, config)
        t = time.time()

        trajectory, num_nodes = greedy_search.search()
        tottime = time.time() - t
        success = trajectory is not None
        plan_length = len(trajectory.actions) if success else 0
        if not success:
            trajectory = Trajectory(mover.seed, mover.seed)
        trajectory.states = None
        trajectory.metrics = {
            'n_objs_pack': config.n_objs_pack,
            'tottime': tottime,
            'success': success,
            'plan_length': plan_length,
            'num_nodes': num_nodes,
        }

        with open(solution_file_name, 'wb') as f:
            pickle.dump(trajectory, f)
    print 'Time: %.2f Success: %d Plan length: %d Num nodes: %d' % (tottime, success, trajectory.metrics['plan_length'],
                                                                    trajectory.metrics['num_nodes'])

    print('\n'.join(str(a.discrete_parameters.values()) for a in trajectory.actions))

    return


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Greedy planner')
    parser.add_argument('-pidx', type=int, default=0)
    parser.add_argument('-train_seed', type=int, default=0)
    parser.add_argument('-planner_seed', type=int, default=0)
    parser.add_argument('-n_objs_pack', type=int, default=1)
    parser.add_argument('-num_train', type=int, default=5000)
    parser.add_argument('-timelimit', type=float, default=600)
    parser.add_argument('-visualize_plan', action='store_true', default=False)
    parser.add_argument('-visualize_sim', action='store_true', default=False)
    parser.add_argument('-dontsimulate', action='store_true', default=False)
    parser.add_argument('-plan', action='store_true', default=False)
    parser.add_argument('-dont_use_gnn', action='store_true', default=False)
    parser.add_argument('-dont_use_h', action='store_true', default=False)
    parser.add_argument('-loss', type=str, default='largemargin')
    parser.add_argument('-domain', type=str, default='two_arm_mover')

    config = parser.parse_args()
    run(config)
