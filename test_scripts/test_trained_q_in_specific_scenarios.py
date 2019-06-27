from problem_environments.mover_env import Mover
from learn.gnn import GNN
from trajectory_representation.state import State
from planners.subplanners.motion_planner import OperatorBaseMotionPlanner
from learn import model

from manipulation.bodies.bodies import set_color, box_body
from mover_library.utils import set_obj_xytheta, get_body_xytheta, visualize_path

import numpy as np
import random
import os
import argparse
import pickle
import copy

is_goal_idx = -3
is_reachable_idx = -2


def parse_arguments():
    parser = argparse.ArgumentParser(description='Process configurations')
    parser.add_argument('-n_hidden', type=int, default=32)
    parser.add_argument('-n_layers', type=int, default=4)
    parser.add_argument('-seed', type=int, default=0)
    parser.add_argument('-lr', type=float, default=1e-4)
    parser.add_argument('-optimizer', type=str, default='adam')
    parser.add_argument('-batch_size', type=int, default=32)
    parser.add_argument('-num_test', type=int, default=1000)
    parser.add_argument('-num_train', type=int, default=12000)
    parser.add_argument('-val_portion', type=float, default=0.1)
    parser.add_argument('-top_k', type=int, default=3)
    parser.add_argument('-use_mse', action='store_true', default=False)
    parser.add_argument('-donttrain', action='store_true', default=False)
    parser.add_argument('-same_vertex_model', action='store_true', default=False)
    parser.add_argument('-diff_weight_msg_passing', action='store_true', default=False)
    parser.add_argument('-operator', type=str, default='two_arm_pick')
    parser.add_argument('-num_fc_layers', type=int, default=2)
    parser.add_argument('-no_goal_nodes', action='store_true', default=False)
    parser.add_argument('-n_msg_passing', type=int, default=1)
    parser.add_argument('-weight_initializer', type=str, default='glorot_uniform')
    parser.add_argument('-mse_weight', type=float, default=0.2)
    configs = parser.parse_args()
    return configs


def load_q_function(configs, entities):
    num_entities = len(entities)
    num_node_features = 10
    num_edge_features = 4
    configs.num_train = 708
    configs.num_test = 708

    pick_q_function = GNN(num_entities, num_node_features, num_edge_features, configs, entities)
    pick_q_function.load_weights()
    print "Using:", pick_q_function.weight_file_name
    return pick_q_function


def print_q_values(applicable_ops, state, q_fcn):
    q_values = []
    for op in applicable_ops:
        discrete_param = op.discrete_parameters['object'].GetName()
        discrete_param_node = state.nodes[discrete_param]
        is_reachable = discrete_param_node[is_reachable_idx]
        is_goal_entity = discrete_param_node[is_goal_idx]
        is_in_goal_region = state.edges[(discrete_param, 'home_region')][1]
        is_in_way_to_goal_region = state.edges[(discrete_param, 'home_region')][0]
        is_in_way_to_goal_obj = state.edges[(discrete_param, state.goal_entities[0])][0]

        literal = "reachable %r goal %r in_goal_region %r in_way_to_goal_R %r in_way_to_goal_obj %r" \
                  % (is_reachable, is_goal_entity, is_in_goal_region, is_in_way_to_goal_region, is_in_way_to_goal_obj)
        qval = q_fcn.predict(state, op)
        print literal, discrete_param, qval
        q_values.append(qval)

    return q_values


def get_state(mover, scenario):
    if scenario == 1:
        goal_obj = mover.objects[1]
    elif scenario == 2:
        goal_obj = mover.objects[3]
    elif scenario == 3:
        goal_obj = mover.objects[4]
    else:
        raise NotImplementedError
    set_color(goal_obj, [1, 0, 0])
    fstate = './scenario_' + str(scenario) + '_state.pkl'

    fstate_exists = os.path.isfile(fstate)
    if fstate_exists:
        state = pickle.load(open(fstate, 'r'))
    else:
        state = State(mover, goal_entities=[goal_obj.GetName(), 'home_region'])
        state.make_pklable()  # removing openrave files to pkl
        pickle.dump(state, open(fstate, 'wb'))
        state.make_plannable(mover)
    return state


def color_entities_according_to_q_values(ops, q_values):
    q_values = np.array(q_values).squeeze()
    argsorted = np.argsort(q_values)
    sorted_entity_ops = np.array(ops)[argsorted]
    sorted_entity_values = np.sort(ops)

    idx = 0
    prev_value = sorted_entity_values[0]
    for op, entity_value in zip(sorted_entity_ops, sorted_entity_values):
        if entity_value != prev_value:
            prev_value = entity_value
            idx += 1
        set_color(op.discrete_parameters['object'], [0, float(idx) / len(sorted_entity_values), 0])


def test_scenario(configs, scenario):
    if scenario == 'reachable_goal_entities':
        scenario = 1
    elif scenario == 'reachable_goal_object_unreachable_goal_region':
        scenario = 2
    elif scenario == 'unreachable_goal_object_unreachable_goal_region':
        scenario = 3
    else:
        raise NotImplementedError

    mover = Mover(0)
    mover.env.SetViewer('qtcoin')
    motion_planner = OperatorBaseMotionPlanner(mover, 'prm')
    mover.set_motion_planner(motion_planner)
    q_fcn = load_q_function(configs, mover.entity_names)

    applicable_ops = mover.get_applicable_ops()
    state = get_state(mover, scenario)

    q_values = print_q_values(applicable_ops, state, q_fcn)
    color_entities_according_to_q_values(applicable_ops, q_values)


def main():
    # this script tests the trained q in hypothetical yet useful NAMO scenarios
    configs = parse_arguments()
    seed = 1
    np.random.seed(seed)
    random.seed(seed)
    test_scenario(configs, scenario='unreachable_goal_object_unreachable_goal_region')
    # reachable_goal_entities(configs)
    # unreachable_goal_region()


if __name__ == '__main__':
    main()
