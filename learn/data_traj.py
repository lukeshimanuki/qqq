import os
import glob
import pickle
import pdb

import numpy as np


def extract_example(state, pick_action, place_action, pick_reward, place_reward, remaining_steps=0):
    names = sorted(list(state.nodes)[::-1], key=lambda name: 'region' in name)
    name_to_idx = {name: i for i, name in enumerate(names)}

    nodes = [state.nodes[name] for name in names]
    edges = [[state.edges[(a, b)] for b in names] for a in names]
    actions = [name_to_idx[pick_action.discrete_parameters['object']],
               name_to_idx[place_action.discrete_parameters['region'].name]]
    operators = 0
    # costs = pick_reward + place_reward
    costs = remaining_steps
    return nodes, edges, actions, operators, costs


import copy


def encode_in_one_hot(predicate_eval):
    if predicate_eval == 1:
        return [0, 1]
    else:
        return [1, 0]


def make_one_hot_encoded_node(node):
    one_hot_encoded_predicates = []
    for predicate_eval in node[6:]:
        eval = encode_in_one_hot(predicate_eval)
        one_hot_encoded_predicates.append(eval)
    one_hot_encoded = np.hstack([node[:6], np.hstack(one_hot_encoded_predicates)])
    return one_hot_encoded


def make_one_hot_encoded_edge(edge):
    one_hot_encoded_predicates = []
    for predicate_eval in edge:
        eval = encode_in_one_hot(predicate_eval)
        one_hot_encoded_predicates.append(eval)
    one_hot_encoded = np.hstack(one_hot_encoded_predicates)
    return one_hot_encoded


def get_edges(state, region_nodes, entity_names):
    # Desired output shape: n_e x n_e x n_r x n_edge

    regions = [r for r in entity_names if 'region' in r and 'entire' not in r]
    n_edge_features = 44
    n_regions = len(regions)
    n_entities = len(entity_names)
    edges = np.zeros((n_entities, n_entities, n_regions, n_edge_features))
    for aidx, a in enumerate(entity_names):
        for bidx, b in enumerate(entity_names):
            ab_binary_edge = make_one_hot_encoded_edge(state.binary_edges[(a, b)])
            ba_binary_edge = make_one_hot_encoded_edge(state.binary_edges[(b, a)])
            for ridx, r in enumerate(regions):
                ar_binary_edge = make_one_hot_encoded_edge(state.binary_edges[(a, r)]) # do we need this?
                br_binary_edge = make_one_hot_encoded_edge(state.binary_edges[(b, r)])
                ternary_edge1 = make_one_hot_encoded_edge(state.ternary_edges[(a, b, r)])
                ternary_edge2 = make_one_hot_encoded_edge(state.ternary_edges[(b, a, r)])
                edge_feature = np.hstack([region_nodes[r], ar_binary_edge, br_binary_edge, ab_binary_edge,
                                          ba_binary_edge, ternary_edge1, ternary_edge2])
                edges[aidx, bidx, ridx, :] = edge_feature
    return edges


def get_actions(op_skeleton, entity_names):
    name_to_idx = {name: i for i, name in enumerate(entity_names)}
    regions = [r for r in entity_names if 'region' in r]
    region_name_to_idx = {name: i for i, name in enumerate(regions)}

    if op_skeleton.type == 'two_arm_pick':
        object_idx = name_to_idx[op_skeleton.discrete_parameters['object']]
        action = np.array([object_idx])
    elif op_skeleton.type == 'two_arm_place':
        region_name = op_skeleton.discrete_parameters['region']
        region_idx = 0 if region_name == 'home_region' else 1
        action = np.array([region_idx])
    elif op_skeleton.type == 'two_arm_pick_two_arm_place':
        object_idx = name_to_idx[op_skeleton.discrete_parameters['two_arm_place_object']]
        region_name = op_skeleton.discrete_parameters['two_arm_place_region']
        if region_name == 'home_region':
            region_idx = 0
        elif region_name == 'loading_region':
            region_idx = 1
        else:
            raise NotImplementedError

        n_regions = 2
        n_entities = len(entity_names)
        action = np.zeros((n_entities, n_regions))
        action[object_idx, region_idx] = 1
    elif op_skeleton.type == 'one_arm_pick_one_arm_place':
        object_idx = name_to_idx[op_skeleton.discrete_parameters['object'].GetName()]
        region_name = op_skeleton.discrete_parameters['region'].name
        region_idx = region_name_to_idx[region_name]

        n_regions = len(regions)
        n_entities = len(entity_names)
        action = np.zeros((n_entities, n_regions))
        action[object_idx, region_idx] = 1
    else:
        raise NotImplementedError

    return action


def extract_individual_example(state, op_instance, remaining_steps=0):
    entity_names = list(state.nodes.keys())[::-1]
    nodes = []
    region_nodes = {}
    for name in entity_names:
        onehot = make_one_hot_encoded_node(state.nodes[name])
        nodes.append(onehot)
        if name.find('region') != -1 and name.find('entire') ==-1:
            region_nodes[name] = onehot

    nodes = np.vstack(nodes)
    edges = get_edges(state, region_nodes, entity_names)
    actions = get_actions(op_instance, entity_names)


    costs = remaining_steps
    return nodes, edges, actions, costs


def extract_file(filename, desired_operator_type='two_arm_pick'):
    traj = pickle.load(open(filename, 'rb'))

    nodes = []
    edges = []
    actions = []
    costs = []
    if len(traj.actions) == 0:
        print filename, 'was not solvable'
        return None, None, None, None

    for state, action, reward in zip(traj.states, traj.actions, traj.rewards):
        if action.type == desired_operator_type:
            node, edge, action, cost = extract_individual_example(state, action, reward)
            nodes.append(node)
            edges.append(edge)
            actions.append(action)
            costs.append(cost)
    nodes = np.stack(nodes, axis=0)
    edges = np.stack(edges, axis=0)
    actions = np.stack(actions, axis=0)
    costs = np.stack(costs, axis=0)
    return nodes, edges, actions, costs


# filename is a directory
def load_data(dirname, desired_operator_type='two_arm_pick'):
    cachefile = "{}{}.pkl".format(dirname, desired_operator_type)
    if os.path.isfile(cachefile):
        print "Loading the cached file:", cachefile
        return pickle.load(open(cachefile, 'rb'))

    print "Caching file..."
    file_list = glob.glob("{}/pap_traj_*.pkl".format(dirname))

    nodes = []
    actions = []
    costs = []
    edges = []
    for filename in file_list:
        fnodes, fedges, factions, fcosts = extract_file(filename, desired_operator_type)
        if fnodes is not None:
            nodes.append(fnodes)
            actions.append(factions)
            edges.append(fedges)
            costs.append(fcosts)

    nodes = np.vstack(nodes).squeeze()
    edges = np.vstack(edges).squeeze()
    actions = np.vstack(actions).squeeze()
    costs = np.hstack(costs).squeeze()

    data = (nodes, edges, actions, costs)
    pickle.dump(data, open(cachefile, 'wb'))
    return data


if __name__ == '__main__':
    pick_data = load_data('./test_results/hpn_results_on_mover_domain/1/trajectory_data/special_cases/',
                          desired_operator_type='two_arm_pick_two_arm_place')
    # place_data = load_data('./test_results/mcts_results_on_mover_domain/widening_5/uct_1.0/trajectory_data',
    #                       desired_operator_type='two_arm_place')
