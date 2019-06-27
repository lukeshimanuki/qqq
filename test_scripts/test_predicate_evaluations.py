import numpy as np
import random
import pickle
import argparse

from test_rsc import run_rsc_on_problem
from manipulation.bodies.bodies import set_color
from problem_environments.mover_env import Mover
from trajectory_representation.trajectory import Trajectory
from data_processing.process_planning_experience import process_plan_file, get_raw_dir, save_traj, get_save_dir

from mover_library.utils import visualize_path, CustomStateSaver, two_arm_place_object, two_arm_pick_object


def color_reachable_entities(reachable_entities):
    for o in reachable_entities:
        set_color(o, [1, 1, 1])


def check_pick_paths(state, mover, plan):
    picks = plan[::2]
    places = plan[1::2]
    obj_to_pick = {p.discrete_parameters['object']: p for p in picks}
    obj_to_place = {p.discrete_parameters['object']: p for p in places}

    paths_to_entities = state.pick_in_way.minimum_constraint_path_to_entity
    for entity in paths_to_entities:
        path = paths_to_entities[entity]
        if len(path) == 0:
            continue
        if entity.find('region') == -1:
            set_color(mover.env.GetKinBody(entity), [0, 0, 0])

        if entity in obj_to_pick:
            data_pick = obj_to_pick[entity]
            if entity in state.is_reachable.reachable_entities:
                we_used_data_pick = np.all(data_pick.continuous_parameters['q_goal'] == path[-1])
                assert we_used_data_pick

        #visualize_path(path)
        print "Path to", entity


def colliding_obj_has_in_way_evaluate_to_true(minimum_constraint_path, state, obj, region):
    in_way_objs = minimum_constraint_path
    for obj_in_way in in_way_objs:
        key = (obj, obj_in_way, region)
        objs_in_way_had_prediate_evaluate_to_true = state.ternary_edges[key][0]
        assert objs_in_way_had_prediate_evaluate_to_true


def check_place_paths(inway, state, mover, plan):
    picks = plan[::2]
    places = plan[1::2]
    obj_to_pick = {p.discrete_parameters['object']: p for p in picks}
    obj_to_place = {p.discrete_parameters['object']: p for p in places}

    for obj_region_pair, path in zip(inway.minimum_constraint_path_to_entity.keys(),
                                     inway.minimum_constraint_path_to_entity.values()):
        obj = obj_region_pair[0]
        region = mover.regions[obj_region_pair[1]]
        print "Object region pair", obj, region
        pick_op = inway.pick_used[obj]

        if obj in obj_to_pick:
            data_pick = obj_to_pick[obj]
            we_used_data_pick = np.all(
                data_pick.continuous_parameters['q_goal'] == pick_op.continuous_parameters['q_goal'])
            assert we_used_data_pick

            data_place = obj_to_place[obj]
            same_region_as_data = region == data_place.discrete_parameters['region']

            if same_region_as_data:
                we_used_data_place_in_mc_path = np.all(data_place.continuous_parameters['q_goal'] == path[-1])
                assert we_used_data_place_in_mc_path

            colliding_obj_has_in_way_evaluate_to_true(inway.minimum_constraints_to_entity[(obj, region.name)],
                                                      state,
                                                      obj,
                                                      region.name)
        else:
            colliding_obj_has_in_way_evaluate_to_true(inway.minimum_constraints_to_entity[(obj, region.name)],
                                                      state,
                                                      obj,
                                                      region.name)


def load_plan(scenario):
    raw_plan_dir = get_raw_dir(None)
    filename = raw_plan_dir + scenario + '.pkl'
    plan = pickle.load(open(filename, 'r'))['plan']
    return plan


def test(scenario):
    mover = Mover(1)
    mover.env.SetViewer('qtcoin')
    state = pickle.load(open('node_idx_0_state.pkl', 'r'))
    reachable_objects = [mover.env.GetKinBody(o_name) for o_name in state.is_reachable.reachable_entities
                         if o_name.find('region') == -1]

    color_reachable_entities(reachable_objects)

    plan = load_plan(scenario)
    check_pick_paths(state, mover, plan)
    check_place_paths(state.place_in_way, state, mover, plan)


def main():
    seed = 1
    np.random.seed(seed)
    random.seed(seed)

    parser = argparse.ArgumentParser(description='parameters')
    parser.add_argument('-scenario', type=int, default=2)
    parameters = parser.parse_args()

    if parameters.scenario == 1:
        test(scenario='reachable_goal_entities')
    elif parameters.scenario == 2:
        test(scenario='reachable_goal_object_unreachable_goal_region')
    elif parameters.scenario == 3:
        test(scenario='unreachable_goal_object_unreachable_goal_region')


if __name__ == '__main__':
    main()
