import numpy as np
import random
import pickle
import argparse

from test_rsc import run_rsc_on_problem
from manipulation.bodies.bodies import set_color
from problem_environments.mover_env import Mover
from trajectory_representation.trajectory import Trajectory
from data_processing.process_planning_experience import process_plan_file, get_raw_dir, save_traj, get_save_dir


from mover_library.utils import visualize_path


def gather_data_from_scenario(scenario):
    mover = Mover(0)

    if scenario == 'reachable_goal_entities':
        goal_obj = mover.objects[1]
    elif scenario == 'reachable_goal_object_unreachable_goal_region':
        goal_obj = mover.objects[3]
    elif scenario == 'unreachable_goal_object_unreachable_goal_region':
        goal_obj = mover.objects[4]
    else:
        raise NotImplementedError

    run_rsc_on_problem(mover, [goal_obj.GetName()], './' + scenario + '.pkl')


def print_chosen_info(state, op):
    is_goal_idx = -3
    is_reachable_idx = -2

    discrete_param = op.discrete_parameters['object']
    discrete_param_node = state.nodes[discrete_param]
    is_reachable = discrete_param_node[is_reachable_idx]
    is_goal_entity = discrete_param_node[is_goal_idx]
    is_in_goal_region = state.edges[(discrete_param, 'home_region')][1]
    is_in_way_to_goal_region = state.edges[(discrete_param, 'home_region')][0]
    is_in_way_to_goal_obj = state.edges[(discrete_param, state.goal_entities[0])][0]

    literal = "reachable %r goal %r in_goal_region %r in_way_to_goal_R %r in_way_to_goal_obj %r" \
              % (is_reachable, is_goal_entity, is_in_goal_region, is_in_way_to_goal_region, is_in_way_to_goal_obj)

    print literal


def verify_gathered_data(scenario):
    mover = Mover(1)

    if scenario == 'reachable_goal_entities':
        goal_obj = mover.objects[1]
    elif scenario == 'reachable_goal_object_unreachable_goal_region':
        goal_obj = mover.objects[3]
    elif scenario == 'unreachable_goal_object_unreachable_goal_region':
        goal_obj = mover.objects[4]
    else:
        raise NotImplementedError

    raw_plan_dir = get_raw_dir(None)
    plan = pickle.load(open(raw_plan_dir + scenario + '.pkl', 'r'))['plan']
    mover.env.SetViewer('qtcoin')
    set_color(goal_obj, [1, 0, 0])

    save_fname = get_save_dir(None) + scenario + '.pkl'
    traj = pickle.load(open(save_fname, 'r'))
    state_traj = traj.states
    import pdb;pdb.set_trace()
    for idx, p in enumerate(plan):
        if not ('region' in p.discrete_parameters):
            import pdb;pdb.set_trace()
            state = state_traj[idx]
            print_chosen_info(state, p)

        print "%d / %d" % (idx, len(plan))
        p.execute()
        import pdb;pdb.set_trace()


def process_into_state_action_sequences(scenario):
    if scenario == 'reachable_goal_entities':
        goal_obj = 'rectangular_packing_box1'
    elif scenario == 'reachable_goal_object_unreachable_goal_region':
        goal_obj = 'rectangular_packing_box2'
    elif scenario == 'unreachable_goal_object_unreachable_goal_region':
        goal_obj = 'square_packing_box3'
    else:
        raise NotImplementedError
    goal_entities = ['home_region', goal_obj]
    raw_plan_dir = get_raw_dir(None)
    filename = raw_plan_dir + scenario + '.pkl'
    traj = process_plan_file(filename, 1, goal_entities)

    save_fname = get_save_dir(None) + scenario + '.pkl'
    save_traj(traj, save_fname)


def main():
    seed = 1
    np.random.seed(seed)
    random.seed(seed)

    parser = argparse.ArgumentParser(description='parameters')
    parser.add_argument('-mode', type=str, default='gather')
    parser.add_argument('-scenario', type=int, default=2)
    parameters = parser.parse_args()

    if parameters.mode == 'gather':
        if parameters.scenario == 1:
            gather_data_from_scenario(scenario='reachable_goal_entities')
        elif parameters.scenario == 2:
            gather_data_from_scenario(scenario='reachable_goal_object_unreachable_goal_region')
        elif parameters.scenario == 3:
            gather_data_from_scenario(scenario='unreachable_goal_object_unreachable_goal_region')
    elif parameters.mode == 'verify':
        if parameters.scenario == 1:
            verify_gathered_data(scenario='reachable_goal_entities')
        elif parameters.scenario == 2:
            verify_gathered_data(scenario='reachable_goal_object_unreachable_goal_region')
        elif parameters.scenario == 3:
            verify_gathered_data(scenario='unreachable_goal_object_unreachable_goal_region')
    elif parameters.mode == 'process':
        if parameters.scenario == 1:
            process_into_state_action_sequences(scenario='reachable_goal_entities')
        elif parameters.scenario == 2:
            process_into_state_action_sequences(scenario='reachable_goal_object_unreachable_goal_region')
        elif parameters.scenario == 3:
            process_into_state_action_sequences(scenario='unreachable_goal_object_unreachable_goal_region')


if __name__ == '__main__':
    main()
