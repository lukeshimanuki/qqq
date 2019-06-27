# this file collects data by calling a planner on whatever the environment
from problem_environments.mover_env import Mover

from planners.hierarchical_mcts import HierarchicalMCTS
import numpy as np
import random
import argparse


def main():
    parser = argparse.ArgumentParser(description='MCTS parameters')
    parser.add_argument('-uct', type=float, default=1.0)
    parser.add_argument('-widening_parameter', type=float, default=0.8)
    parser.add_argument('-sampling_strategy', type=str, default='unif')
    parser.add_argument('-problem_idx', type=int, default=0)
    parser.add_argument('-domain', type=str, default='mover')
    parser.add_argument('-planner', type=str, default='mcts')
    parser.add_argument('-v', action='store_true', default=False)
    parser.add_argument('-debug', action='store_true', default=False)
    parser.add_argument('-mcts_iter', type=int, default=50)
    parser.add_argument('-n_feasibility_checks', type=int, default=50)
    parser.add_argument('-random_seed', type=int, default=-1)
    planner_parameters = parser.parse_args()

    np.random.seed(planner_parameters.problem_idx)
    random.seed(planner_parameters.problem_idx)
    environment = Mover()

    # 1. put the robot in the truck
    # 2. generate possible operator skeletons in the state current state

    planner = HierarchicalMCTS(None, environment, 'mover', planner_parameters)
    trajectory = planner.solve()




if __name__ == '__main__':
    main()
