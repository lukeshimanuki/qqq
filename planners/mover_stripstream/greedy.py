import os
import time
import pickle
import random
import copy
import argparse
import numpy as np
import Queue
import sys
import cProfile
import pstats

from collections import deque

sys.path.extend([
    '/home/caelan/Programs/pddlstream/',
    '/home/caelan/Programs/openrave_wrapper',
    '/home/caelan/Programs/motion-planners',
])

# from pddlstream.algorithms.focused import solve_focused
# from pddlstream.algorithms.incremental import solve_incremental
# from pddlstream.language.constants import print_solution
# from pddlstream.utils import read
# from pddlstream.language.generator import from_gen_fn, from_list_fn, from_test, fn_from_constant, from_fn

from problem_environments.mover_env import Mover
from problem_environments.one_arm_mover_env import OneArmMover
from generators.PickUniform import PickWithBaseUnif
from generators.PlaceUniform import PlaceUnif
# from operator_utils.grasp_utils import solveTwoArmIKs, compute_two_arm_grasp

from trajectory_representation.operator import Operator
from trajectory_representation.pick_and_place_state import PaPState
from trajectory_representation.shortest_path_pick_and_place_state import ShortestPathPaPState
from trajectory_representation.one_arm_pap_state import OneArmPaPState
from trajectory_representation.minimum_constraint_pick_and_place_state import MinimiumConstraintPaPState
from trajectory_representation.trajectory import Trajectory

from mover_library.utils import set_robot_config, set_obj_xytheta, visualize_path, two_arm_pick_object, \
    two_arm_place_object, get_body_xytheta, grab_obj, release_obj, fold_arms

# from motion_planner import rrt_region

from openravepy import RaveSetDebugLevel, DebugLevel

import numpy as np
import tensorflow as tf
import openravepy

from manipulation.primitives.display import set_viewer_options, draw_line, draw_point
from manipulation.primitives.savers import DynamicEnvironmentStateSaver

prm_vertices, prm_edges = pickle.load(open('prm.pkl', 'rb'))
# prm_edges = [set(l) - {i} for i,l in enumerate(prm_edges)]
prm_vertices = list(prm_vertices)  # TODO: needs to be a list rather than ndarray

connected = np.array([len(s) >= 2 for s in prm_edges])
prm_indices = {tuple(v): i for i, v in enumerate(prm_vertices)}
DISABLE_COLLISIONS = False
MAX_DISTANCE = 1.0


def gen_grasp(pick_unif):
    # note generate grasp, ik solution gc, relative base conf, and absolute base transform for grasping
    def fcn(obj_name):
        pick_unif.problem_env.reset_to_init_state_stripstream()
        obj = pick_unif.problem_env.env.GetKinBody(obj_name)

        for i in range(1000):
            print "Calling gengrasp"
            action = pick_unif.predict(obj, pick_unif.problem_env.regions['entire_region'], n_iter=100)
            pick_base_pose = action['base_pose']
            grasp = action['grasp_params']
            g_config = action['g_config']
            pick_unif.problem_env.reset_to_init_state_stripstream()
            if g_config is None:
                # yield None
                continue
            print grasp, pick_base_pose
            yield [grasp, pick_base_pose]

    return fcn


def find_path(graph, start, goal, heuristic=lambda x: 0, collision=lambda x: False):
    visited = {s for s in start}
    queue = Queue.PriorityQueue()
    for s in start:
        if not collision(s):
            queue.put((heuristic(s), 0, np.random.rand(), s, [s]))
    while not queue.empty():
        _, dist, _, vertex, path = queue.get()

        for next in graph[vertex] - visited:
            visited.add(next)

            if collision(next):
                continue

            if goal(next):
                return path + [next]
            else:
                newdist = dist + np.linalg.norm(prm_vertices[vertex] - prm_vertices[next])
                queue.put((newdist + heuristic(next), newdist, np.random.rand(), next, path + [next]))
    return None


# components = []
# visited = set()
# for i0 in range(len(prm_vertices)):
# 	if i0 in visited:
# 		continue
# 	component = [i0]
# 	visited.add(i0)
# 	queue = deque([i0])
# 	while queue:
# 		i1 = queue.popleft()
# 		for i2 in prm_edges[i1]:
# 			if i2 not in visited:
# 				visited.add(i2)
# 				component.append(i2)
# 				queue.append(i2)
# 	components.append(component)
# print('Components:', [len(component) for component in components])

def enumerate_paths(graph, start, goal, heuristic=lambda x: 0):
    edges = copy.deepcopy(graph)

    for i in range(1000):
        path = find_path(edges, start, goal, heuristic)
        if path is not None:
            # randomly remove edge
            remove_idx = random.randint(0, len(path) - 2)
            edges[path[remove_idx]].remove(path[remove_idx + 1])

            yield path
        else:
            break


# stack = [(start, [start])]
# while stack:
#	 (vertex, path) = stack.pop()
#	 for next in sorted(graph[vertex] - set(path), key=heuristic)[::-1]:
#		 if next == goal:
#			 yield path + [next]
#		 else:
#			 stack.append((next, path + [next]))

def gen_grasp_traj(problem, pick_unif):
    # note generate grasp, ik solution gc, relative base conf, and absolute base transform for grasping
    def fcn(obj_name, init_conf):
        pick_unif.problem_env.reset_to_init_state_stripstream()
        obj = pick_unif.problem_env.env.GetKinBody(obj_name)

        generators = []

        for i in range(10):
            problem.reset_to_init_state_stripstream()
            print "Calling gengrasp"
            action = pick_unif.predict(obj, pick_unif.problem_env.regions['entire_region'], n_iter=50)
            pick_base_pose = action['base_pose']
            if pick_base_pose is None:
                # yield None
                print('continuing pick_base_pose')
                continue
            grasp = action['grasp_params']
            g_config = action['g_config']
            if g_config is None:
                # yield None
                print('continuing g_config')
                continue

            if grasp is None:
                print('continuing grasp')
                continue

            # try:
            #	 problem.apply_two_arm_pick_action_stripstream((pick_base_pose, grasp), obj)
            # except:
            #	 #yield None
            #	 continue

            print('found pick')

            yield grasp, pick_base_pose, np.stack((init_conf, pick_base_pose))
            continue

            # don't pick vertices that aren't connected to other things
            start = (np.linalg.norm(prm_vertices - init_conf, axis=1) + (1 - connected) * 1e6).argmin()
            goal = (np.linalg.norm(prm_vertices - pick_base_pose, axis=1) + (1 - connected) * 1e6).argmin()

            disth = lambda x: np.linalg.norm(prm_vertices[x, :2] - prm_vertices[goal, :2])

            # if not dfs(prm_edges, start, goal, disth):
            #	 continue

            generators.append(enumerate_paths(prm_edges, start, goal, disth))

            for gen in generators:
                path = next(gen)
                if path is not None:
                    yield [grasp, pick_base_pose,
                           np.concatenate([[init_conf], prm_vertices[path, :], [pick_base_pose]])]

            continue

            pick_unif.problem_env.reset_to_init_state_stripstream()
            problem.disable_objects_in_region('entire_region')
            obj.Enable(True)
            assert obj.IsEnabled()
            assert problem.env.GetKinBody('floorwalls').IsEnabled()
            assert len(problem.robot.GetGrabbed()) == 0
            assert problem.robot.IsEnabled()

            set_robot_config(init_conf, problem.robot)
            path, status = problem.get_base_motion_plan(pick_base_pose.squeeze(), 'entire_region')
            problem.enable_objects_in_region('entire_region')

            if status == "HasSolution":
                problem.reset_to_init_state_stripstream()
                print "Input", obj_name, grasp, pick_base_pose
                yield [grasp, pick_base_pose, path]
                continue
            else:
                problem.reset_to_init_state_stripstream()
                yield None
                continue

            yield [grasp, pick_base_pose, path]

    return fcn


# negated, but too lazy to change the name
# (ie returns True if not reachable)
def reachable_pred(problem):
    def fcn(q1, q2, *obj_poses):
        objs = obj_poses[::2]
        poses = obj_poses[1::2]

        problem.reset_to_init_state_stripstream()

        problem.enable_objects_in_region('entire_region')

        set_robot_config(q1, problem.robot)

        for obj_name, pose in zip(objs, poses):
            o = problem.env.GetKinBody(obj_name)
            set_obj_xytheta(pose, o)
            o.Enable(True)

        def collision_check(x):
            set_robot_config(prm_vertices[x], problem.robot)
            return problem.env.CheckCollision(problem.robot)

        start = (np.linalg.norm(prm_vertices - q1, axis=1) < .5).nonzero()[0]
        if len(start) == 0:
            start = [(np.linalg.norm(prm_vertices - q1, axis=1) + (1 - connected) * 1e6).argmin()]
        goal = (np.linalg.norm(prm_vertices - q2, axis=1) + (1 - connected) * 1e6).argmin()
        path = find_path(prm_edges, start, lambda x: x == goal,
                         lambda x: np.linalg.norm(prm_vertices[x] - prm_vertices[goal]), collision_check)
        path = None

        if path is not None:
            problem.reset_to_init_state_stripstream()

            pickle.dump([prm_vertices[p] for p in path], open('pick_path.pkl', 'wb'))
            return False
        else:
            set_robot_config(q1, problem.robot)

            path, status = problem.get_base_motion_plan(q2, 'entire_region', [1000])

            problem.reset_to_init_state_stripstream()

            if status == 'HasSolution':
                pickle.dump(path, open('pick_path.pkl', 'wb'))
                return False
            else:
                return True

    return fcn


def reachable_place_pred(problem):
    def fcn(obj, grasp, q1, q2, *obj_poses):
        objs = obj_poses[::2]
        poses = obj_poses[1::2]

        # try:
        #	 problem.apply_two_arm_pick_action_stripstream((q1, grasp), obj)
        # except:
        #	 return True

        problem.reset_to_init_state_stripstream()

        problem.enable_objects_in_region('entire_region')

        set_robot_config(q1, problem.robot)

        for obj_name, pose in zip(objs, poses):
            o = problem.env.GetKinBody(obj_name)

            if obj_name == obj:
                o.Enable(False)
                continue

            set_obj_xytheta(pose, o)
            o.Enable(True)

        def collision_check(x):
            set_robot_config(prm_vertices[x], problem.robot)
            return problem.env.CheckCollision(problem.robot)

        start = (np.linalg.norm(prm_vertices - q1, axis=1) < .5).nonzero()[0]
        if len(start) == 0:
            start = [(np.linalg.norm(prm_vertices - q1, axis=1) + (1 - connected) * 1e6).argmin()]
        goal = (np.linalg.norm(prm_vertices - q2, axis=1) + (1 - connected) * 1e6).argmin()
        path = find_path(prm_edges, start, lambda x: x == goal,
                         lambda x: np.linalg.norm(prm_vertices[x] - prm_vertices[goal]), collision_check)
        path = None

        # problem.reset_to_init_state_stripstream()
        # path, status = problem.get_base_motion_plan(q2, 'entire_region', [10, 200, 300, 400, 5000])

        # if status == 'HasSolution':
        if path is not None:
            problem.reset_to_init_state_stripstream()

            pickle.dump([prm_vertices[p] for p in path], open('place_path.pkl', 'wb'))
            return False
        else:
            set_robot_config(q1, problem.robot)

            path, status = problem.get_base_motion_plan(q2, 'entire_region', [1000])

            problem.reset_to_init_state_stripstream()

            if status == 'HasSolution':
                pickle.dump(path, open('place_path.pkl', 'wb'))
                return False
            else:
                return True

    return fcn


def reachable_region_pred(problem):
    def fcn(obj, q1, r, *obj_poses):
        objs = obj_poses[::2]
        poses = obj_poses[1::2]
        region = problem.regions[r]

        problem.reset_to_init_state_stripstream()

        problem.enable_objects_in_region('entire_region')

        oo = None

        for obj_name, pose in zip(objs, poses):
            o = problem.env.GetKinBody(obj_name)

            if obj_name == obj:
                o.Enable(False)
                oo = o
                continue

            set_obj_xytheta(pose, o)
            o.Enable(True)

        def collision_check(x):
            set_robot_config(prm_vertices[x], problem.robot)
            return problem.env.CheckCollision(problem.robot)

        start = (np.linalg.norm(prm_vertices - q1, axis=1) < .5).nonzero()[0]
        if len(start) == 0:
            start = [(np.linalg.norm(prm_vertices - q1, axis=1) + (1 - connected) * 1e6).argmin()]
        path = find_path(prm_edges, start, lambda x: region.contains_point([prm_vertices[x, 0], prm_vertices[x, 1], 1]),
                         lambda x: 0, collision_check)

        # problem.reset_to_init_state_stripstream()
        # path, status = problem.get_base_motion_plan(q2, 'entire_region', [10, 200, 300, 400, 5000])

        problem.reset_to_init_state_stripstream()

        # if status == 'HasSolution':
        if path is not None:
            pickle.dump([prm_vertices[p] for p in path], open('region_path.pkl', 'wb'))
            return False
        else:
            set_robot_config(q1, problem.robot)

            def collision(q):
                set_robot_config(q, problem.robot)
                return problem.env.CheckCollision(problem.robot)
                set_obj_xytheta(q, oo)
                return problem.env.CheckCollision(oo)

            path, status = problem.get_base_motion_plan_region(lambda q: region.contains_point([q[0], q[1], 1]),
                                                               'entire_region', [1000],
                                                               rrt=rrt_region, )  # c_fn=collision)

            problem.reset_to_init_state_stripstream()

            if status == 'HasSolution':
                pickle.dump(path, open('region_path.pkl', 'wb'))
                return False
            else:
                return True

    return fcn


def place_check_traj_collision(problem):
    def fcn(holding_obj_name, grasp, pick_base_conf, placed_obj_name, placed_obj_pose, holding_obj_place_base_pose,
            holding_obj_place_traj):
        # import pdb; pdb.set_trace()
        holding_obj = problem.env.GetKinBody(holding_obj_name)
        placed_obj = problem.env.GetKinBody(placed_obj_name)
        if grasp is None:
            return True
        try:
            problem.apply_two_arm_pick_action_stripstream((pick_base_conf, grasp), holding_obj)
        except:
            return True

        if holding_obj_name != placed_obj_name:
            # set the obstacle in place
            set_obj_xytheta(placed_obj_pose, placed_obj)
        else:
            problem.reset_to_init_state_stripstream()
            return False  # this is already checked

        if len(problem.robot.GetGrabbed()) == 0:
            pass

        # check collision
        # todo disable objects
        problem.disable_objects_in_region('entire_region')
        holding_obj.Enable(True)
        placed_obj.Enable(True)

        for p in holding_obj_place_traj:
            set_robot_config(p, problem.robot)
            if problem.env.CheckCollision(problem.robot):
                problem.reset_to_init_state_stripstream()
                problem.enable_objects_in_region('entire_region')
        # return True

        problem.enable_objects_in_region('entire_region')
        problem.reset_to_init_state_stripstream()
        return False

    return fcn


def pick_check_traj_collision(problem):
    def fcn(init_config, pick_base_conf, placed_obj_name, placed_obj_pose, traj):
        # import pdb;pdb.set_trace()
        if traj is None:
            return True

        placed_obj = problem.env.GetKinBody(placed_obj_name)
        # set the obstacle in place
        set_obj_xytheta(placed_obj_pose, placed_obj)

        # check collision
        # todo disable objects
        problem.disable_objects_in_region('entire_region')
        placed_obj.Enable(True)

        for p in traj:
            set_robot_config(p, problem.robot)
            if problem.env.CheckCollision(problem.robot):
                problem.reset_to_init_state_stripstream()
                problem.enable_objects_in_region('entire_region')
                return True

        problem.enable_objects_in_region('entire_region')
        problem.reset_to_init_state_stripstream()
        return False

    return fcn


def check_edge_collision(problem, qs, obj=None):
    # TODO(lukeshim): account for yaw wrap around
    for pose in qs:
        # for pose in np.linspace(q1, q2, int(2. * np.linalg.norm(q2 - q1) + 2)):
        # TODO(lukeshim): use np.arange to always check at a fixed distance
        set_robot_config(pose, problem.robot)
        # if (obj is None) and problem.env.CheckCollision(problem.robot):
        #	return True
        if (obj is not None) and problem.env.CheckCollision(problem.robot, obj):
            # TODO(lukeshim): consider only checking obj collisions here
            return True
    return False


def blocks_move(problem):
    def fcn(p, *qs):
        o = 'square_packing_box1'
        if DISABLE_COLLISIONS:
            return False
        if all(np.linalg.norm((p - q)[:2]) > .8 for q in qs):
            return False
        # TODO: cache bounding box to prune some collisions
        obj = problem.env.GetKinBody(o)
        # set the obstacle in place
        set_obj_xytheta(p, obj)

        # check collision
        problem.disable_objects_in_region('entire_region')
        obj.Enable(True)
        result = check_edge_collision(problem, qs, obj=obj)
        # TODO(lukeshim): no need to reset so often. Causes significant overhead
        # problem.reset_to_init_state_stripstream()
        problem.enable_objects_in_region('entire_region')
        return result

    return fcn


def blocks_place(problem):
    def fcn(p1, p2):
        return np.linalg.norm((p1 - p2)[:2]) < .3

    return fcn


def gen_edge(problem, num_neighbors=3):
    def fcn(q1):
        key = tuple(q1)
        if key in prm_indices:
            # for i in prm_edges[prm_indices[key]]:
            #	yield prm_vertices[i],
            raise RuntimeError()
        else:
            dist = np.linalg.norm(np.array(prm_vertices) - np.array(q1), axis=1)
            vertices = []
            # handles = []
            problem.disable_objects_in_region('entire_region')
            for v in sorted(range(len(prm_vertices)), key=lambda i: dist[i])[:num_neighbors]:
                q2 = prm_vertices[v]
                if check_edge_collision(problem, q1, q2):
                    continue
                vertices.append((q2,))
            # handles.append(draw_edge(problem.env, q1, prm_vertices[v], z=0.25, color=(0, 0, 0, 1)))
            # yield prm_vertices[v],
            # for i in prm_edges[v]:
            #	yield prm_vertices[i],
            # raw_input('Continue?')
            problem.enable_objects_in_region('entire_region')
            return vertices

    return fcn


def test_edge(problem):
    def fcn(q1, q2):
        return np.linalg.norm(np.array(q2) - np.array(q1)) < MAX_DISTANCE

    return fcn


def gen_pick(problem, pick_unif):
    def fcn(o):
        pick_unif.problem_env.reset_to_init_state_stripstream()
        obj = pick_unif.problem_env.env.GetKinBody(o)

        while True:
            problem.reset_to_init_state_stripstream()
            action = pick_unif.predict(obj, pick_unif.problem_env.regions['entire_region'], n_iter=50)
            pick_base_pose = action['base_pose']
            if pick_base_pose is None:
                yield None
                continue
            grasp = action['grasp_params']
            g_config = action['g_config']
            if g_config is None:
                yield None
                continue

            if grasp is None:
                yield None
                continue

            # try:
            #	 problem.apply_two_arm_pick_action_stripstream((pick_base_pose, grasp), obj)
            # except:
            #	 #yield None
            #	 continue

            yield pick_base_pose, grasp

    return fcn


def gen_place(problem, place_unif):
    def fcn(o, r):
        while True:
            problem.reset_to_init_state_stripstream()
            obj = problem.env.GetKinBody(o)
            # try:
            #	 problem.apply_two_arm_pick_action_stripstream((pick_base_pose, grasp), obj) # how do I ensure that we are in the same state in both openrave and stripstream?
            # except:
            #	 yield None
            #	 continue
            place_action = place_unif.predict(obj, problem.regions[r], n_iter=50)

            place_base_pose = place_action['base_pose']
            object_pose = place_action['object_pose']
            if place_base_pose is None or object_pose is None:
                yield None
                continue
            yield place_base_pose, object_pose

    return fcn


def fpl(q, p):
    if np.linalg.norm((q - p)[:2]) < .5:
        return False
    disp = p[:2] - q[:2]
    head = np.array([np.cos(q[2]), np.sin(q[2])])
    return np.linalg.norm(disp) < 1 and head.dot(disp) / np.linalg.norm(disp) > .5


def front_place(problem):
    def fcn(q):
        for p in prm_vertices:
            if fpl(q, p):
                yield p,
        # r = 1
        # yield np.array([
        #	q[0] + r * np.cos(q[2]),
        #	q[1] + r * np.sin(q[2]),
        #	q[2],
        # ]),
        return

    return fcn


def front_pick(problem):
    def fcn(q, p):
        # return True
        if np.linalg.norm((q - p)[:2]) < .5:
            return False
        disp = p[:2] - q[:2]
        head = np.array([np.cos(q[2]), np.sin(q[2])])
        return np.linalg.norm(disp) < 1.5 and head.dot(disp) / np.linalg.norm(disp) > .5

    return fcn


def gen_placement(problem, place_unif):
    #  todo take region as parameter
    # note generate object placement, relative base conf, absolute base conf, and the path from q1 to abs base conf
    def fcn(obj_name, grasp, pick_base_pose, region):
        # simulate pick
        generators = []
        for i in range(100):
            print 'Placement region ', region
            problem.reset_to_init_state_stripstream()
            if pick_base_pose is None or pick_base_pose.size < 3:
                # yield None
                break
            obj = problem.env.GetKinBody(obj_name)
            if grasp is None:
                break
            # try:
            #	 problem.apply_two_arm_pick_action_stripstream((pick_base_pose, grasp), obj) # how do I ensure that we are in the same state in both openrave and stripstream?
            # except:
            #	 yield None
            #	 continue
            place_action = place_unif.predict(obj, problem.regions[region], n_iter=50)

            place_base_pose = place_action['base_pose']
            object_pose = place_action['object_pose']
            if object_pose is None:
                # yield None
                continue
            object_pose = object_pose
            yield place_base_pose, object_pose, np.stack((pick_base_pose, place_base_pose))
            continue

            start = (np.linalg.norm(prm_vertices - pick_base_pose, axis=1) + (1 - connected) * 1e6).argmin()
            goal = (np.linalg.norm(prm_vertices - place_base_pose, axis=1) + (1 - connected) * 1e6).argmin()

            disth = lambda x: np.linalg.norm(prm_vertices[x, :2] - prm_vertices[goal, :2])

            generators.append(enumerate_paths(prm_edges, start, goal, disth))

            for gen in generators:
                path = next(gen)
                if path is not None:
                    yield (place_base_pose, object_pose,
                           np.concatenate([[pick_base_pose], prm_vertices[path, :], [place_base_pose]]))

            continue

            problem.disable_objects_in_region('entire_region')
            obj.Enable(True)
            assert obj.IsEnabled()
            assert problem.env.GetKinBody('floorwalls').IsEnabled()
            assert len(problem.robot.GetGrabbed()) != 0
            assert problem.robot.IsEnabled()
            path, status = problem.get_base_motion_plan(place_base_pose.squeeze(), 'entire_region')
            problem.enable_objects_in_region('entire_region')

            if status == "HasSolution":
                problem.reset_to_init_state_stripstream()
                print "Input", obj_name, grasp, pick_base_pose
                yield (place_base_pose, object_pose, path)
            else:
                problem.reset_to_init_state_stripstream()
                yield None

    return fcn


def read_pddl(filename):
    directory = os.path.dirname(os.path.abspath(__file__))
    return read(os.path.join(directory, filename))


def get_problem(mover):
    tt = time.time()

    obj_names = [obj.GetName() for obj in mover.objects]
    obj_poses = [get_body_xytheta(mover.env.GetKinBody(obj_name)).squeeze() for obj_name in obj_names]

    initial_robot_conf = get_body_xytheta(mover.robot).squeeze()

    def dfs(start, goal, collide):
        path = find_path(prm_edges, list(start), goal, collision=collide)
        if path is None:
            return path
        return path  # [-1]

    def inregion(i, r):
        q = prm_vertices[i]
        if r == 'home_region':
            return q[1] > -3
        elif r == 'loading_region':
            return q[1] < -5
        else:
            assert False

    n_objs_pack = config.n_objs_pack

    if config.domain == 'two_arm_mover':
        statecls = ShortestPathPaPState
        goal = ['home_region'] + [obj.GetName() for obj in mover.objects[:n_objs_pack]]
    elif config.domain == 'one_arm_mover':
        statecls = OneArmPaPState
        goal = ['rectangular_packing_box1_region'] + [obj.GetName() for obj in mover.objects[:n_objs_pack]]
    else:
        raise NotImplementedError

    """
    pr = cProfile.Profile()
    pr.enable()
    """
    state = statecls(mover, goal)
    """
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(30)
    pstats.Stats(pr).sort_stats('cumtime').print_stats(30)
    """

    state.make_pklable()

    from learn.data_traj import extract_individual_example
    from learn.pap_gnn import PaPGNN
    import collections
    mconfig_type = collections.namedtuple('mconfig_type',
                                          'operator n_msg_passing n_layers num_fc_layers n_hidden no_goal_nodes top_k optimizer lr use_mse batch_size seed num_train val_portion num_test mse_weight diff_weight_msg_passing same_vertex_model weight_initializer loss')
    pick_mconfig = mconfig_type(
        operator='two_arm_pick',
        n_msg_passing=1,
        n_layers=4,
        num_fc_layers=2,
        n_hidden=32,
        no_goal_nodes=False,

        top_k=3,
        optimizer='adam',
        lr=1e-4,
        use_mse=True,

        batch_size='32',
        seed=0,
        num_train=12000,
        val_portion=.1,
        num_test=600,
        mse_weight=.2,
        diff_weight_msg_passing=False,
        same_vertex_model=False,
        weight_initializer='glorot_uniform',
        loss=config.loss,
    )
    place_mconfig = mconfig_type(
        operator='two_arm_place',
        n_msg_passing=1,
        n_layers=4,
        num_fc_layers=2,
        n_hidden=32,
        no_goal_nodes=False,

        top_k=1,
        optimizer='adam',
        lr=1e-4,
        use_mse=True,

        batch_size='32',
        seed=0,
        num_train=12000,
        val_portion=.1,
        num_test=600,
        mse_weight=.2,
        diff_weight_msg_passing=False,
        same_vertex_model=False,
        weight_initializer='glorot_uniform',
        loss=config.loss,
    )

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
    elif config.domain == 'one_arm_mover':
        num_entities = 16
    else:
        raise NotImplementedError
    num_node_features = 10
    num_edge_features = 44
    entity_names = mover.entity_names

    with tf.variable_scope('pap'):
        pap_model = PaPGNN(num_entities, num_node_features, num_edge_features, pap_mconfig, entity_names)
    pap_model.load_weights()

    def heuristic(state, action):
        if action.type == 'two_arm_pick':
            o = action.discrete_parameters['object']
            return -pick_model.predict(state, action) + (2 if state.edges[(o, 'home_region')][1] else 0) - sum(
                state.edges[(i, 'home_region')][1] for i in state.nodes if i != o)
        # return -pick_model.sess.run(eval_pick_model, {pnodes: nodes, pedges: edges, paction_params: action_params}) + (2 if state.edges[(o,'home_region')][1] else 0) - sum(state.edges[(i,'home_region')][1] for i in state.nodes if i != o)
        # return -pick_model.sess.run(eval_pick_model, {pnodes: nodes, pedges: edges, paction_params: action_params}) + (2 if state.edges[(o,'home_region')][1] else 0) - sum(state.edges[(i,'home_region')][1] for i in state.nodes if i != o)
        elif action.type == 'two_arm_place':
            o = action.discrete_parameters['object']
            r = action.discrete_parameters['region'].name
            return -place_model.predict(state, action) + (2 if state.edges[(o, 'home_region')][1] else 0) - sum(
                state.edges[(i, 'home_region')][1] for i in state.nodes if i != o) - (4 if r == 'home_region' else 0)
        # return -place_model.sess.run(eval_place_model, {pnodes: nodes, pedges: edges, paction_params: action_params}) + (2 if state.edges[(o,'home_region')][1] else 0) - sum(state.edges[(i,'home_region')][1] for i in state.nodes if i != o) - (4 if r == 'home_region' else 0)
        # return -place_model.sess.run(eval_place_model, {pnodes: [nodes], pedges: [edges], paction_params: [action_params]}) + (2 if state.edges[(o,'home_region')][1] else 0) - sum(state.edges[(i,'home_region')][1] for i in state.nodes if i != o) - (4 if r == 'home_region' else 0)
        elif action.type == 'two_arm_pick_two_arm_place':
            o = action.discrete_parameters['two_arm_place_object']
            r = action.discrete_parameters['two_arm_place_region']
            nodes, edges, actions, _ = extract_individual_example(state, action)
            nodes = nodes[..., 6:]

            object_is_goal = state.nodes[o][8]
            region_is_goal = state.nodes[r][8]
            number_in_goal = sum(state.binary_edges[(i, r)][0] for i in state.nodes for r in mover.regions if
                                 i != o and state.nodes[r][8]) + int(region_is_goal)
            redundant = state.binary_edges[(o, r)][0]
            helps_goal = object_is_goal and region_is_goal and not redundant
            redundant = 0
            unhelpful = 0 #object_is_goal and not region_is_goal

            if config.dont_use_gnn:
                return 1 * redundant - number_in_goal - 2 * helps_goal + 2 * unhelpful
            elif config.dont_use_h:
                gnn_pred = -pap_model.predict_with_raw_input_format(nodes[None, ...], edges[None, ...],
                                                                    actions[None, ...])
                return gnn_pred
            else:
                gnn_pred = -pap_model.predict_with_raw_input_format(nodes[None, ...], edges[None, ...], actions[None, ...])
                return 1 * redundant - number_in_goal - 2 * helps_goal + 2 * unhelpful + gnn_pred

        elif action.type == 'one_arm_pick_one_arm_place':
            o = action.discrete_parameters['object'].GetName()
            r = action.discrete_parameters['region'].name
            nodes, edges, actions, _ = extract_individual_example(state, action)
            nodes = nodes[..., 6:]

            object_is_goal = state.nodes[o][8]
            region_is_goal = state.nodes[r][8]
            number_in_goal = sum(state.binary_edges[(i, r)][0] for i in state.nodes for r in mover.regions if
                                 i != o and r in state.nodes and state.nodes[r][8]) + int(region_is_goal)
            redundant = state.binary_edges[(o, r)][0]
            helps_goal = object_is_goal and region_is_goal and not redundant
            unhelpful = object_is_goal and not region_is_goal

            if config.dont_use_gnn:
                return 1 * redundant - number_in_goal - 2 * helps_goal + 2 * unhelpful
            elif config.dont_use_h:
                gnn_pred = -pap_model.predict_with_raw_input_format(nodes[None, ...], edges[None, ...],
                                                                    actions[None, ...])
                return gnn_pred
            else:
                gnn_pred = -pap_model.predict_with_raw_input_format(nodes[None, ...], edges[None, ...], actions[None, ...])
                return 1 * redundant - number_in_goal - 2 * helps_goal + 2 * unhelpful + gnn_pred
        else:
            raise NotImplementedError

    ps = PickWithBaseUnif(mover)
    pls = PlaceUnif(mover)

    def fpi2(q, p, o):
        if not fpi(q, p):
            return False

        # obj = mover.env.GetKinBody(o)
        # set_robot_config(q, mover.robot)
        # set_obj_xytheta(p, obj)
        # pick = ps.compute_grasp_action(obj, mover.regions['entire_region'], 100, q)
        # if pick['base_pose'] is None:
        #	return False

        return True

    def can_grasp(q, p, o):
        if not fpi2(q, p, o):
            return False

        mover.disable_objects_in_region('entire_region')
        obj = mover.env.GetKinBody(o)
        grasps = compute_two_arm_grasp(
            depth_portion=.5,
            height_portion=.5,
            theta=0,
            obj=obj,
            robot=mover.robot,
        )

        old_q = get_body_xytheta(mover.robot)
        old_p = get_body_xytheta(obj)
        set_robot_config(q, mover.robot)
        set_obj_xytheta(p, obj)

        grasp = solveTwoArmIKs(mover.env, mover.robot, obj, grasps)
        grasp = True

        set_obj_xytheta(old_p, obj)
        set_robot_config(old_q, mover.robot)

        return grasp is not None

    def fpl2(q, p, o):
        if not fpl(q, p):
            return False

        mover.enable_objects_in_region('entire_region')
        obj = mover.env.GetKinBody(o)
        old_q = get_body_xytheta(mover.robot)
        old_p = get_body_xytheta(obj)
        set_robot_config(q, mover.robot)
        set_obj_xytheta(p, obj)
        if mover.env.CheckCollision(obj):
            set_obj_xytheta(old_p, obj)
            set_robot_config(old_q, mover.robot)
        # return False
        set_obj_xytheta(old_p, obj)
        set_robot_config(old_q, mover.robot)

        return True

    def can_place(q, g, o):
        p = 0

    mover.reset_to_init_state_stripstream()
    if config.visualize_plan:
        mover.env.SetViewer('qtcoin')
        set_viewer_options(mover.env)

    depth_limit = 60

    class Node(object):
        def __init__(self, parent, action, state, reward=0):
            self.parent = parent  # parent.state is initial state
            self.action = action
            self.state = state  # resulting state
            self.reward = reward  # resulting reward

            if parent is None:
                self.depth = 1
            else:
                self.depth = parent.depth + 1

        def backtrack(self):
            node = self
            while node is not None:
                yield node
                node = node.parent

    action_queue = Queue.PriorityQueue()  # (heuristic, nan, operator skeleton, state. trajectory)
    initnode = Node(None, None, state)
    for o in mover.entity_names:
        if 'region' in o:
            continue
        # action = Operator('two_arm_pick', {'object': o})
        for r in mover.entity_names:
            if 'region' not in r or 'entire' in r:
                continue
            if config.domain == 'two_arm_mover':
                action = Operator('two_arm_pick_two_arm_place', {'two_arm_place_object': o, 'two_arm_place_region': r})
            elif config.domain == 'one_arm_mover':
                action = Operator('one_arm_pick_one_arm_place', {'object': mover.env.GetKinBody(o), 'region': mover.regions[r]})
            else:
                raise NotImplementedError

            action_queue.put((heuristic(state, action), float('nan'), action, initnode))
    # print('\n'.join(str(v) for v in sorted([
    #	(heuristic(state, Operator('two_arm_pick_two_arm_place', {'two_arm_place_object': o, 'two_arm_place_region': r})), o,r)
    #	for o in obj_names
    #	for r in ('home_region', 'loading_region')
    # ])))
    iter = 0
    while True:
        if time.time() - tt > config.timelimit:
            return None, iter

        iter += 1

        if iter > 3000:
            print('failed to find plan: iteration limit')
            return None, iter

        if action_queue.empty():
            print('failed to find plan: ran out of actions')
            return None, iter
        # break
        # print('hit depth limit')
        # depth_limit *= 2
        # continue

        _, _, action, node = action_queue.get()
        state = node.state

        # interest = 'square_packing_box1'
        # iregion = 'home_region'
        # print(state.nodes[interest][9])
        # print(state.binary_edges[(interest, iregion)][2])
        # print([state.binary_edges[(entity.GetName(), interest)][1] for entity in mover.objects])
        # print([state.ternary_edges[(interest, entity.GetName(), iregion)][0] for entity in mover.objects])

        print('\n'.join([str(parent.action.discrete_parameters.values()) for parent in list(node.backtrack())[-2::-1]]))
        print("{}".format(action.discrete_parameters.values()))

        def collide(i, ignore=set(), target='robot', holding=None):
            q = prm_vertices[i]
            if True:
                set_robot_config(q, mover.robot)
                if mover.env.CheckCollision(mover.robot) or holding is not None and mover.env.CheckCollision(holding):
                    return True
            return False

        success = False
        if node.depth >= 2 and action.type == 'two_arm_pick' and node.parent.action.discrete_parameters['object'] == \
                action.discrete_parameters['object']:  # and plan[-1][1] == r:
            print('skipping because repeat', action.discrete_parameters['object'])
            continue

        if node.depth > depth_limit:
            print('skipping because depth limit', node.action.discrete_parameters.values())

        # reset to state
        for obj_name, obj_pose in state.object_poses.items():
            set_obj_xytheta(obj_pose, mover.env.GetKinBody(obj_name))
        set_robot_config(state.robot_pose, mover.robot)

        if action.type == 'two_arm_pick':
            o = action.discrete_parameters['object']
            obj = mover.env.GetKinBody(o)

            old_q = get_body_xytheta(mover.robot)
            old_p = get_body_xytheta(obj)

            for _ in range(10):
                pick_action = None
                mover.enable_objects_in_region('entire_region')
                for _ in range(10):
                    a = ps.predict(obj, mover.regions['entire_region'], 10)
                    if a['base_pose'] is not None:
                        pick_action = a
                        break
                # set_robot_config(q, mover.robot)
                # if not mover.env.CheckCollision(mover.robot):
                #	print('free pick')
                #	pick_action = action
                #	break

                if pick_action is None:
                    print('pick_action is None')
                    set_robot_config(old_q, mover.robot)
                    set_obj_xytheta(old_p, obj)
                    continue

                start_neighbors = {
                    i for i, q in enumerate(prm_vertices)
                    if np.linalg.norm((q - old_q)[:2]) < .8
                }
                pick_neighbors = {
                    i for i, q in enumerate(prm_vertices)
                    if np.linalg.norm((q - pick_action['base_pose'])[:2]) < .8
                }
                if np.linalg.norm((pick_action['base_pose'] - old_q)[:2]) < .8:
                    pick_traj = []
                else:
                    pick_traj = dfs(start_neighbors, lambda i: i in pick_neighbors, lambda i: collide(i))

                if pick_traj is None:
                    print('pick_traj is None')
                    set_robot_config(old_q, mover.robot)
                    set_obj_xytheta(old_p, obj)
                    continue

                set_robot_config(pick_action['base_pose'], mover.robot)
                grab_obj(obj)
                newstate = statecls(mover, goal, state, action)
                newstate.make_pklable()
                release_obj()
                pick_action['path'] = [old_q] + [prm_vertices[i] for i in pick_traj] + [pick_action['base_pose']]
                action.set_continuous_parameters(pick_action)
                newnode = Node(node, action, newstate)

                success = True

                for r in ('home_region', 'loading_region'):
                    newaction = Operator('two_arm_place', {'object': o, 'region': r})
                    action_queue.put(
                        (heuristic(newstate, newaction) - 0. * newnode.depth - 0., float('nan'), newaction, newnode))

                break

        elif action.type == 'two_arm_place':
            r = action.discrete_parameters['region'].name
            o = node.action.discrete_parameters['object']
            obj = mover.env.GetKinBody(o)

            old_q = get_body_xytheta(mover.robot)
            old_p = get_body_xytheta(obj)

            for _ in range(10):
                place_action = None
                mover.enable_objects_in_region('entire_region')
                for _ in range(10):
                    grab_obj(obj)
                    a = pls.predict(obj, mover.regions[r], 10)
                    if len(mover.robot.GetGrabbed()) > 0:
                        release_obj()
                    q = a['base_pose']
                    p = a['object_pose']
                    if q is not None and p is not None:
                        # set_robot_config(q, mover.robot)
                        # set_obj_xytheta(p, obj)
                        # if not mover.env.CheckCollision(mover.robot) and not mover.env.CheckCollision(obj):
                        if True:
                            place_action = a
                            break
                set_robot_config(old_q, mover.robot)

                if place_action is None:
                    print('place_action is None')
                    set_robot_config(old_q, mover.robot)
                    set_obj_xytheta(old_p, obj)
                    continue

                # grab_obj(mover.robot, obj)
                set_obj_xytheta([1000, 1000, 0], obj)
                pick_neighbors = {
                    i for i, q in enumerate(prm_vertices)
                    if np.linalg.norm((q - old_q)[:2]) < .8
                }
                place_neighbors = {
                    i for i, q in enumerate(prm_vertices)
                    if np.linalg.norm((q - place_action['base_pose'])[:2]) < .8
                }
                if np.linalg.norm((place_action['base_pose'] - old_q)[:2]) < .8:
                    place_traj = []
                else:
                    place_traj = dfs(list(pick_neighbors), lambda i: i in place_neighbors, lambda i: collide(i))
                # release_obj(mover.robot, obj)

                if place_traj is None:
                    print('place_traj is None')
                    set_robot_config(old_q, mover.robot)
                    set_obj_xytheta(old_p, obj)
                    continue

                # newplan = copy.deepcopy(plan)
                # state.make_pklable()
                # newplan.append((o, r, pick_action['base_pose'], place_action['base_pose'], place_action['object_pose'],
                #	[get_body_xytheta(mover.robot)] + [prm_vertices[i] for i in pick_traj] + [pick_action['base_pose']],
                #	[pick_action['base_pose']] + [prm_vertices[i] for i in place_traj] + [place_action['base_pose']],
                # copy.deepcopy(state)))
                # state.make_plannable(mover)
                # print('action successful')
                # print(newplan[-1][:5])
                success = True

                set_robot_config(place_action['base_pose'], mover.robot)
                set_obj_xytheta(place_action['object_pose'], obj)
                newstate = statecls(mover, goal, node.state, action)
                newstate.make_pklable()
                place_action['path'] = [old_q] + [prm_vertices[i] for i in place_traj] + [place_action['base_pose']]
                action.set_continuous_parameters(place_action)
                newnode = Node(node, action, newstate)

                if all(mover.regions['home_region'].contains_point(
                        get_body_xytheta(mover.env.GetKinBody(o))[0].tolist()[:2] + [1]) for o in
                       obj_names[:n_objs_pack]):
                    print("found successful plan: {}".format(n_objs_pack))
                    trajectory = Trajectory(mover.seed, mover.seed)
                    plan = list(newnode.backtrack())[::-1]
                    trajectory.states = [nd.state for nd in plan]
                    trajectory.actions = [nd.action for nd in plan[1:]]
                    trajectory.rewards = [nd.reward for nd in plan[1:]]
                    trajectory.state_prime = [nd.state for nd in plan[1:]]
                    trajectory.seed = mover.seed
                    print(trajectory)
                    if len(mover.robot.GetGrabbed()) > 0:
                        release_obj()
                    return trajectory, iter

                for o in obj_names:
                    newaction = Operator('two_arm_pick', {'object': o})
                    action_queue.put(
                        (heuristic(newstate, newaction) - 1. * newnode.depth, float('nan'), newaction, newnode))

                break

        elif action.type == 'two_arm_pick_two_arm_place':
            o = action.discrete_parameters['two_arm_place_object']
            obj = mover.env.GetKinBody(o)

            old_q = get_body_xytheta(mover.robot)
            old_p = get_body_xytheta(obj)

            success = False
            for _ in range(10):
                pick_action = None
                mover.enable_objects_in_region('entire_region')
                for _ in range(10):
                    a = ps.predict(obj, mover.regions['entire_region'], 10)
                    if a['base_pose'] is not None:
                        pick_action = a
                        break
                # set_robot_config(q, mover.robot)
                # if not mover.env.CheckCollision(mover.robot):
                #	print('free pick')
                #	pick_action = action
                #	break

                if pick_action is None:
                    print('pick_action is None')
                    set_robot_config(old_q, mover.robot)
                    set_obj_xytheta(old_p, obj)
                    continue

                start_neighbors = {
                    i for i, q in enumerate(prm_vertices)
                    if np.linalg.norm((q - old_q)[:2]) < .8
                }
                pick_neighbors = {
                    i for i, q in enumerate(prm_vertices)
                    if np.linalg.norm((q - pick_action['base_pose'])[:2]) < .8
                }
                if np.linalg.norm((pick_action['base_pose'] - old_q)[:2]) < .8:
                    pick_traj = []
                else:
                    pick_traj = dfs(start_neighbors, lambda i: i in pick_neighbors, lambda i: collide(i))

                if pick_traj is None:
                    print('pick_traj is None')
                    set_robot_config(old_q, mover.robot)
                    set_obj_xytheta(old_p, obj)
                    continue

                # set_robot_config(pick_action['base_pose'], mover.robot)
                # grab_obj(obj)
                # newstate = PaPState(mover, goal, state, action)
                # newstate.make_pklable()
                # release_obj()
                pick_action['path'] = [old_q] + [prm_vertices[i] for i in pick_traj] + [pick_action['base_pose']]
                # action.set_continuous_parameters(pick_action)
                # newnode = Node(node, action, newstate)

                success = True

                # for r in ('home_region', 'loading_region'):
                #	newaction = Operator('two_arm_place', {'object': o, 'region': mover.regions[r]})
                #	action_queue.put((heuristic(newstate, newaction) - 1. * newnode.depth - 0., float('nan'), newaction, newnode))

                break

            if not success:
                continue

            r = action.discrete_parameters['two_arm_place_region']
            o = action.discrete_parameters['two_arm_place_object']
            obj = mover.env.GetKinBody(o)

            # old_q = get_body_xytheta(mover.robot)
            # old_p = get_body_xytheta(obj)

            success = False

            def reset_for_place():
                if len(mover.robot.GetGrabbed()) > 0:
                    release_obj()
                fold_arms()
                set_robot_config(old_q, mover.robot)
                set_obj_xytheta(old_p, obj)
                two_arm_pick_object(obj, pick_action)

            for _ in range(10):
                place_action = None
                mover.enable_objects_in_region('entire_region')
                for _ in range(10):
                    reset_for_place()
                    a = pls.predict(obj, mover.regions[r], 10)
                    q = a['base_pose']
                    p = a['object_pose']
                    if q is not None and p is not None:
                        # set_robot_config(q, mover.robot)
                        # set_obj_xytheta(p, obj)
                        # if not mover.env.CheckCollision(mover.robot) and not mover.env.CheckCollision(obj):
                        if True:
                            place_action = a
                            break

                # set_robot_config(old_q, mover.robot)

                if place_action is None:
                    print('place_action is None')
                    # set_robot_config(old_q, mover.robot)
                    # set_obj_xytheta(old_p, obj)
                    continue

                # grab_obj(mover.robot, obj)
                # set_obj_xytheta([1000,1000,0], obj)
                pick_neighbors = {
                    i for i, q in enumerate(prm_vertices)
                    if np.linalg.norm((q - pick_action['base_pose'])[:2]) < .8
                }
                place_neighbors = {
                    i for i, q in enumerate(prm_vertices)
                    if np.linalg.norm((q - place_action['base_pose'])[:2]) < .8
                }
                if np.linalg.norm((place_action['base_pose'] - pick_action['base_pose'])[:2]) < .8:
                    place_traj = []
                else:
                    set_robot_config(old_q, mover.robot)
                    set_obj_xytheta(old_p, obj)
                    two_arm_pick_object(obj, pick_action)
                    if mover.env.CheckCollision(mover.robot) or mover.env.CheckCollision(obj):
                        print('pick in collision')
                        two_arm_place_object(place_action)
                        continue
                    place_traj = dfs(list(pick_neighbors), lambda i: i in place_neighbors,
                                     lambda i: collide(i, holding=obj))
                    assert len(mover.robot.GetGrabbed()) > 0
                    if place_traj is not None:
                        for i in place_traj:
                            if collide(i, holding=obj):
                                print('in collision: {}'.format(i))
                                import pdb;
                                pdb.set_trace()

                            set_robot_config(prm_vertices[i], mover.robot)

                            if mover.env.CheckCollision(mover.robot):
                                import pdb;
                                pdb.set_trace()
                            for objj in mover.objects:
                                if mover.env.CheckCollision(objj):
                                    import pdb;
                                    pdb.set_trace()
                    two_arm_place_object(place_action)
                    if mover.env.CheckCollision(mover.robot) or mover.env.CheckCollision(obj):
                        print('place in collision')
                        continue
                # release_obj(mover.robot, obj)

                if place_traj is None:
                    print('place_traj is None')
                    # set_robot_config(old_q, mover.robot)
                    # set_obj_xytheta(old_p, obj)
                    continue

                # newplan = copy.deepcopy(plan)
                # state.make_pklable()
                # newplan.append((o, r, pick_action['base_pose'], place_action['base_pose'], place_action['object_pose'],
                #	[get_body_xytheta(mover.robot)] + [prm_vertices[i] for i in pick_traj] + [pick_action['base_pose']],
                #	[pick_action['base_pose']] + [prm_vertices[i] for i in place_traj] + [place_action['base_pose']],
                # copy.deepcopy(state)))
                # state.make_plannable(mover)
                # print('action successful')
                # print(newplan[-1][:5])
                success = True

                # set_robot_config(place_action['base_pose'], mover.robot)
                # set_obj_xytheta(place_action['object_pose'], obj)
                place_action['path'] = [pick_action['base_pose']] + [prm_vertices[i] for i in place_traj] + [
                    place_action['base_pose']]
                action.set_continuous_parameters((pick_action, place_action))

                set_robot_config(old_q, mover.robot)
                set_obj_xytheta(old_p, obj)
                two_arm_pick_object(obj, pick_action)
                for q in place_action['path']:
                    # if collide(i, holding=obj):
                    #	print('in collision: {}'.format(i))
                    #	import pdb; pdb.set_trace()

                    set_robot_config(q, mover.robot)

                    if mover.env.CheckCollision(mover.robot):
                        import pdb;
                        pdb.set_trace()
                    for objj in mover.objects:
                        if mover.env.CheckCollision(objj):
                            import pdb;
                            pdb.set_trace()
                two_arm_place_object(place_action)

                newstate = statecls(mover, goal, node.state, action)
                newstate.make_pklable()
                newnode = Node(node, action, newstate)

                if all(mover.regions['home_region'].contains_point(
                        get_body_xytheta(mover.env.GetKinBody(o))[0].tolist()[:2] + [1]) for o in
                       obj_names[:n_objs_pack]):
                    print("found successful plan: {}".format(n_objs_pack))
                    trajectory = Trajectory(mover.seed, mover.seed)
                    plan = list(newnode.backtrack())[::-1]
                    trajectory.states = [nd.state for nd in plan]
                    trajectory.actions = [nd.action for nd in plan[1:]]
                    trajectory.rewards = [nd.reward for nd in plan[1:]]
                    trajectory.state_prime = [nd.state for nd in plan[1:]]
                    trajectory.seed = mover.seed
                    print(trajectory)
                    # if len(mover.robot.GetGrabbed()) > 0:
                    #	release_obj()
                    return trajectory, iter

                for o in obj_names:
                    if o == action.discrete_parameters['two_arm_place_object']:
                        continue
                    for r in ('home_region', 'loading_region'):
                        newaction = Operator('two_arm_pick_two_arm_place',
                                             {'two_arm_place_object': o, 'two_arm_place_region': r})
                        action_queue.put(
                            (heuristic(newstate, newaction) - 1. * newnode.depth, float('nan'), newaction, newnode))

                break

        elif action.type == 'one_arm_pick_one_arm_place':
            obj = action.discrete_parameters['object']
            region = action.discrete_parameters['region']
            o = obj.GetName()
            r = region.name

            if (o,r) in state.nocollision_place_op:
                pick_op, place_op = node.state.nocollision_place_op[(o,r)]
                action = Operator(
                    operator_type='one_arm_pick_one_arm_place',
                    discrete_parameters={
                        'object': obj,
                        'region': mover.regions[r],
                    },
                    continuous_parameters={
                        'pick': pick_op.continuous_parameters,
                        'place': place_op.continuous_parameters,
                    }
                )
                action.execute()

                success = True

                newstate = statecls(mover, goal, node.state, action)
                newstate.make_pklable()
                newnode = Node(node, action, newstate)

                # TODO: figure out why region.contains isn't working
                if o in goal and r in goal:
                #if all(mover.regions['rectangular_packing_box1_region'].contains_point(
                #        get_body_xytheta(mover.env.GetKinBody(o))[0].tolist()[:2] + [1]) for o in
                #       obj_names[:n_objs_pack]):
                    print("found successful plan: {}".format(n_objs_pack))
                    trajectory = Trajectory(mover.seed, mover.seed)
                    plan = list(newnode.backtrack())[::-1]
                    trajectory.states = [nd.state for nd in plan]
                    for s in trajectory.states:
                        s.pap_params = None
                        s.pick_params = None
                        s.place_params = None
                        s.nocollision_pick_op = None
                        s.collision_pick_op = None
                        s.nocollision_place_op = None
                        s.collision_place_op = None
                    trajectory.actions = [nd.action for nd in plan[1:]]
                    for op in trajectory.actions:
                        op.discrete_parameters = {
                            key: value.name if 'region' in key else value.GetName()
                            for key,value in op.discrete_parameters.items()
                        }
                    trajectory.rewards = [nd.reward for nd in plan[1:]]
                    trajectory.state_prime = [nd.state for nd in plan[1:]]
                    trajectory.seed = mover.seed
                    print(trajectory)
                    return trajectory, iter

                for o in mover.entity_names:
                    if 'region' in o:
                        continue
                    if o == action.discrete_parameters['object'].GetName():
                        continue
                    for r in mover.entity_names:
                        if 'region' not in r:
                            continue
                        newaction = Operator('one_arm_pick_one_arm_place',
                                             {'object': mover.env.GetKinBody(o), 'region': mover.regions[r]})
                        action_queue.put(
                            (heuristic(newstate, newaction) - 1. * newnode.depth, float('nan'), newaction, newnode))
        else:
            raise NotImplementedError

        if not success:
            print('failed to execute action')
        else:
            print('action successful')

    assert False
    # import sys; sys.exit()

    # import sys; sys.stderr.write(str(len([('BlocksMove', p, q) for p in obj_poses for q in prm_vertices if bm('square_packing_box1', p, q)])))
    import sys;
    sys.stderr.write('generated initial state\n')

    goal = ['and'] + [('InRegion', obj_name, 'home_region') for obj_name in obj_names[0:4]]
    # goal = ['or'] + [('InRegion', obj_name, 'home_region') for obj_name in obj_names]
    # goal = ['or'] + [('InRegion', obj_name, 'home_region') for obj_name in set(obj_names) - {'rectangular_packing_box1'}]
    # goal = ['and'] + [('InRegion', obj_name, 'home_region') for obj_name in obj_names[0:2]]
    # goal = ('Holding', obj_names[1])

    # print(mover, mover.__dict__.keys())
    # print(init)
    # print(goal)

    return (domain_pddl, constant_map, stream_pddl, stream_map, init, goal)


##################################################


def turn_pick_action_into_operator(action):
    operator_type = 'two_arm_pick'
    obj_name = action[1][0]
    grasp = action[1][1]

    pick_base_pose = action[1][2]
    discrete_parameters = {'object': obj_name}
    continuous_parameters = {'grasp': grasp, 'base_pose': pick_base_pose}
    low_level_motion = None  # todo fix this
    operator = Operator(operator_type, discrete_parameters, continuous_parameters, low_level_motion)
    return operator


def turn_place_action_into_operator(action):
    operator_type = 'two_arm_place'  # action[0]
    obj_name = action[1][0]
    grasp = action[1][1]
    pick_base_pose = action[1][2]
    object_place_pose = action[1][3]

    place_base_pose = action[1][4]
    low_level_motion = action[1][5]
    place_region_name = action[1][6]

    discrete_parameters = {'region': place_region_name}
    continuous_parameters = {'base_pose': place_base_pose}
    operator = Operator(operator_type, discrete_parameters, continuous_parameters, low_level_motion)
    return operator


def make_operator_instance_from_stripstream_action(action):
    if action[0].find('pick') != -1:
        return action
    elif action[0].find('place') != -1:
        operator = turn_place_action_into_operator(action)
    else:
        raise NotImplementedError
    return operator


def simulate_plan(mover, plan):
    mover.env.SetViewer('qtcoin')
    mover.reset_to_init_state_stripstream()
    set_viewer_options(mover.env)
    # import pdb;pdb.set_trace()
    raw_input('Start?')
    handles = []
    for step_idx, action in enumerate(plan):
        # todo finish this visualization script
        # if action[0].find('pick') != -1:
        #	 obj_name = action[1][0]
        #	 object_initial_pose = action[1][1]
        #	 grasp = action[1][2]
        #	 pick_base_pose = action[1][3]
        #	 try:
        #		 mover.apply_two_arm_pick_action_stripstream((pick_base_pose, grasp), mover.env.GetKinBody(obj_name))
        #	 except:
        #		 pass
        # else:
        #	 place_obj_name = action[1][0]
        #	 place_base_pose = action[1][4]
        #	 path = action[1][5]
        #	 #visualize_path(mover.robot, path)
        #	 action = {'base_pose': place_base_pose}
        #	 obj = mover.env.GetKinBody(place_obj_name)
        #	 two_arm_place_object(obj, mover.robot, action)
        # set_robot_config(action[1][7], mover.robot)
        # set_obj_xytheta(action[1][6], mover.env.GetKinBody(action[1][0]))
        if 'pick' in action[0]:
            set_obj_xytheta([1000, 1000, 0], mover.env.GetKinBody(action[1][0]))
        # set_robot_config(action[1][3], mover.robot)
        elif 'place' in action[0]:
            set_obj_xytheta(action[1][1], mover.env.GetKinBody(action[1][0]))
        elif 'move' in action[0]:
            q1, q2 = action[1]
            handles.append(draw_edge(mover.env, q1, q2, z=0.25, color=(0, 0, 0, 1), width=1.5))
            set_robot_config(q2, mover.robot)
        elif 'vaporize' in action[0]:
            set_obj_xytheta([1000, 1000, 0], mover.env.GetKinBody(action[1][0]))
        # import pdb;pdb.set_trace()
        raw_input('Continue?')


def extract_state(mover):
    object_poses = {
        obj.GetName(): obj.GetTransform()
        for obj in mover.objects
    }
    return (
        mover.seed,
        mover.robot.GetTransform(),
        object_poses,
    )


def trajectory_length(traj):
    return sum(np.linalg.norm((a - b)[:2]) for a, b in zip(traj[:-1], traj[1:]))


def extract_cost(plan):
    # return len(plan)
    # return sum(trajectory_length(step[1][-1] if step[0] == 'pick' else step[1][-2]) for step in plan)
    return sum(np.linalg.norm(a[1] - a[0]) if t == 'move' else 10 for t, a in plan)


def extract_training_data(mover, plan):
    examples = []
    # mover.env.SetViewer('qtcoin')
    mover.reset_to_init_state_stripstream()
    state = extract_state(mover)
    last_idx = 0
    for step_idx, action in enumerate(plan):
        # todo finish this visualization script
        if action[0].find('pick') != -1:
            # obj_name = action[1][0]
            # object_initial_pose = action[1][1]
            # grasp = action[1][2]
            # pick_base_pose = action[1][2]
            # mover.apply_two_arm_pick_action_stripstream((pick_base_pose, grasp), mover.env.GetKinBody(obj_name))
            pass
        elif 'place' in action[0]:
            place_obj_name = action[1][0]
            # place_base_pose = action[1][2]
            # path = action[1][5]
            # visualize_path(mover.robot, path)
            # m_action = {'base_pose': place_base_pose}
            obj = mover.env.GetKinBody(place_obj_name)
            set_obj_xytheta(action[1][1], obj)
            set_robot_config(action[1][2], mover.robot)
            # two_arm_place_object(obj, mover.robot, m_action)
            examples.append((
                state,
                ['home_region'] + [obj.GetName() for obj in mover.objects[:4]],
                (action[1][0], action[1][3]),
                extract_cost(plan[last_idx:]),
            ))
            state = extract_state(mover)
            last_idx = step_idx

    return examples


def draw_vertex(env, v, z=0.2, **kwargs):
    x, y, _ = v
    return draw_point(env, [x, y, z], **kwargs)


def draw_edge(env, v1, v2, z=0.2, **kwargs):
    x1, y1, _ = v1
    x2, y2, _ = v2
    return draw_line(env, [x1, y1, z], [x2, y2, z], **kwargs)


def solve_pddlstream(pddlstream_problem, mover, viewer=False):
    solution_file_name = './test_results/stripstream_results_on_mover_domain/plan_random_seed_' + str(
        mover.seed) + '.pkl'
    is_problem_solved_before = os.path.isfile(solution_file_name)
    # is_problem_solved_before = False
    if viewer:
        mover.env.SetViewer('qtcoin')
        set_viewer_options(mover.env)

    for region in mover.regions:
        mover.regions[region].draw(mover.env)
    handles = []
    for vertex in prm_vertices:
        handles.append(draw_vertex(mover.env, vertex, color=(1, 0, 0, 1), size=0.01))
    print('Vertices:', len(prm_vertices))
    for i1, edges in enumerate(prm_edges):  # prm_indices
        for i2 in edges:
            handles.append(draw_edge(mover.env, prm_vertices[i1], prm_vertices[i2], color=(1, 0, 0, 1), width=1.5))
    print('Edges:', sum(len(edges) for edges in prm_edges))

    if not is_problem_solved_before:
        return None
        stime = time.time()
        pr = cProfile.Profile()
        pr.enable()
        # solution = solve_incremental(pddlstream_problem, unit_costs=True, max_time=600, planner='ff-wastar2', verbose=True)
        # solution = solve_incremental(pddlstream_problem, unit_costs=True, max_time=600, planner='ff-lazy', verbose=True)
        solution = solve_focused(pddlstream_problem, unit_costs=True, max_time=60, debug=False, planner='ff-lazy')
        pr.disable()
        pstats.Stats(pr).sort_stats('tottime').print_stats(10)
        print_solution(solution)
        search_time = time.time() - stime
        print_solution(solution)
        plan, cost, evaluations = solution
        if plan is not None:
            print 'Success'
            pickle.dump(plan, open(solution_file_name, 'wb'))
        # simulate_plan(mover, plan)
        # import pdb;pdb.set_trace()
        else:
            print "Plan not found"
        print("time: {}".format(search_time))

    else:
        print "Already solved"
        plan = pickle.load(open(solution_file_name, 'r'))
    # executable_plan = process_plan(ss_plan)
    # simulate_plan(mover, plan)

    return plan


##################################################

import multiprocessing
import traceback

from planners.subplanners.motion_planner import BaseMotionPlanner
import socket


def generate_training_data_single():
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

    # todo change the root node
    mover.init_saver = DynamicEnvironmentStateSaver(mover.env)

    hostname = socket.gethostname()
    if hostname == 'dell-XPS-15-9560' or hostname == 'phaedra' or hostname == 'shakey' or hostname == 'lab' or \
            hostname == 'glaucus':
        root_dir = './'
    else:
        root_dir = '/data/public/rw/pass.port/tamp_q_results/'

    if config.dont_use_gnn:
        solution_file_dir = root_dir + '/test_results/greedy_results_on_mover_domain/' \
                            + '/domain_' + config.domain \
                            + '/n_objs_pack_' + str(config.n_objs_pack) \
                            + '/test_purpose/no_gnn/'
    elif config.dont_use_h:
        solution_file_dir = root_dir + '/test_results/greedy_results_on_mover_domain/' \
                            + '/domain_' + config.domain \
                            + '/n_objs_pack_' + str(config.n_objs_pack) \
                            + '/test_purpose/no_h/' \
                            + '/num_train_' + str(config.num_train) + '/'
    else:
        """
        redundant = state.binary_edges[(o, r)][0]
        helps_goal = object_is_goal and region_is_goal and not redundant
        unhelpful = object_is_goal and not region_is_goal
        """

        solution_file_dir = root_dir + '/test_results/greedy_results_on_mover_domain/' \
                            + '/domain_' + config.domain \
                            + '/n_objs_pack_' + str(config.n_objs_pack) \
                            + '/test_purpose/helps_goal/' \
                            + '/num_train_' + str(config.num_train) + '/'

    solution_file_name = 'pidx_' + str(config.pidx) + \
                         '_planner_seed_' + str(config.planner_seed) + \
                         '_train_seed_' + str(config.train_seed) + '.pkl'

    if not os.path.isdir(solution_file_dir):
        os.makedirs(solution_file_dir)

    solution_file_name = solution_file_dir + solution_file_name

    is_problem_solved_before = os.path.isfile(solution_file_dir + solution_file_name)

    if is_problem_solved_before and not config.plan:
        with open(solution_file_name, 'rb') as f:
            trajectory = pickle.load(f)
            success = trajectory.metrics['success']
            tottime = trajectory.metrics['tottime']
    else:
        t = time.time()
        trajectory, num_nodes = get_problem(mover)
        tottime = time.time() - t
        success = trajectory is not None
        plan_length = len(trajectory.actions) if success else 0

        if not success:
            trajectory = Trajectory(mover.seed, mover.seed)

        trajectory.metrics = {
            'n_objs_pack': config.n_objs_pack,
            'tottime': tottime,
            'success': success,
            'plan_length': plan_length,
            'num_nodes': num_nodes,
        }

        with open(solution_file_name, 'wb') as f:
            pickle.dump(trajectory, f)

    print("time: {}".format(','.join(str(trajectory.metrics[m]) for m in [
        'n_objs_pack',
        'tottime',
        'success',
        'plan_length',
        'num_nodes',
    ])))

    print('\n'.join(str(a.discrete_parameters.values()) for a in trajectory.actions))

    mover.reset_to_init_state_stripstream()
    if not config.dontsimulate:
        if config.visualize_sim:
            mover.env.SetViewer('qtcoin')
            set_viewer_options(mover.env)

            raw_input('Start?')
        handles = []
        for step_idx, action in enumerate(trajectory.actions):
            if action.type == 'two_arm_pick':
                o = action.discrete_parameters['object']
                pickq = action.continuous_parameters['base_pose']
                pickt = action.continuous_parameters['path']
                obj = mover.env.GetKinBody(o)
                if len(pickt) > 0:
                    for q1, q2 in zip(pickt[:-1], pickt[1:]):
                        handles.append(
                            draw_edge(mover.env, q1.squeeze(), q2.squeeze(), z=0.25, color=(0, 0, 0, 1), width=1.5))
                set_robot_config(pickq, mover.robot)
                if config.visualize_sim:
                    raw_input('Continue?')
                set_obj_xytheta([1000, 1000, 0], obj)
                if config.visualize_sim:
                    raw_input('Continue?')
            elif action.type == 'two_arm_place':
                o = action.discrete_parameters['object']
                r = action.discrete_parameters['region'].name
                placeq = action.continuous_parameters['base_pose']
                placep = action.continuous_parameters['object_pose']
                placet = action.continuous_parameters['path']
                obj = mover.env.GetKinBody(o)
                if len(placet) > 0:
                    for q1, q2 in zip(placet[:-1], placet[1:]):
                        handles.append(
                            draw_edge(mover.env, q1.squeeze(), q2.squeeze(), z=0.25, color=(0, 0, 0, 1), width=1.5))
                set_robot_config(placeq, mover.robot)
                if config.visualize_sim:
                    raw_input('Continue?')
                set_obj_xytheta(placep, obj)
                if config.visualize_sim:
                    raw_input('Continue?')
            elif action.type == 'two_arm_pick_two_arm_place':
                def check_collisions(q=None):
                    if q is not None:
                        set_robot_config(q, mover.robot)
                    collision = False
                    if mover.env.CheckCollision(mover.robot):
                        collision = True
                    for obj in mover.objects:
                        if mover.env.CheckCollision(obj):
                            collision = True
                    if collision:
                        print('collision')
                        if config.visualize_sim:
                            raw_input('Continue after collision?')

                check_collisions()
                o = action.discrete_parameters['two_arm_place_object']
                pick_params, place_params = action.continuous_parameters

                full_path = [get_body_xytheta(mover.robot)[0]] + pick_params['path'] + [pick_params['base_pose']] + \
                            place_params['path'] + [place_params['base_pose']]
                for i, (q1, q2) in enumerate(zip(full_path[:-1], full_path[1:])):
                    if np.linalg.norm(np.squeeze(q1)[:2] - np.squeeze(q2)[:2]) > 1:
                        print(i, q1, q2)
                        import pdb;
                        pdb.set_trace()

                pickq = pick_params['base_pose']
                pickt = pick_params['path']
                check_collisions(pickq)
                for q in pickt:
                    check_collisions(q)
                obj = mover.env.GetKinBody(o)
                if len(pickt) > 0:
                    for q1, q2 in zip(pickt[:-1], pickt[1:]):
                        handles.append(
                            draw_edge(mover.env, q1.squeeze(), q2.squeeze(), z=0.25, color=(0, 0, 0, 1), width=1.5))
                # set_robot_config(pickq, mover.robot)
                # if config.visualize_sim:
                #	raw_input('Continue?')
                # set_obj_xytheta([1000,1000,0], obj)
                two_arm_pick_object(obj, pick_params)
                if config.visualize_sim:
                    raw_input('Place?')

                o = action.discrete_parameters['two_arm_place_object']
                r = action.discrete_parameters['two_arm_place_region']
                placeq = place_params['base_pose']
                placep = place_params['object_pose']
                placet = place_params['path']
                check_collisions(placeq)
                for q in placet:
                    check_collisions(q)
                obj = mover.env.GetKinBody(o)
                if len(placet) > 0:
                    for q1, q2 in zip(placet[:-1], placet[1:]):
                        handles.append(
                            draw_edge(mover.env, q1.squeeze(), q2.squeeze(), z=0.25, color=(0, 0, 0, 1), width=1.5))
                # set_robot_config(placeq, mover.robot)
                # if config.visualize_sim:
                #	raw_input('Continue?')
                # set_obj_xytheta(placep, obj)
                two_arm_place_object(place_params)
                if config.visualize_sim:
                    raw_input('Continue?')

                check_collisions()
            else:
                assert False

        n_objs_pack = config.n_objs_pack

        if all(mover.regions['home_region'].contains_point(get_body_xytheta(o)[0].tolist()[:2] + [1]) for o in
               mover.objects[:n_objs_pack]):
            print("successful plan length: {}".format(len(trajectory.actions)))
        else:
            print('failed to find plan')
        if config.visualize_sim:
            raw_input('Continue?')

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
    generate_training_data_single()
