from __future__ import print_function

import os
import time
import pickle
import random
import copy
import Queue
import sys
import cProfile
import pstats
import argparse

import pddlstream.algorithms.instantiate_task
pddlstream.algorithms.instantiate_task.FD_INSTANTIATE = False

from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.constants import print_solution
from pddlstream.utils import read
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_test, fn_from_constant, from_fn
from pddlstream.algorithms.search import SERIALIZE
#import pdb;pdb.set_trace()

from problem_environments.mover_env import Mover
from generators.PickUniform import PickWithBaseUnif
from generators.PlaceUniform import PlaceUnif
from generators.uniform import UniformGenerator, PaPUniformGenerator
from generators.one_arm_pap_uniform_generator import OneArmPaPUniformGenerator

from trajectory_representation.operator import Operator
from planners.subplanners.motion_planner import BaseMotionPlanner

from mover_library.utils import set_robot_config, set_obj_xytheta, visualize_path, two_arm_pick_object, two_arm_place_object, \
    get_body_xytheta, CustomStateSaver, set_color

from mover_library.motion_planner import rrt_region

from openravepy import RaveSetDebugLevel, DebugLevel
from trajectory_representation.trajectory import Trajectory

import numpy as np
import openravepy

from collections import Counter
from manipulation.primitives.display import set_viewer_options, draw_line, draw_point
from manipulation.primitives.savers import DynamicEnvironmentStateSaver

PRM_VERTICES, PRM_EDGES = pickle.load(open('prm.pkl', 'rb'))
# prm_edges = [set(l) - {i} for i,l in enumerate(prm_edges)]
PRM_VERTICES = list(PRM_VERTICES)  # TODO: needs to be a list rather than ndarray

CONNECTED = np.array([len(s) >= 2 for s in PRM_EDGES])
PRM_INDICES = {tuple(v): i for i, v in enumerate(PRM_VERTICES)}
DISABLE_COLLISIONS = False
MAX_DISTANCE = 1.0
MAX_PICK_DISTANCE = 1.0  # Previously 1.5 for pick and 1.0 for place

# cache ik solutions
ikcachename = './ikcache.pkl'
iksolutions = {}
iksolutions = pickle.load(open(ikcachename, 'r'))

PRM_VERTICES = PRM_VERTICES[::10]
PRM_EDGES = []


def gen_grasp(pick_unif):
    # note generate grasp, ik solution gc, relative base conf, and absolute base transform for grasping
    def fcn(obj_name):
        pick_unif.problem_env.reset_to_init_state_stripstream()
        obj = pick_unif.problem_env.env.GetKinBody(obj_name)

        for i in range(1000):
            print("Calling gengrasp")
            action = pick_unif.predict(obj, pick_unif.problem_env.regions['entire_region'], n_iter=100)
            pick_base_pose = action['base_pose']
            grasp = action['grasp_params']
            g_config = action['g_config']
            pick_unif.problem_env.reset_to_init_state_stripstream()
            if g_config is None:
                # yield None
                continue
            print(grasp, pick_base_pose)
            yield [grasp, pick_base_pose]

    return fcn


def find_path(graph, start, goal, heuristic=lambda x: 0, collision=lambda x: False):
    visited = {s for s in start}
    queue = Queue.PriorityQueue()
    for s in start:
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
                newdist = dist + np.linalg.norm(PRM_VERTICES[vertex] - PRM_VERTICES[next])
                queue.put((newdist + heuristic(next), newdist, np.random.rand(), next, path + [next]))
    return None


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
            print("Calling gengrasp")
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
            start = (np.linalg.norm(PRM_VERTICES - init_conf, axis=1) + (1 - CONNECTED) * 1e6).argmin()
            goal = (np.linalg.norm(PRM_VERTICES - pick_base_pose, axis=1) + (1 - CONNECTED) * 1e6).argmin()

            disth = lambda x: np.linalg.norm(PRM_VERTICES[x, :2] - PRM_VERTICES[goal, :2])

            # if not dfs(prm_edges, start, goal, disth):
            #	 continue

            generators.append(enumerate_paths(PRM_EDGES, start, goal, disth))

            for gen in generators:
                path = next(gen)
                if path is not None:
                    yield [grasp, pick_base_pose,
                           np.concatenate([[init_conf], PRM_VERTICES[path, :], [pick_base_pose]])]

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
                print("Input", obj_name, grasp, pick_base_pose)
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
            set_robot_config(PRM_VERTICES[x], problem.robot)
            return problem.env.CheckCollision(problem.robot)

        start = (np.linalg.norm(PRM_VERTICES - q1, axis=1) < .5).nonzero()[0]
        if len(start) == 0:
            start = [(np.linalg.norm(PRM_VERTICES - q1, axis=1) + (1 - CONNECTED) * 1e6).argmin()]
        goal = (np.linalg.norm(PRM_VERTICES - q2, axis=1) + (1 - CONNECTED) * 1e6).argmin()
        path = find_path(PRM_EDGES, start, lambda x: x == goal,
                         lambda x: np.linalg.norm(PRM_VERTICES[x] - PRM_VERTICES[goal]), collision_check)
        path = None

        if path is not None:
            problem.reset_to_init_state_stripstream()

            pickle.dump([PRM_VERTICES[p] for p in path], open('pick_path.pkl', 'wb'))
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
            set_robot_config(PRM_VERTICES[x], problem.robot)
            return problem.env.CheckCollision(problem.robot)

        start = (np.linalg.norm(PRM_VERTICES - q1, axis=1) < .5).nonzero()[0]
        if len(start) == 0:
            start = [(np.linalg.norm(PRM_VERTICES - q1, axis=1) + (1 - CONNECTED) * 1e6).argmin()]
        goal = (np.linalg.norm(PRM_VERTICES - q2, axis=1) + (1 - CONNECTED) * 1e6).argmin()
        path = find_path(PRM_EDGES, start, lambda x: x == goal,
                         lambda x: np.linalg.norm(PRM_VERTICES[x] - PRM_VERTICES[goal]), collision_check)
        path = None

        # problem.reset_to_init_state_stripstream()
        # path, status = problem.get_base_motion_plan(q2, 'entire_region', [10, 200, 300, 400, 5000])

        # if status == 'HasSolution':
        if path is not None:
            problem.reset_to_init_state_stripstream()

            pickle.dump([PRM_VERTICES[p] for p in path], open('place_path.pkl', 'wb'))
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
            set_robot_config(PRM_VERTICES[x], problem.robot)
            return problem.env.CheckCollision(problem.robot)

        start = (np.linalg.norm(PRM_VERTICES - q1, axis=1) < .5).nonzero()[0]
        if len(start) == 0:
            start = [(np.linalg.norm(PRM_VERTICES - q1, axis=1) + (1 - CONNECTED) * 1e6).argmin()]
        path = find_path(PRM_EDGES, start, lambda x: region.contains_point([PRM_VERTICES[x, 0], PRM_VERTICES[x, 1], 1]),
                         lambda x: 0, collision_check)

        # problem.reset_to_init_state_stripstream()
        # path, status = problem.get_base_motion_plan(q2, 'entire_region', [10, 200, 300, 400, 5000])

        problem.reset_to_init_state_stripstream()

        # if status == 'HasSolution':
        if path is not None:
            pickle.dump([PRM_VERTICES[p] for p in path], open('region_path.pkl', 'wb'))
            return False
        else:
            set_robot_config(q1, problem.robot)

            def collision(q):
                set_robot_config(q, problem.robot)
                return problem.env.CheckCollision(problem.robot)
                set_obj_xytheta(q, oo)
                return problem.env.CheckCollision(oo)

            path, status = problem.get_base_motion_plan_region(
                lambda q: region.contains_point([q[0], q[1], 1]), 'entire_region', [1000],
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


def collides_move(problem):
    def fcn(q1, q2, o, p):
        obj = problem.env.GetKinBody(o)
        problem.disable_objects()
        obj.Enable(True)
        set_obj_xytheta(p, obj)
        n = 2 + int(np.linalg.norm(q1 - q2) / .8)
        for i in range(n):
            q = q1 + (q2 - q1) * i / (n - 1)
            set_robot_config(q)
            if problem.env.CheckCollision(problem.robot):
                return True
        return False

    return fcn

def collides_carry(problem):
    def fcn(o, p, q1, q2, oo=None, g=None):
        if oo is None:
            pass
        else:
            assert g is not None
        return False
    return fcn

def not_near(problem):
    def fcn(q1, q2):
        return np.linalg.norm(q1[:2] - q2[:2]) < 1.2

    return fcn

def blocks_move(problem):
    obj = problem.objects[0]

    def fcn(p, *qs):
        if DISABLE_COLLISIONS:
            return False
        if all(np.linalg.norm((p - q)[:2]) > 0.8 for q in qs):
            return False
        # TODO: cache bounding box to prune some collisions
        # set the obstacle in place
        set_obj_xytheta(p, obj)

        # check collision
        # problem.disable_objects_in_region('entire_region')
        obj.Enable(True)
        result = check_edge_collision(problem, qs, obj=obj)
        # TODO(lukeshim): no need to reset so often. Causes significant overhead
        # problem.reset_to_init_state_stripstream()
        # problem.enable_objects_in_region('entire_region')
        return result

    return fcn


def blocks_place(problem):
    obj1, obj2 = problem.objects[:2]  # TODO: these are objects of different sizes

    def fcn(p1, p2):
        if DISABLE_COLLISIONS:
            return False
        if 0.3 < np.linalg.norm((p1 - p2)[:2]):  # Previously this was incorrect
            return False
        set_obj_xytheta(p1, obj1)
        set_obj_xytheta(p2, obj2)
        return problem.env.CheckCollision(obj1, obj2)

    return fcn


def gen_edge(problem, num_neighbors=3):
    def fcn(q1):
        key = tuple(q1)
        if key in PRM_INDICES:
            # for i in prm_edges[prm_indices[key]]:
            #	yield PRM_VERTICES[i],
            raise RuntimeError()
        else:
            dist = np.linalg.norm(np.array(PRM_VERTICES) - np.array(q1), axis=1)
            vertices = []
            # handles = []
            problem.disable_objects_in_region('entire_region')
            for v in sorted(range(len(PRM_VERTICES)), key=lambda i: dist[i])[:num_neighbors]:
                q2 = PRM_VERTICES[v]
                if check_edge_collision(problem, q1, q2):
                    continue
                vertices.append((q2,))
            # handles.append(draw_edge(problem.env, q1, PRM_VERTICES[v], z=0.25, color=(0, 0, 0, 1)))
            # yield PRM_VERTICES[v],
            # for i in prm_edges[v]:
            #	yield PRM_VERTICES[i],
            # raw_input('Continue?')
            problem.enable_objects_in_region('entire_region')
            return vertices

    return fcn


def test_edge(problem):
    def fcn(q1, q2):
        return np.linalg.norm(np.array(q2) - np.array(q1)) < MAX_DISTANCE

    return fcn

def gen_pap(problem):
    def fcn(o, r, s):
        while True:
            s.Restore()
            if problem.name == 'two_arm_mover':
                action = Operator('two_arm_pick_two_arm_place', {'object': o, 'region': r})
                sampler = PaPUniformGenerator(action, problem, None)
                params = sampler.sample_next_point(action, n_iter=200, n_parameters_to_try_motion_planning=3)
                if params['is_feasible']:
                    action.continuous_parameters = params
                    action.execute()
                    t = CustomStateSaver(problem.env)
                    yield params, t
                else:
                    yield None

                if params['is_feasible']:
                    action.continuous_parameters = params
                    action.execute()
                    t = CustomStateSaver(problem.env)
                    yield params, t
                else:
                    yield None
            elif problem.name == 'one_arm_mover':
                action = Operator('one_arm_pick_one_arm_place', {'object': problem.env.GetKinBody(o), 'region': problem.regions[r]})
                current_region = problem.get_region_containing(problem.env.GetKinBody(o)).name
                sampler = OneArmPaPUniformGenerator(action, problem, cached_picks=(iksolutions[current_region], iksolutions[r]))
                pick_params, place_params, status = sampler.sample_next_point(action)

                if status == 'HasSolution':
                    action.continuous_parameters = {'pick': pick_params, 'place': place_params}
                    action.execute()
                    t = CustomStateSaver(problem.env)
                    yield action.continuous_parameters, t
                else:
                    yield None
            else:
                raise NotImplementedError

    return fcn


def transform_pick(problem):
    def fcn(o, g, q):
        obj = problem.env.GetKinBody(o)
        while True:
            if problem.name == 'two_arm_mover':
                problem.reset_to_init_state_stripstream()
                s = q + .001 # TODO: gen random
                pickp, pickq, grasp, grasp_config = g
                set_obj_xytheta(pickp, obj)
                action = {'base_pose': pickq, 'g_config': grasp_config}
                two_arm_pick_object(o, action)
                set_robot_config(s)
                placep = get_body_xytheta(obj)
                problem.reset_to_init_state_stripstream()
                yield placep,s
            elif problem.name == 'one_arm_mover':
                raise NotImplementedError
            else:
                raise NotImplementedError

    return fcn


def gen_pick(problem, pick_unif):
    def fcn(o, p, prm_q=None):
        obj = pick_unif.problem_env.env.GetKinBody(o)

        while True:
            print('gen-pick')
            problem.reset_to_init_state_stripstream()
            set_obj_xytheta(p, obj)
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

            yield pick_base_pose, (grasp, g_config)

    return fcn


def gen_place(problem, place_unif):
    def fcn(o, pickp, pickq, (g, gc), r, prm_q=None):
        obj = problem.env.GetKinBody(o)
        while True:
            print('gen-place')
            problem.reset_to_init_state_stripstream()
            set_obj_xytheta(pickp, obj)
            action = {'base_pose': pickq, 'g_config': gc}
            two_arm_pick_object(obj, action)
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

def gen_conf(problem, place_unif):
    def fcn(r):
        while True:
            problem.reset_to_init_state_stripstream()
            obj = problem.env.GetKinBody(o)
            grab_obj(obj)
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
            yield place_base_pose

    return fcn


def test_front_pick(problem, q, p):
    # TODO: KD-tree to do this more quickly
    disp = p[:2] - q[:2]
    if np.linalg.norm(disp) < 1e-3:  # previously 0.5 Too close
        return False
    if MAX_PICK_DISTANCE < np.linalg.norm(disp):  # Too far
        return False
    yaw = q[2]
    head = np.array([np.cos(yaw), np.sin(yaw)])
    if head.dot(disp) / np.linalg.norm(disp) < 0.5:  # pi / 3
        return False
    obj = problem.objects[0]
    set_obj_xytheta(p, obj)
    obj.Enable(True)
    return not check_edge_collision(problem, [q], obj=obj)


def front_place(problem):
    def fcn(q):
        for p in PRM_VERTICES:
            if test_front_pick(problem, q, p):
                yield p,

    # r = 1
    # yield np.array([
    #	q[0] + r * np.cos(q[2]),
    #	q[1] + r * np.sin(q[2]),
    #	q[2],
    # ]),
    return fcn


def gen_placement(problem, place_unif):
    #  todo take region as parameter
    # note generate object placement, relative base conf, absolute base conf, and the path from q1 to abs base conf
    def fcn(obj_name, grasp, pick_base_pose, region):
        # simulate pick
        generators = []
        for i in range(100):
            print('Placement region ', region)
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

            start = (np.linalg.norm(PRM_VERTICES - pick_base_pose, axis=1) + (1 - CONNECTED) * 1e6).argmin()
            goal = (np.linalg.norm(PRM_VERTICES - place_base_pose, axis=1) + (1 - CONNECTED) * 1e6).argmin()

            disth = lambda x: np.linalg.norm(PRM_VERTICES[x, :2] - PRM_VERTICES[goal, :2])

            generators.append(enumerate_paths(PRM_EDGES, start, goal, disth))

            for gen in generators:
                path = next(gen)
                if path is not None:
                    yield (place_base_pose, object_pose,
                           np.concatenate([[pick_base_pose], PRM_VERTICES[path, :], [place_base_pose]]))

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
                print("Input", obj_name, grasp, pick_base_pose)
                yield (place_base_pose, object_pose, path)
            else:
                problem.reset_to_init_state_stripstream()
                yield None

    return fcn


def get_problem(mover, n_objs_pack=1):
    directory = os.path.dirname(os.path.abspath(__file__))
    domain_pddl = read(os.path.join(directory, 'domainprm.pddl'))
    stream_pddl = read(os.path.join(directory, 'streamprm.pddl'))

    pick_sampler = PickWithBaseUnif(mover)
    place_sampler = PlaceUnif(mover)
    constant_map = {}
    stream_map = {
        # 'gen-grasp-traj': from_gen_fn(gen_grasp_traj(mover, pick_sampler)),
        # 'gen-placement': from_gen_fn(gen_placement(mover, place_sampler)),
        #'gen-edge': from_list_fn(gen_edge(mover)),
        # 'test-edge': from_test(test_edge(mover)),
        #'gen-pap': from_gen_fn(gen_pap(mover)),
        'transform-pick': from_gen_fn(transform_pick(mover)),
        #'gen-pick': from_gen_fn(gen_pick(mover, pick_sampler)),
        #'gen-place': from_gen_fn(gen_place(mover, place_sampler)),
        #'NotNear': not_near(mover),
        #'gen-conf': from_gen_fn(gen_conf(mover, place_sampler)),
        #'front-place': from_gen_fn(front_place(mover)),
        # 'FrontPick': front_pick(mover),
        #'BlocksMove': blocks_move(mover),
        #'CollidesMove': collides_carry(mover),
        #'CollidesCarry': collides_carry(mover),
        #'BlocksPlace': blocks_place(mover),
        # 'PlaceTrajPoseCollision': place_check_traj_collision(mover),
        # 'PickTrajPoseCollision': pick_check_traj_collision(mover),
        # 'ReachablePred': reachable_pred(mover),
        # 'ReachablePlacePred': reachable_place_pred(mover),
        # 'ReachableRegionPred': reachable_region_pred(mover),
        #'Distance': lambda q1, q2: np.linalg.norm((q2 - q1)[:2]),
    }
    # stream_map = 'debug'

    obj_names = [obj.GetName() for obj in mover.objects]
    obj_poses = [get_body_xytheta(mover.env.GetKinBody(obj_name)).squeeze() for obj_name in obj_names]

    initial_robot_conf = get_body_xytheta(mover.robot).squeeze()
    # import pdb;pdb.set_trace()

    init = [('Pickable', obj_name) for obj_name in obj_names]
    init += [('InRegion', obj_name, mover.get_region_containing(mover.env.GetKinBody(obj_name)).name) for obj_name in obj_names]
    init += [('QInRegion', q, 'home_region' if mover.regions['home_region'].contains_point(list(q[:2])+[1]) else 'loading_region') for q in PRM_VERTICES]
    init += [('Region', region) for region in mover.regions if region != 'entire_region']


    for o,p in zip(obj_names, obj_poses):
        num_grasps = 0
        num_tries = 0
        while num_grasps < 2 and num_tries < 10:
            num_tries += 1
            mover.reset_to_init_state_stripstream()
            action = pick_sampler.predict(mover.env.GetKinBody(o), mover.regions['entire_region'], n_iter=50)

            pickq = action['base_pose']
            if pickq is None:
                continue
            grasp = action['grasp_params']
            grasp_config = action['g_config']
            if grasp_config is None:
                continue
            if grasp is None:
                continue

            num_grasps += 1
            g = p, pickq, grasp, grasp_config
            init += [('Grasp', g)]
            init += [('Sampled', pickq)]
            init += [('Q', pickq)]
            init += [
                ('Pick', o, p, q, pickq, g)
                for q in PRM_VERTICES
                if np.linalg.norm(q[:2] - pickq[:2]) < 1.2
            ]
    mover.reset_to_init_state_stripstream()

    goal_objects = obj_names[0:n_objs_pack]
    #non_goal_objects = obj_names[n_objs_pack:]

    if mover.name == 'two_arm_mover':
        goal_region = 'home_region'
        nongoal_regions = ['loading_region']
    elif mover.name == 'one_arm_mover':
        goal_region = mover.target_box_region.name
        nongoal_regions = list(mover.shelf_regions)
    else:
        raise NotImplementedError
    #init += [('NonGoalObject', obj_name) for obj_name in non_goal_objects]
    init += [('GoalObject', obj_name) for obj_name in goal_objects]
    init += [('NonGoalRegion', region) for region in nongoal_regions]
    init += [('GoalRegion', goal_region)]

    #init_state = CustomStateSaver(mover.env)
    #init += [('State', init_state)]
    #init += [('AtState', init_state)]

    # robot initialization
    init += [('EmptyArm',)]
    init += [('AtConf', initial_robot_conf)]
    init += [('BaseConf', initial_robot_conf)]
    init += [('Q', initial_robot_conf)]
    #init += [('BaseConf', initial_robot_conf), ('Sampled', initial_robot_conf)]

    # object initialization
    init += [('Pose', obj_pose) for obj_name, obj_pose in zip(obj_names, obj_poses)]
    #init += [('PoseInRegion', obj_pose, 'loading_region') for obj_name, obj_pose in zip(obj_names, obj_poses)]
    init += [('AtPose', obj_name, obj_pose) for obj_name, obj_pose in zip(obj_names, obj_poses)]
    #init += [('PlacedAt', obj_pose) for obj_pose in obj_poses]

    # prm initialization
    init += [('BaseConf', q) for q in PRM_VERTICES]
    #init += [('Q', q) for q in PRM_VERTICES]
    init += [('Edge', PRM_VERTICES[q1], PRM_VERTICES[q2])
             for q1, e in enumerate(PRM_EDGES) for q2 in e]
    # TODO: goal serialization (can allow algorithm to pick the easist
    # TODO: selectively introduce objects
    # TODO: reachability test with only interesting pick/place confs
    # TODO: downsample the set of placements

    #poses = list(PRM_VERTICES) + obj_poses
    #init += [('PoseInRegion', q, 'home_region') for q in poses if -3 < q[1]]
    # init += [('PoseInRegion', q, 'home_region') for q in PRM_VERTICES if -3 < q[1]]
    # init += [('PoseInRegion', q, 'loading_region') for q in PRM_VERTICES if q[1] < -5]

    # TODO: why are there separate pick and place mechanics?
    # fp = front_place(mover)
    # pairs = [(q, p[0]) for q in PRM_VERTICES for p in fp(q) if p is not None]
    # init += [('FrontPlace', q, p) for q,p in pairs]
    # fpi = front_pick(mover)
    # poses = [p for q,p in pairs] + obj_poses
    #init += [('FrontPick', q, p) for q in PRM_VERTICES for p in poses
    #         if test_front_pick(mover, q, p)]
    # init += [('FrontPlace', q, p) for q in PRM_VERTICES for p in poses if fpl(q,p)]
    init += [('Edge', initial_robot_conf, q) for q in PRM_VERTICES
             if np.linalg.norm(q - initial_robot_conf) < 1.2]
    #init += [('Pose', p) for p in poses]
    # init += [('FrontPlace', initial_robot_conf, p) for p in fp(initial_robot_conf)]
    # init += [('Edge', PRM_VERTICES[q2], PRM_VERTICES[q1])
    #		 for q1, e in enumerate(prm_edges) for q2 in e]
    # start = (np.linalg.norm(PRM_VERTICES - initial_robot_conf, axis=1) + (1-connected) * 1e6).argmin()
    # init += [('Edge', initial_robot_conf, PRM_VERTICES[start])] + [
    #	('Edge', initial_robot_conf, PRM_VERTICES[q]) for q in prm_edges[start]
    # ]

    # bm = blocks_move(mover)
    # try:
    #	blocksmove = pickle.load(open('blocksmove.pkl', 'rb'))
    # except:
    #	blocksmove = [(p, q) for p in range(len(pairs)) for q in range(len(PRM_VERTICES)) if bm(pairs[p][1], PRM_VERTICES[q])]
    #	pickle.dump(blocksmove, open('blocksmove.pkl', 'wb'))
    # init += [('BlocksMove', pairs[p][1], PRM_VERTICES[q]) for p, q in blocksmove]
    # init += [('BlocksMoveS', p, q) for p in poses for q in PRM_VERTICES if bm(p, q)]

    # import sys; sys.stderr.write(str(len([('BlocksMove', p, q) for p in obj_poses for q in PRM_VERTICES if bm('square_packing_box1', p, q)])))
    import sys;
    #sys.stderr.write('generated initial state\n')

    goal = ['and'] + [('InRegion', obj_name, goal_region)
                      for obj_name in goal_objects]
    #goal = ['not', ('EmptyArm',)]
    #goal = ['and'] + [('Moved', goal_objects[0])]
    #goal = ['and'] + [('InRegion', obj_name, 'home_region')
    #                  for obj_name in obj_names[0]]
    #goal = ['or'] + [('InRegion', obj_name, 'home_region') for obj_name in obj_names]
    # goal = ['or'] + [('InRegion', obj_name, 'home_region') for obj_name in set(obj_names) - {'rectangular_packing_box1'}]
    # goal = ['and'] + [('InRegion', obj_name, 'home_region') for obj_name in obj_names[0:2]]
    # goal = ('Holding', obj_names[1])

    # print(mover, mover.__dict__.keys())
    # print(init)
    print('Num init:', Counter(fact[0] for fact in init))
    print('Goal:', goal)
    print('Streams:', sorted(stream_map))

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal


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
            body = mover.env.GetKinBody(action[1][0])
            body.SetVisible(False)
        # set_robot_config(action[1][3], mover.robot)
        elif 'place' in action[0]:
            body = mover.env.GetKinBody(action[1][0])
            set_obj_xytheta(action[1][1], body)
            body.SetVisible(True)
        elif 'move' in action[0]:
            q1, q2 = action[1]
            handles.append(draw_edge(mover.env, q1, q2, z=0.25, color=(0, 0, 0, 1), width=1.5))
            set_robot_config(q2, mover.robot)
        elif 'vaporize' in action[0]:
            body = mover.env.GetKinBody(action[1][0])
            body.SetVisible(False)
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
    return sum(np.linalg.norm((a - b)[:2])
               for a, b in zip(traj[:-1], traj[1:]))


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


def solve_pddlstream(mover, execute=False, resolve=False, viewer=False):
    solution_file_name = './test_results/stripstream_results_on_mover_domain/plan_random_seed_{}.pkl'.format(mover.seed)
    is_problem_solved_before = not resolve and os.path.isfile(solution_file_name)
    if viewer:
        mover.env.SetViewer('qtcoin')
        set_viewer_options(mover.env)

    for region in mover.regions:
        if region in ['home_region']:  # home_region, entire_region, loading_region
            mover.regions[region].draw(mover.env)
    handles = []
    for vertex in PRM_VERTICES:
        handles.append(draw_vertex(mover.env, vertex, color=(1, 0, 0, 1), size=0.01))
    print('Vertices:', len(PRM_VERTICES))
    for i1, edges in enumerate(PRM_EDGES):  # prm_indices
        for i2 in edges:
            handles.append(draw_edge(mover.env, PRM_VERTICES[i1], PRM_VERTICES[i2], color=(1, 0, 0, 1), width=1.5))
    print('Edges:', sum(len(edges) for edges in PRM_EDGES))
    if is_problem_solved_before:
        print("Already solved")
        plan = pickle.load(open(solution_file_name, 'r'))
        if execute:
            simulate_plan(mover, plan)
        return plan

    pddlstream_problem = get_problem(mover)
    # raw_input('Start?')
    # return None
    stime = time.time()
    pr = cProfile.Profile()
    pr.enable()
    # planner = 'ff-lazy' # -tiebreak
    # planner = 'ff-eager-tiebreak' # -tiebreak
    planner = 'ff-wastar5'
    # planner = 'cea-wastar5' # Performs worse than ff-wastar
    # planner = 'ff-ehc' # Worse

    #import pdb;pdb.set_trace()
    solution = solve_focused(pddlstream_problem, unit_costs=True, max_time=config.timelimit,
    #solution = solve_incremental(pddlstream_problem, unit_costs=True, max_time=config.timelimit,
                                 planner=planner, debug=True, verbose=True)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    search_time = time.time() - stime
    #print_solution(solution)
    plan, cost, evaluations = solution
    #import pdb;pdb.set_trace()
    if plan is not None:
        print('Success')
        print(plan)
        pickle.dump(plan, open(solution_file_name, 'wb'))
        if execute:
            simulate_plan(mover, plan)
    # import pdb;pdb.set_trace()
    else:
        print("Plan not found")
    print("time: {}".format(search_time))

    return plan

def solve_stripstream(mover, config):
    if config.visualize_plan:
        mover.env.SetViewer('qtcoin')
        set_viewer_options(mover.env)

    for region in mover.regions:
        if region in ['home_region']:  # home_region, entire_region, loading_region
            mover.regions[region].draw(mover.env)
    handles = []
    for vertex in PRM_VERTICES:
        handles.append(draw_vertex(mover.env, vertex, color=(1, 0, 0, 1), size=0.01))
    print('Vertices:', len(PRM_VERTICES))
    for i1, edges in enumerate(PRM_EDGES):  # prm_indices
        for i2 in edges:
            handles.append(draw_edge(mover.env, PRM_VERTICES[i1], PRM_VERTICES[i2], color=(1, 0, 0, 1), width=1.5))
    print('Edges:', sum(len(edges) for edges in PRM_EDGES))

    pddlstream_problem = get_problem(mover, config.n_objs_pack)
    # raw_input('Start?')
    # return None
    stime = time.time()
    pr = cProfile.Profile()
    pr.enable()
    # planner = 'ff-lazy' # -tiebreak
    # planner = 'ff-eager-tiebreak' # -tiebreak
    planner = 'ff-wastar5'
    # planner = 'cea-wastar5' # Performs worse than ff-wastar
    # planner = 'ff-ehc' # Worse

    #import pdb;pdb.set_trace()
    set_color(mover.objects[0], [1, 0, 0])
    solution = solve_focused(pddlstream_problem, unit_costs=True, max_time=10 * 60,
    #solution = solve_incremental(pddlstream_problem, unit_costs=True, max_time=10 * 60,
                                 planner=planner, debug=True, verbose=True)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    search_time = time.time() - stime
    #print_solution(solution)
    plan, cost, evaluations = solution
    #import pdb;pdb.set_trace()
    print("time: {}".format(search_time))
    if plan is not None:
        print('Success')

        trajectory = Trajectory(mover.seed, mover.seed)
        trajectory.actions = [
            Operator('two_arm_pick_two_arm_place', {
                'two_arm_place_object': action.args[0],
                'two_arm_place_region': action.args[1],
            }, action.args[3])
            for action in plan
        ]
        trajectory.seed = mover.seed
        print(trajectory)
        return trajectory, 0 # TODO: count num nodes
    else:
        print("Plan not found")
        return None, 0


##################################################

import multiprocessing
import traceback


def generate_training_data_single(seed, examples, **kwargs):
    np.random.seed(seed)
    random.seed(seed)
    mover = Mover(problem_idx=0)
    mover.seed = seed
    """
    for obj in mover.objects:
        if obj.GetName() != 'square_packing_box1' and obj.GetName() != 'rectangular_packing_box1' \
                and obj.GetName() != 'square_packing_box2' and obj.GetName() != 'square_packing_box3' \
                and obj.GetName() != 'square_packing_box3' and obj.GetName() != 'square_packing_box4' \
                and obj.GetName() != 'rectangular_packing_box2' and obj.GetName() != 'rectangular_packing_box3':
            object_idx = np.where([o==obj for o in mover.objects])[0][0]
            del mover.objects[object_idx]
            mover.env.Remove(obj)
    """
    all_objects = list(mover.objects)
    mover.objects = all_objects  # [:4]
    for obj in all_objects:
        if obj not in mover.objects:
            mover.env.RemoveKinBody(obj)
    mover.init_saver = DynamicEnvironmentStateSaver(mover.env)
    # mover.env.SetViewer('qtcoin')
    # import pdb;pdb.set_trace()
    #import pdb;pdb.set_trace()

    try:
        plan = solve_pddlstream(mover, **kwargs)
        #import pdb;pdb.set_trace()

        # import pdb; pdb.set_trace()

        if plan is not None:
            for example in extract_training_data(mover, plan):
                examples.put(example, False)
            print("generated examples for seed {}".format(seed))

    except Exception as e:
        print("{}: {}".format(type(e).__name__, e.message))
        traceback.print_exc()

    mover.problem_config['env'].Destroy()
    openravepy.RaveDestroy()


def generate_training_data_core(seeds, examples):
    while True:
        try:
            seed = seeds.get(False)
        except multiprocessing.Queue.Empty:
            return

        print("planning for seed {}".format(seed))
        process = multiprocessing.Process(target=generate_training_data_single, args=(seed, examples))
        process.start()
        process.join(120)
        if process.is_alive():
            process.terminate()
            print("ran out of time for seed {}".format(seed))


def generate_training_data(num_seeds, num_cores):
    examples_list = []

    seeds = multiprocessing.Queue()
    examples = multiprocessing.Queue()

    for seed in range(num_seeds):
        seeds.put(seed, False)

    cores = [
        multiprocessing.Process(target=generate_training_data_core, args=(seeds, examples))
        for core in range(num_cores)
    ]

    for core in cores:
        core.start()

    while any(core.is_alive() for core in cores):
        examples_list.append(examples.get())

        with open('training_data.pkl', 'wb') as out_file:
            pickle.dump(examples_list, out_file)

    return examples_list


if __name__ == '__main__':
    # todo pass in objects and their poses to picktrajgen and placetrajgen and call RRT with objects placed at the
    #	   poses
    parser = argparse.ArgumentParser()
    parser.add_argument('-e', '--execute', action='store_true', help='Execute')
    parser.add_argument('-r', '--resolve', action='store_true', help='Resolve')
    parser.add_argument('-s', '--serial', action='store_true', help='Serial')
    parser.add_argument('-v', '--viewer', action='store_true', help='Viewer')
    args = parser.parse_args()
    RaveSetDebugLevel(DebugLevel.Fatal)
    np.random.seed(0)
    random.seed(0)
    mover = Mover(problem_idx=0)
    mover.seed=0
    mover.set_motion_planner(BaseMotionPlanner(mover, 'prm'))
    solve_pddlstream(mover)
    """
    if args.serial:
        examples = multiprocessing.Queue()
        generate_training_data_single(1, examples, execute=args.execute, resolve=args.resolve, viewer=args.viewer)
    else:
        generate_training_data(1000, 2)
    """
