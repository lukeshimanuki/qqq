from random import randint

import numpy as np

from misc.functions import pairs

def linear_shortcut(body, path, indices=None):
  if indices is None: indices = range(body.GetActiveDOF())
  n = len(path)
  q = path[0]
  for i in range(n): # TODO - might be able to reduce the number of samples if moving the bottleneck DOF
    q = (1./(n-i))*body.SubtractActiveDOFValues(path[-1], q) + q
    q_new = path[i].copy()
    q_new[indices] = q[indices]
    yield q_new

def dof_smooth_path(path, shortcut, collision, iterations=50):
  # TODO - random subsets of DOFS / explicitly related DOFS (arms/base/etc)
  smoothed_path = path
  for d in range(len(path[0])-3):
    segment = list(shortcut(smoothed_path[:], indices=[d]))
    if all(not collision(q) for q in segment):
      smoothed_path = segment

  for _ in range(iterations):
    if len(smoothed_path) <= 2:
      return smoothed_path
    i, j = randint(0, len(smoothed_path)-1), randint(0, len(smoothed_path)-1)
    if abs(i-j) <= 1: continue
    if j < i: i, j = j, i
    d = randint(0, len(path[0])-1)
    segment = list(shortcut(smoothed_path[i:j+1], indices=[d]))
    if all(not collision(q) for q in segment):
      smoothed_path = smoothed_path[:i] + segment + smoothed_path[j+1:]
  return smoothed_path

def sparsify_path(robot, path):
  diff = [robot.SubtractActiveDOFValues(q2, q1) for q1, q2 in pairs(path)]
  sparse_path = []
  config = path[0]
  i = 0
  while i < len(diff) - 1:
    sparse_path.append(config.copy())
    config += diff[i]
    for j in range(i+1, len(diff)):
      if not np.allclose(diff[i], diff[j]):
        break
      config += diff[j]
    i = j
  return sparse_path + [config.copy()]