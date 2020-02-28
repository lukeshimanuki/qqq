from misc.functions import irange, argmin

from manipulation.constants import RRT_ITERATIONS
from motion_planners.rrt import TreeNode, configs

# TODO - can also sample the ellipsoid of closest things to the goal (or intersection of close things to planner and goal)
# TODO - bidirectional version of this
def hill_climbing_rrt(start, goal, distance, sample, extend, collision, iterations=RRT_ITERATIONS):
  if collision(start): return None
  best_h = distance(start, goal)
  best_node = TreeNode(start)
  nodes = [best_node]
  while True:
    for q in extend(best_node.config, goal):
      if collision(q):
        break
      best_node = TreeNode(q, parent=best_node)
      nodes.append(best_node)
    else:
      return configs(best_node.retrace())
    for _ in irange(iterations):
      s = sample()
      last = argmin(lambda n: distance(n.config, s), nodes)
      success = False
      for q in extend(last.config, s):
        if collision(q):
          break
        last = TreeNode(q, parent=last)
        nodes.append(last)
        h = distance(q, goal)
        if h < best_h:
          best_h = h
          best_node = last
          success = True
          break
      if success:
        break
  return None