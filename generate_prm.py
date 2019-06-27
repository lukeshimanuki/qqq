import collections
import os
import pdb
import pickle

import numpy as np
from matplotlib import pyplot as plt

def load_prm(path):
	vertices = []
	for seed in range(1000):
		filename = "{}/plan_random_seed_{}.pkl".format(path, seed)

		try:
			plan = pickle.load(open(filename, 'rb'))
		except:
			continue

		for action, params in plan:
			for point in params[5]:
				if vertices and any(np.linalg.norm((point - vertex)[:3]) < .6 for vertex in vertices):# and np.abs(point[2] - vertex[2]) < .2:
					continue
				if -5.3 < point[1] < -3.2 and not (.2 < point[0] < 1.3):
					continue
				vertices.append(point)

		print(len(vertices))

	edges = [
		set(sorted([
			i
			for i,b in enumerate(vertices)
			if np.linalg.norm((a - b)[:2]) < .8
		], key=lambda i: np.linalg.norm((vertices[i] - a)[:2]))[1:])
		for a in vertices
	]

	
	def find_path(graph, start, goal, heuristic=lambda x: 0, collision=lambda x: False):
		visited = {start}
		import Queue
		queue = Queue.PriorityQueue()
		queue.put((heuristic(start), 0, np.random.rand(), start, [start]))
		while len(visited) < len(graph):
			if queue.empty():
				mindist = float('inf')
				best = None
				for v in visited:
					for i,q in enumerate(vertices):
						if i in visited:
							continue
						dist = np.linalg.norm(vertices[v] - q)
						if dist < mindist:
							mindist = dist
							best = v,i

				assert(best is not None)
				graph[best[0]].add(best[1])
				graph[best[1]].add(best[0])
				print(best)
				visited.add(best[1])
				queue.put((heuristic(best[1]), 0, np.random.rand(), best[1], [best[1]]))

			_, dist, _, vertex, path = queue.get()

			for next in graph[vertex] - visited:
				visited.add(next)

				if collision(next):
					continue

				if goal(next):
					return path + [next]
				else:
					newdist = dist + np.linalg.norm(vertices[vertex] - vertices[next])
					queue.put((newdist + heuristic(next), newdist, np.random.rand(), next, path + [next]))

		return None

	find_path(edges, 0, lambda x: False)

	return np.stack(vertices), edges

def plot(prm):
	vertices, edges = prm
	plt.plot(vertices[:,0], vertices[:,1], '.k')
	for i, e in enumerate(edges):
		for j in e:
			plt.plot([vertices[i][0], vertices[j][0]], [vertices[i][1], vertices[j][1]], '-k')
	plt.savefig('prm.png')

if __name__ == '__main__':
	prm = load_prm('./plans')

	pickle.dump(prm, open('./prm.pkl', 'wb'))

	plot(prm)

	pdb.set_trace()

