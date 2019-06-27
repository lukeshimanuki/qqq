import state

import math
import collections

def label(graph):
	GOAL_NUM = 5
	goalTables = [
		(i,obj)
		for i,obj in enumerate(graph.nodes)
		if obj[state.ObjType.Table] > .5
		and obj[len(state.ObjType) + state.ObjAttrs.goal] > .5
	]
	tables = [
		(i,obj)
		for i,obj in enumerate(graph.nodes)
		if obj[state.ObjType.Table] > .5
	]
	blocks = [
		(i,obj)
		for i,obj in enumerate(graph.nodes)
		if obj[state.ObjType.Block] > .5
	]
	floors = [
		(i,obj)
		for i,obj in enumerate(graph.nodes)
		if obj[state.ObjType.Floor] > .5
	]

	onTop = collections.defaultdict(list)
	for edge in graph.edges:
		if edge.edgeType == state.RelType.OnTop:
			onTop[edge.a].append(edge.b)
	availableBlocks = {
		i for i,block in blocks
		if i not in onTop or not any(g in onTop[i] for g in goalTables) # on goal
	}

	needed = GOAL_NUM - len({
		i for i,block in blocks
		if i in onTop and any(g in onTop[i] for g in goalTables)
	})

	if needed < 0:
		return 0.

	if needed > len(availableBlocks):
		return 1.

	xKey = len(state.ObjType)

	goalX = tables[0][1][xKey]

	distances = [abs(graph.nodes[blockIdx][xKey] - goalX) for blockIdx in availableBlocks]


	sigmoid = lambda x: 1. / (1 + math.e ** -x)
	return sum(sorted(distances)[:needed])

