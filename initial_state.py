import random
import graph
import state
import generated_examples

example_idx = 0


def fetch_prelabelled():
	global example_idx
	while example_idx < len(generated_examples.examples):
		example = generated_examples.examples[example_idx]
		example_idx += 1

		if len(example) == 2:
			return example


def fetch_trajectory_from_training_data():
	return None


def create_node(entity):
	pass


def create_edge(edge):
	pass


def generate_prelabelled_from_mover_env():
	example = fetch_trajectory_from_training_data()
	states = example['states']
	operators = example['operators']
	rewards = example['rewards']
	state_primes = example['state_primes']

	graph_for_trajectory = []
	for s, o, r, _ in zip(states, operators, rewards, state_primes):
		nodes = [create_node(entity) for entity in s.entities]
		edges = [create_edge(predicates) for predicates in s.predicates]
		state_graph = graph.Graph(nodes, edges)
		graph_for_trajectory.append(state_graph)


def generate_prelabelled(numTables=-1, numBlocks=-1, example=fetch_prelabelled()):
	# fetches an initial_state,plan_cost pair from generated_examples.examples whenever it is called

	# tables_pos is a list of (x,y) locations of the table
	# blocks_pos is a list of (x,y) locations of the blocks
	(tables_pos, blocks_pos), label = example

	numTables = len(tables_pos)
	numBlocks = len(blocks_pos)
	assert(numTables >= 2)

	# TODO: extract from data
	w = .5
	h = .7
	bw = .07
	bh = .1
	mass = 1.

	# create graph nodes for each object in the scene
	tables = [
		state.object(state.ObjType.Table,
			x=x,
			y=y,
			width=w,
			height=h,
			goal=1. if i == 0 else 0.,
		)
		for i, (x, y) in enumerate(tables_pos)
	]

	blocks = [
		state.object(state.ObjType.Block,
			x=x,
			y=y,
			width=bw,
			height=bh,
			weight=mass,
		)
		for i, (x, y) in enumerate(blocks_pos)
	]
	floor = state.object(state.ObjType.Floor, height=0.)

	def which_table(block_num):
		bpos = blocks_pos[block_num]
		for i, tpos in enumerate(tables_pos):
			if abs(bpos[0] - tpos[0]) <= w/2 and abs(bpos[1] - tpos[1]) <= w/2:
				return i

	# create edge feature between each object and a table or floor
	edges = [
		edge
		for i, block in enumerate(blocks)
		for edge in state.onTop(numTables + i, which_table(i))
	] + [
		edge
		for i, table in enumerate(tables)
		for edge in state.onTop(i, numTables + numBlocks)
	]
	return graph.Graph(tables + blocks + [floor], edges), label / 100000.


def generate(numTables, numBlocks):
	assert(numTables >= 2)

	tables = [
		state.object(state.ObjType.Table,
			x=random.random(),
			y=random.random(),
			width=random.random(),
			height=random.random(),
			goal=1. if i == 0 else 0.,
		)
		for i in range(numTables)
	]

	blocks = [
		state.object(state.ObjType.Block,
			x=random.random(),
			y=random.random(),
			width=random.random(),
			height=random.random(),
		)
		for i in range(numBlocks)
	]

	floor = state.object(state.ObjType.Floor, y=0.)

	edges = [
		edge
		for i, block in enumerate(blocks)
		for edge in state.onTop(numTables + i, random.randint(0, numTables - 1))
	] + [
		edge
		for i, table in enumerate(tables)
		for edge in state.onTop(i, numTables + numBlocks)
	]

	return graph.Graph(tables + blocks + [floor], edges)

