from collections import deque
import heapq
from math import sqrt

class Board:

	# Possible swaps for different boards with different '_' index
	possible_swaps = {
		0: [1,3],
		1: [0,2,4],
		2: [1,5],
		3: [0,4,6],
		4: [3,5,1,7],
		5: [2,4,8],
		6: [3,7],
		7: [4,6,8],
		8: [5,7],
	}
	# Possible swaps and their "real world name"
	possible_moves = {
		0: {1:"Right",3:"Down"},
		1: {0:"Left",2:"Right",4:"Down"},
		2: {1:"Left",5:"Down"},
		3: {0:"Up",4:"Right",6:"Down"},
		4: {3:"Left",5:"Right",1:"Up",7:"Down"},
		5: {2:"Up",4:"Left",8:"Down"},
		6: {3:"Up",7:"Right"},
		7: {4:"Up",6:"Left",8:"Right"},
		8: {5:"Up",7:"Left"},
	} 
	# All possible goal states  ("x" is any number, because we only care about the first row)
	possible_goals = [
		["_","3","8",'x','x','x','x','x','x'],
		["_","4","7",'x','x','x','x','x','x'],
		["_","5","6",'x','x','x','x','x','x'],
		["1","2","8",'x','x','x','x','x','x'],
		["1","3","7",'x','x','x','x','x','x'],
		["1","4","6",'x','x','x','x','x','x'],
		["2","3","6",'x','x','x','x','x','x'],
		["2","4","5",'x','x','x','x','x','x'],
	]

	# Init
	def __init__(self, board):
		self.board = board

	# Expand current state
	def expand(self, state):
		children = []
		for idx, i in enumerate(state):
			if i == "_":
				for swapIdx in self.possible_swaps[idx]:
					children.append([self.swap(state, idx, swapIdx), self.possible_moves[idx][swapIdx]])
		return children

	# Swap empty cell with given adjecent cell
	def swap(self, state, idx1, idx2):
		state = state.copy()
		tempIdx1Val = state[idx1]
		state[idx1] = state[idx2]
		state[idx2] = tempIdx1Val
		return state

	# Check if first row sums up to 11
	def is_goal_state(self, state):
		state = state.copy()
		row_sum = 0
		for i in state[0:3]:
			if i != "_":
				row_sum += int(i)
		if row_sum == 11:
			return True
		return False

	# Check if the 8 puzzle game default goal state is satisfied (for test)
	def is_goal_state_default(self, state):
		state = state.copy()
		return state == ['_', '1', '2', '3', '4', '5', '6', '7', '8']
    
    # dfs method	
	def dfs(self):
		initial_state = self.board.copy()
		stack = [(initial_state, [], 0)]  # Stack includes state, path, and depth
		visited = set()

		while stack:
			state, path, depth = stack.pop()
	        
			visited.add(tuple(state))
	        
			if self.is_goal_state(state):
				return path  # Return the path to the goal state
	        
			for new_state, move in self.expand(state):
				if tuple(new_state) not in visited:
					stack.append((new_state, path + [move], depth + 1))  # Increase depth
	                    
		return None  # No solution found within the depth limit

	# bfs method
	def bfs(self):
		initial_state = self.board.copy()
		queue = deque([(initial_state, [])])
		visited = set()

		while queue:
			state, path = queue.popleft()
			visited.add(tuple(state))

			if self.is_goal_state(state):
				return path  # Return the path to the goal state
			for new_state, move in self.expand(state):
				if tuple(new_state) not in visited:
					queue.append((new_state, path + [move]))  # Add to queue with the move taken
		return None

	# Function to perform Uniform Cost Search
	def ucs(self):
		initial_state = self.board.copy()
		frontier = []  # Priority queue
		heapq.heappush(frontier, (0, initial_state, []))  # (cost, state, path)
		visited = set()

		while frontier:
			cost, state, path = heapq.heappop(frontier)
			visited.add(tuple(state))
			
			if self.is_goal_state(state):
				return path  # Return the path to the goal state

			for new_state, move in self.expand(state):
				if tuple(new_state) not in visited:
					new_cost = cost + 1  # Each move costs 1
					heapq.heappush(frontier, (new_cost, new_state, path + [move]))

		return None  # No solution found

	# Function to calculate the Manhattan distance heuristic for a state
	def manhattan_distance(self, state):
		min_distance = 99999999999
		# Loop through different goal states
		for goal_state in self.possible_goals:
			distance = 0
			# For each goal state calculate the distance
			for num in goal_state[0:3]:  # We don't need to calculate distance for the empty tile
				idx = state.index(num)
				goal_idx = goal_state.index(num)
				# Calculate the Manhattan distance
				distance += abs(idx % 3 - goal_idx % 3) + abs(idx // 3 - goal_idx // 3)
			# Change min_distance if distance is smaller to avoid overestimate
			if distance < min_distance:
				min_distance = distance
		return min_distance

	# Function to perform A* search
	def a_star_search(self):
		initial_state = self.board.copy()
		frontier = []
		heapq.heappush(frontier, (0, initial_state, []))  # (cost + heuristic, state, path)
		visited = set()
		cost_so_far = {tuple(initial_state): 0}  # Cost from start to the node

		while frontier:
			_, current_state, path = heapq.heappop(frontier)
			visited.add(tuple(current_state))

			if self.is_goal_state(current_state):
				return path  # Return the path to the goal state

			for new_state, move in self.expand(current_state):
				new_cost = cost_so_far[tuple(current_state)] + 1  # Assume cost for each move is 1
				if tuple(new_state) not in visited or new_cost < cost_so_far.get(tuple(new_state), float('inf')):
					cost_so_far[tuple(new_state)] = new_cost
					priority = new_cost + self.manhattan_distance(new_state)
					heapq.heappush(frontier, (priority, new_state, path + [move]))

		return None  # No solution found

	# Define the Straight Line Distance (Euclidean distance) heuristic function
	def straight_line_distance(self, state):
		min_distance = 99999999999
		# Loop through different goal states
		for goal_state in self.possible_goals:
			dist = 0 # To count distance
			for num in goal_state[0:3]:  # We don't need to calculate distance for the empty tile
				idx = state.index(num)
				goal_idx = goal_state.index(num)
				# Calculate the coordinates (x, y) for the current and goal positions
				x1, y1 = idx % 3, idx // 3
				x2, y2 = goal_idx % 3, goal_idx // 3
				# Calculate the Straight Line Distance
				dist += sqrt((x2 - x1)**2 + (y2 - y1)**2)
			# Change min_distance if distance is smaller to avoid overestimate
			if dist < min_distance:
				min_distance = dist
		return min_distance

	# Function to perform A* search with Straight Line Distance heuristic
	def a_star_search_sld(self):
		initial_state = self.board.copy()
		frontier = []
		heapq.heappush(frontier, (0, initial_state, []))  # (cost + heuristic, state, path)
		visited = set()
		cost_so_far = {tuple(initial_state): 0}  # Cost from start to the node

		while frontier:
			_, current_state, path = heapq.heappop(frontier)
			visited.add(tuple(current_state))

			if self.is_goal_state(current_state):
				return path  # Return the path to the goal state

			for new_state, move in self.expand(current_state):
				new_cost = cost_so_far[tuple(current_state)] + 1  # Assume cost for each move is 1
				if tuple(new_state) not in visited or new_cost < cost_so_far.get(tuple(new_state), float('inf')):
					cost_so_far[tuple(new_state)] = new_cost
					priority = new_cost + self.straight_line_distance(new_state)
					heapq.heappush(frontier, (priority, new_state, path + [move]))

		return None  # No solution found


def read_input_file(file_path):
    with open('input.txt', 'r') as file:
        state = file.read().strip().split(',')
    return state

initial_state = read_input_file('input.txt')

board = Board(initial_state)
print(board.dfs())
print(board.bfs())
print(board.ucs())
print(board.a_star_search())
print(board.a_star_search_sld())