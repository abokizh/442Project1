from collections import deque
import heapq
from math import sqrt

goal_state = ['_', '1', '2', '3', '4', '5', '6', '7', '8'] # declaring the goal state to solve the puzzle
# We are using a DFS with depth limit for the 8-puzzle problem, due to computing time constraints. The limit can be modified by the user at any time.

def dfs(initial_state, limit=30):
    stack = [(initial_state, [], 0)]  # Stack includes state, path, and depth
    visited = set()
    ex = 0
    while stack:
        state, path, depth = stack.pop()
        
        if depth > limit:  # Depth-limited search
            continue
        
        visited.add(tuple(state))
        
        if state == goal_state:
            #print(ex)
            return path  # Return the path to the goal state
        
        if depth < limit:
            ex+=1
            for move, new_state in get_successors(state):
                if tuple(new_state) not in visited:
                    stack.append((new_state, path + [move], depth + 1))  # Increase depth
                    
    return None  # No solution found within the depth limit

def get_successors(state):
    def swap(state, i, j):
        new_state = list(state)
        new_state[i], new_state[j] = new_state[j], new_state[i]
        return new_state

    index = state.index('_')  # Find the index of the empty space
    successors = []

    # Mapping the possible moves to index changes
    moves = {
        'Left': -1,
        'Up': -3,
        'Right': 1,
        'Down': 3
    }
    
    for move, idx_change in moves.items():
        new_index = index + idx_change
        if 0 <= new_index < 9:  # Check if the new index is within bounds
            # Check if the move is valid within the puzzle boundaries
            if (index % 3 == 0 and move == 'Left') or (index % 3 == 2 and move == 'Right'):
                continue
            successors.append((move, swap(state, index, new_index)))
            
    return successors

# Function to read the initial state from the input file
def read_input_file(file_path):
    with open('input.txt', 'r') as file:
        state = file.read().strip().split(',')
    return state

# Format the solution path according to the project specifications
def format_solution_path(state, path):
    formatted_path = []
    state = state.copy()  # Create a copy to avoid mutating the original state
    index = state.index('_')  # Find the index of the empty space

    # Define movements in terms of changes in the index
    move_indices = {
        'D': -3,
        'U': 3,
        'R': -1,
        'L': 1
    }

    # Apply each move in the path to the state
    for move in path:
        # Calculate the index of the tile that needs to move
        tile_index = index + move_indices[move]
        # Append the tile number and direction to the formatted path
        formatted_path.append(f"{state[tile_index]}{move}")
        # Swap the empty space with the tile
        state[index], state[tile_index] = state[tile_index], state[index]
        # Update the index of the empty space
        index = tile_index

    return ''.join(formatted_path)

# Read initial state from file
initial_state = read_input_file('input.txt')

# Execute DFS with depth limit
solution_path_dfs = dfs(initial_state, limit=90)

# If a solution is found, format it according to the project specifications
if solution_path_dfs:
    # Adjust the solution path to use single-letter direction keys
    direction_mapping = {'Up': 'D', 'Down': 'U', 'Left': 'R', 'Right': 'L'}
    adjusted_solution_path = [direction_mapping[move] for move in solution_path_dfs]
    # Format the path
    formatted_solution_dfs = format_solution_path(initial_state, adjusted_solution_path)
    formatted_solution_output_dfs = f"The solution of Q1.1.a is:\n{formatted_solution_dfs}"
else:
    formatted_solution_output_dfs = "No solution found within the depth limit"

# Function to perform Breadth-First Search
def bfs(initial_state):
    queue = deque([(initial_state, [])])  # Queue for FIFO order, with path
    visited = set()
    ex = 0
    while queue:
        state, path = queue.popleft()
        visited.add(tuple(state))

        if state == goal_state:
            #print(ex)
            return path  # Return the path to the goal state
        ex+=1
        for move, new_state in get_successors(state):
            if tuple(new_state) not in visited:
                queue.append((new_state, path + [move]))  # Add to queue with the move taken

    return None  # No solution found
# ...

# Read initial state from file
initial_state = read_input_file('input.txt')

# Execute BFS
solution_path_bfs = bfs(initial_state)

# If a solution is found, format it.
if solution_path_bfs:
    # Adjust the solution path to use single-letter direction keys
    adjusted_solution_path_bfs = [direction_mapping[move] for move in solution_path_bfs]
    # Format the path
    formatted_solution_bfs = format_solution_path(initial_state, adjusted_solution_path_bfs)
    formatted_solution_output_bfs = f"The solution of Q1.1.b is:\n{formatted_solution_bfs}"
else:
    formatted_solution_output_bfs = "No solution found with BFS"

# Function to perform Uniform Cost Search
def ucs(initial_state):
    frontier = []  # Priority queue
    heapq.heappush(frontier, (0, initial_state, []))  # (cost, state, path)
    visited = set()
    ex=0
    while frontier:
        cost, state, path = heapq.heappop(frontier)
        visited.add(tuple(state))

        if state == goal_state:
            #print(ex)
            return path  # Return the path to the goal state
        ex+=1
        for move, new_state in get_successors(state):
            if tuple(new_state) not in visited:
                new_cost = cost + 1  # Each move costs 1
                heapq.heappush(frontier, (new_cost, new_state, path + [move]))

    return None  # No solution found

# Execute Uniform Cost Search
solution_path_ucs = ucs(initial_state)

# If a solution is found, format it according to the project specifications
if solution_path_ucs:
    # Adjust the solution path to use single-letter direction keys
    adjusted_solution_path_ucs = [direction_mapping[move] for move in solution_path_ucs]
    # Format the path
    formatted_solution_ucs = format_solution_path(initial_state, adjusted_solution_path_ucs)
    formatted_solution_output_ucs = f"The solution of Q1.1.c is:\n{formatted_solution_ucs}"
else:
    formatted_solution_output_ucs = "No solution found with UCS"

    # Function to calculate the Manhattan distance heuristic for a state
def manhattan_distance(state, goal_state):
    distance = 0
    for num in '12345678':  # We don't need to calculate distance for the empty tile
        idx = state.index(num)
        goal_idx = goal_state.index(num)
        # Calculate the Manhattan distance
        distance += abs(idx % 3 - goal_idx % 3) + abs(idx // 3 - goal_idx // 3)
    return distance

    # Function to perform A* search
def a_star_search(initial_state):
    frontier = []
    heapq.heappush(frontier, (0, initial_state, []))  # (cost + heuristic, state, path)
    visited = set()
    cost_so_far = {tuple(initial_state): 0}  # Cost from start to the node
    ex=0
    while frontier:
        _, current_state, path = heapq.heappop(frontier)
        visited.add(tuple(current_state))

        if current_state == goal_state:
            #print(ex)
            return path  # Return the path to the goal state
        ex+=1
        for move, new_state in get_successors(current_state):
            new_cost = cost_so_far[tuple(current_state)] + 1  # Assume cost for each move is 1
            if tuple(new_state) not in visited or new_cost < cost_so_far.get(tuple(new_state), float('inf')):
                cost_so_far[tuple(new_state)] = new_cost
                priority = new_cost + manhattan_distance(new_state, goal_state)
                heapq.heappush(frontier, (priority, new_state, path + [move]))

    return None  # No solution found

# Execute A* search
solution_path_a_star = a_star_search(initial_state)

# If a solution is found, format it.
if solution_path_a_star:
    # Adjust the solution path to use single-letter direction keys
    adjusted_solution_path_a_star = [direction_mapping[move] for move in solution_path_a_star]
    # Format the path
    formatted_solution_a_star = format_solution_path(initial_state, adjusted_solution_path_a_star)
    formatted_solution_output_a_star = f"The solution of Q1.1.d is:\n{formatted_solution_a_star}"
else:
    formatted_solution_output_a_star = "No solution found with A*"

    # Define the Straight Line Distance (Euclidean distance) heuristic function
def straight_line_distance(state, goal_state):
    dist = 0 # To count distance
    for num in '12345678':  # We don't need to calculate distance for the empty tile
        idx = state.index(num)
        goal_idx = goal_state.index(num)
        # Calculate the coordinates (x, y) for the current and goal positions
        x1, y1 = idx % 3, idx // 3
        x2, y2 = goal_idx % 3, goal_idx // 3
        # Calculate the Straight Line Distance
        dist += sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist

    # Function to perform A* search with Straight Line Distance heuristic
def a_star_search_sld(initial_state):
    frontier = []
    heapq.heappush(frontier, (0, initial_state, []))  # (cost + heuristic, state, path)
    visited = set()
    cost_so_far = {tuple(initial_state): 0}  # Cost from start to the node
    ex=0
    while frontier:
        _, current_state, path = heapq.heappop(frontier)
        visited.add(tuple(current_state))

        if current_state == goal_state:
            #print(ex)
            return path  # Return the path to the goal state
        ex+=1
        for move, new_state in get_successors(current_state):
            new_cost = cost_so_far[tuple(current_state)] + 1  # Assume cost for each move is 1
            if tuple(new_state) not in visited or new_cost < cost_so_far.get(tuple(new_state), float('inf')):
                cost_so_far[tuple(new_state)] = new_cost
                priority = new_cost + straight_line_distance(new_state, goal_state)
                heapq.heappush(frontier, (priority, new_state, path + [move]))

    return None  # No solution found

# Execute A* search with Straight Line Distance heuristic
solution_path_a_star_sld = a_star_search_sld(initial_state)

# If a solution is found, format it
if solution_path_a_star_sld:
    # Adjust the solution path to use single-letter direction keys
    adjusted_solution_path_a_star_sld = [direction_mapping[move] for move in solution_path_a_star_sld]
    # Format the path
    formatted_solution_a_star_sld = format_solution_path(initial_state, adjusted_solution_path_a_star_sld)
    formatted_solution_output_a_star_sld = f"The solution of Q1.1.e is:\n{formatted_solution_a_star_sld}"
else:
    formatted_solution_output_a_star_sld = "No solution found with A* using Straight Line Distance"

# Print the formatted solution output for both DFS, BFS, UCS, A* and A*_sld
print(formatted_solution_output_dfs)
print(formatted_solution_output_bfs)
print(formatted_solution_output_ucs)
print(formatted_solution_output_a_star)
print(formatted_solution_output_a_star_sld)

#For questions 1.2 onwards just modify the previous code:
'''
def dfs_with_node_count(initial_state, limit=50):
    stack = [(initial_state, [], 0)] 
    visited = set()
    node_expansions_dfs = 0  # Counter for the number of node expansions

    while stack:
        state, path, depth = stack.pop()
        if depth > limit:
            continue
        if tuple(state) in visited:
            continue
        visited.add(tuple(state))
        
        # Increment node expansions when a new node is visited
        node_expansions_dfs += 1

        if state == goal_state:
            return path, node_expansions_dfs  # Return the path and the number of node expansions

        # Generate successors and add them to the stack
        if depth < limit:
            for move, new_state in get_successors(state):
                # Add the new state to the stack if it's not visited
                if tuple(new_state) not in visited:
                    stack.append((new_state, path + [move], depth + 1))

    # If no solution is found within the depth limit
    return None, node_expansions_dfs

dfs_solution, node_expansions_count_dfs = dfs_with_node_count(initial_state)
print(f"Node Expansions DFS: {node_expansions_count_dfs}")

def bfs_with_node_count(initial_state):
    queue = deque([(initial_state, [])])  
    visited = set() 
    node_expansions = 0 

    while queue:
        current_state, path = queue.popleft() 
        visited.add(tuple(current_state))  
        if current_state == goal_state:
            return path, node_expansions  
        # Increment the node expansions counter when a node is expanded
        node_expansions += 1

        for move, new_state in get_successors(current_state):
            if tuple(new_state) not in visited:
                queue.append((new_state, path + [move]))  # Enqueue the new state

    return None, node_expansions

bfs_solution, node_expansions_count_bfs = bfs_with_node_count(initial_state)
print(f"Node Expansions BFS: {node_expansions_count_bfs}")

def ucs_with_node_count(initial_state):
    frontier = [] 
    heapq.heappush(frontier, (0, initial_state, [])) 
    visited = set()
    node_expansions = 0 

    while frontier:
        cost, state, path = heapq.heappop(frontier)

        if tuple(state) in visited:
            continue
        visited.add(tuple(state))
        
        node_expansions += 1  # Increment the counter when a new node is visited

        if state == goal_state:
            return path, node_expansions  # Return the path and the number of node expansions

        for move, new_state in get_successors(state):
            if tuple(new_state) not in visited:
                new_cost = cost + 1  # Each move costs 1
                heapq.heappush(frontier, (new_cost, new_state, path + [move]))

    return None, node_expansions  # If no solution is found

ucs_solution, ucs_node_expansions = ucs_with_node_count(initial_state)
print(f"Node Expansions UCS: {ucs_node_expansions}")

def a_star_node_count_manhattan(initial_state):
    frontier = [] 
    heapq.heappush(frontier, (0, initial_state, []))  
    visited = set()
    cost_so_far = {tuple(initial_state): 0}
    node_expansions = 0 

    while frontier:
        _, current_state, path = heapq.heappop(frontier)

        if tuple(current_state) in visited:
            continue
        visited.add(tuple(current_state))

        node_expansions += 1 

        if current_state == goal_state:
            return path, node_expansions  

        for move, new_state in get_successors(current_state):
            new_cost = cost_so_far[tuple(current_state)] + 1  # Each move costs 1
            if tuple(new_state) not in visited or new_cost < cost_so_far.get(tuple(new_state), float('inf')):
                cost_so_far[tuple(new_state)] = new_cost
                priority = new_cost + manhattan_distance(new_state, goal_state)
                heapq.heappush(frontier, (priority, new_state, path + [move]))

    return None, node_expansions

a_star_manhattan_solution, a_star_manhattan_node_expansions = a_star_node_count_manhattan(initial_state)
print(f"Node Expansions A* Manhattan: {a_star_manhattan_node_expansions}")

def a_star_node_count_sld(initial_state):
    frontier = [] 
    heapq.heappush(frontier, (0, initial_state, []))
    visited = set()
    cost_so_far = {tuple(initial_state): 0}
    node_expansions = 0 

    while frontier:
        _, current_state, path = heapq.heappop(frontier)

        if tuple(current_state) in visited:
            continue
        visited.add(tuple(current_state))

        node_expansions += 1 

        if current_state == goal_state:
            return path, node_expansions 
        for move, new_state in get_successors(current_state):
            new_cost = cost_so_far[tuple(current_state)] + 1  # Each move costs 1
            if tuple(new_state) not in visited or new_cost < cost_so_far.get(tuple(new_state), float('inf')):
                cost_so_far[tuple(new_state)] = new_cost
                priority = new_cost + straight_line_distance(new_state, goal_state)
                heapq.heappush(frontier, (priority, new_state, path + [move]))

    return None, node_expansions
a_star_sld_solution, a_star_sld_node_expansions = a_star_node_count_sld(initial_state)
print(f"Node Expansions A* SLD: {a_star_sld_node_expansions}")

'''
