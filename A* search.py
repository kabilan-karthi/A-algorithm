import heapq

# Function to calculate the Manhattan distance heuristic
def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

# Function to perform A* search
def astar_search(graph, start, goal):
    open_set = []
    closed_set = set()

    # Create a priority queue with the start node
    heapq.heappush(open_set, (0, start))

    # Initialize the cost dictionary
    g_scores = {start: 0}

    # Initialize the path dictionary
    parents = {start: None}

    while open_set:
        # Pop the node with the lowest cost from the priority queue
        current_cost, current_node = heapq.heappop(open_set)

        # Check if the goal node is reached
        if current_node == goal:
            path = []
            while current_node:
                path.append(current_node)
                current_node = parents[current_node]
            path.reverse()
            return path

        # Add the current node to the closed set
        closed_set.add(current_node)

        # Explore the neighbors of the current node
        for neighbor in graph[current_node]:
            # Calculate the cost from the start node to the neighbor
            g_score = g_scores[current_node] + graph[current_node][neighbor]

            # Check if the neighbor is already explored or the new path has a lower cost
            if neighbor in closed_set or (neighbor in g_scores and g_score >= g_scores[neighbor]):
                continue

            # Update the cost and parent of the neighbor
            g_scores[neighbor] = g_score
            parents[neighbor] = current_node

            # Calculate the total cost of the path using the heuristic function
            f_score = g_score + heuristic(neighbor, goal)

            # Push the neighbor into the priority queue
            heapq.heappush(open_set, (f_score, neighbor))

    # No path found
    return None

# Example usage
graph = {
    'A': {'B': 5, 'C': 3},
    'B': {'D': 2},
    'C': {'D': 6, 'E': 4},
    'D': {'E': 1},
    'E': {}
}

start_node = 'A'
goal_node = 'E'

path = astar_search(graph, start_node, goal_node)

if path:
    print("Path found:", path)
else:
    print("No path found.")
