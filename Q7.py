import matplotlib.pyplot as plt
from queue import Queue
import random
import time

# Define the grid size
grid_size = 10
grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

# Mark start (S) and target (T) points
start = (0, 0)  # Start point at top-left
target = (grid_size - 1, grid_size - 1)  # Target point at bottom-right

# Function to get neighbors (up, right, down, left, and diagonals)
def get_neighbors(node):
    x, y = node
    neighbors = []
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]  # Up, Right, Bottom, Left, and Diagonals
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < grid_size and 0 <= ny < grid_size:  # Ensure within bounds
            neighbors.append((nx, ny))
    return neighbors

# Function to visualize the grid
def visualize_grid(frontier, explored, path=None):
    plt.clf()  # Clear the current figure to avoid overlapping
    
    # Create a new figure for each update
    plt.figure(figsize=(8, 8))
    ax = plt.gca()
    ax.set_xticks(range(grid_size))
    ax.set_yticks(range(grid_size))
    ax.grid(True)

    # Mark the start and target points
    plt.text(start[1], start[0], "S", fontsize=12, color="blue", ha="center", va="center")
    plt.text(target[1], target[0], "T", fontsize=12, color="green", ha="center", va="center")

    # Mark the frontier and explored nodes
    for node in frontier:
        plt.text(node[1], node[0], "-1", fontsize=12, color="red", ha="center", va="center")
    for node in explored:
        plt.text(node[1], node[0], "X", fontsize=12, color="yellow", ha="center", va="center")

    # Mark the final path
    if path:
        for node in path:
            plt.text(node[1], node[0], "*", fontsize=12, color="green", ha="center", va="center")

    # Add obstacles (represented by 'X' in grid)
    for i in range(grid_size):
        for j in range(grid_size):
            if grid[i][j] == 'X':  # Obstacle
                plt.plot(j, i, marker="s", color="black", markersize=15)

    # Ensure the plot displays correctly
    plt.xlim(-0.5, grid_size - 0.5)
    plt.ylim(grid_size - 0.5, -0.5)
    plt.gca().invert_yaxis()  # Invert y-axis to match grid layout (top-left to bottom-right)

    # Draw the figure and pause for animation effect
    plt.draw()
    plt.pause(0.1)  # Pause for animation effect

# BFS Algorithm
def bfs(grid, start, target):
    frontier = Queue()
    frontier.put(start)
    explored = set()
    explored.add(start)
    parent_map = {start: None}  # For reconstructing the path

    while not frontier.empty():
        current = frontier.get()

        # If goal is found, reconstruct the path
        if current == target:
            path = []
            while current:
                path.append(current)
                current = parent_map[current]
            path.reverse()  # Reverse the path to go from start to goal
            return path

        # Expand neighbors
        for neighbor in get_neighbors(current):
            if neighbor not in explored:
                explored.add(neighbor)
                parent_map[neighbor] = current
                frontier.put(neighbor)

        # Visualize the process (frontier and explored nodes)
        visualize_grid(list(frontier.queue), explored)

        # Add dynamic obstacles during BFS exploration
        grid = add_dynamic_obstacles(grid)
        
        time.sleep(0.5)  # Pause for a brief moment to show progress

    return None  # If no path found

# Add dynamic obstacles to the grid during the search
def add_dynamic_obstacles(grid, probability=0.1):
    """
    Adds dynamic obstacles to the grid with a given probability.
    """
    rows = len(grid)
    cols = len(grid[0])

    # Add random obstacles to the grid
    for i in range(rows):
        for j in range(cols):
            if random.random() < probability:
                grid[i][j] = 'X'  # Marking as obstacle
    return grid

# Run BFS with dynamic obstacles
def run_bfs_with_dynamic_obstacles():
    # Create a new grid with start and target points
    grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]
    
    # Run BFS with dynamic obstacles
    path = bfs(grid, start, target)
    
    if path:
        print("Path found:", path)
    else:
        print("No path found.")

# Initialize the figure and run the BFS with dynamic obstacles
plt.ion()  # Turn on interactive mode for real-time updates
run_bfs_with_dynamic_obstacles()  # Execute the pathfinding algorithm
plt.ioff()  # Turn off interactive mode to prevent auto-closing
plt.show()  # Keep the plot open after the algorithm finishes
