import random
import numpy as np
import heapq
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import pickle

# Map size
WIDTH, HEIGHT = 16, 16

# Define the direction up down left right
directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

def generate_maze():
    # Initialise the map, all of them is black (1)
    maze = np.ones((HEIGHT, WIDTH), dtype=int)

    # Make sure the start and end is different
    while True:
        start_x, start_y = random.randrange(0, WIDTH, 2), random.randrange(0, HEIGHT, 2)
        end_x, end_y = random.randrange(0, WIDTH, 2), random.randrange(0, HEIGHT, 2)
        if (start_x, start_y) != (end_x, end_y):
            break

    # Generate the map vy using Recursive backtracking algorithm
    def carve_passages_from(x, y):
        maze[y][x] = 0  # Flat as Route
        dirs = directions[:]
        random.shuffle(dirs)
        for dx, dy in dirs:
            nx, ny = x + dx * 2, y + dy * 2
            if 0 <= nx < WIDTH and 0 <= ny < HEIGHT and maze[ny][nx] == 1:
                maze[y + dy][x + dx] = 0  # Break through the wall
                carve_passages_from(nx, ny)

    carve_passages_from(start_x, start_y)

    # Add divergence and convergence
    for _ in range(10):  # Randomly break through ten walls
        x = random.randrange(1, WIDTH - 1)
        y = random.randrange(1, HEIGHT - 1)
        maze[y][x] = 0

    # Call A* algorithm
    start = (start_x, start_y)
    end = (end_x, end_y)
    path = a_star_search(maze, start, end)

    if path:
        print(f"Found the path, there are {len(path)} nodes: {path}")
        # Plot the map and route
        draw_maze_and_path(maze, path, start, end)
        # Save
        save_path(path)
    else:
        print("Not found the path")

    print(f"Start：({start_x}, {start_y}), End：({end_x}, {end_y})")

def a_star_search(maze, start, end):
    """
    Using A* algorithm to find the shortest route form start to end
    """
    # Define the heuristic function, here using the Manhattan distance
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # Initializes the open and closed lists
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        current_priority, current = heapq.heappop(open_set)

        if current == end:
            # Rebuilt the route
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        x, y = current
        for dx, dy in directions:
            neighbor = (x + dx, y + dy)
            nx, ny = neighbor
            # Check that the neighbor is within the maze range and is a passable path
            if 0 <= ny < HEIGHT and 0 <= nx < WIDTH and maze[ny][nx] == 0:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, end)
                    heapq.heappush(open_set, (f_score, neighbor))

    return None  # Not find the route

def draw_maze_and_path(maze, path, start, end):
    """
    Plot the map and route
    """
    # Define the colour：0-white(path)，1-black(boarder)
    cmap = ListedColormap(['white', 'black'])

    plt.figure(figsize=(6,6))
    plt.imshow(maze, cmap=cmap, origin='lower')

    # Add gridlines to form a black border of the path
    plt.grid(which='both', color='black', linewidth=2)
    plt.xticks(np.arange(-0.5, WIDTH, 1), [])
    plt.yticks(np.arange(-0.5, HEIGHT, 1), [])

    # Draw markers at the start and end points
    start_x, start_y = start
    end_x, end_y = end
    plt.scatter(start_x, start_y, marker='o', color='black', s=200)  # Black circular start point
    plt.scatter(end_x, end_y, marker='^', color='black', s=200)     # Black triangle end point

    # Plot the path
    if path:
        # Decompression path coordinates
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, color='red', linewidth=2)  # Rad line is path
    else:
        print("Not found the path")

    # Remove axis
    plt.axis('off')
    plt.show()

def save_path(path):
    """
    Save the path
    """
    with open('path_data.pkl', 'wb') as f:
        pickle.dump(path, f)
    print("Path has already saved: 'path_data.pkl'。")




if __name__ == "__main__":
    generate_maze()
