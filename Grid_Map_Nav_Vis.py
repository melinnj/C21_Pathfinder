
#Initialisation:
#Prompts the user to record light thresholds for white and black cells.
#Waits for the user to place the drone in view of the Lighthouse stations and press "E" to execute.
#Graphical Window:
#Displays the grid during mapping and highlights the shortest path once found.
#Mapping:
#Maps cells based on thresholds:
##Light Level>black threshold:Obstacle
#Light Level<white threshold:Navigable
#Pathfinding:Finds and highlights the shortest path.

import heapq  # For priority queue in A*
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
import logging
import time

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default="radio://0/2/2M/EE5C21CF01")

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Drone parameters
FLYING_HEIGHT = 0.2  # Default flying height in meters
DEFAULT_VELOCITY = 0.2  # Default velocity in m/s
GRID_RESOLUTION = 0.1  # Grid resolution in meters (size of each cell)
GRID_ORIGIN_OFFSET = 0.8  # Offset from origin to the edge of the grid (in meters)
light_white_threshold = 0
light_black_threshold = 0

# Initialize the low-level drivers
cflib.crtp.init_drivers()

# Global variables for position and light level
current_position = [0.0, 0.0, 0.0]
light_level = 0


# Lighthouse position and light detection callback
def log_position_and_light_callback(timestamp, data, log_config):
    global current_position, light_level
    current_position[0] = data["stateEstimate.x"]
    current_position[1] = data["stateEstimate.y"]
    current_position[2] = data["stateEstimate.z"]
    light_level = data["motion.shutter"]


# A* Algorithm
def a_star(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))  # Priority queue with (cost, cell)
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        neighbors = get_neighbors(current, rows, cols)
        for neighbor in neighbors:
            if grid[neighbor[0]][neighbor[1]] == 1:  # Obstacle
                continue

            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance


def get_neighbors(cell, rows, cols):
    x, y = cell
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    neighbors = [(x + dx, y + dy) for dx, dy in directions]
    return [(nx, ny) for nx, ny in neighbors if 0 <= nx < rows and 0 <= ny < cols]


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]


# Draw the grid
def draw_grid(grid, path=None):
    rows, cols = grid.shape
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)

    # Draw grid cells
    for row in range(rows):
        for col in range(cols):
            cell_value = grid[row, col]
            color = "grey"  # Default: unmapped
            if cell_value == 0:
                color = "white"  # Navigable
            elif cell_value == 1:
                color = "black"  # Obstacle
            ax.add_patch(Rectangle((col, rows - 1 - row), 1, 1, color=color))

    # Highlight the path
    if path:
        for cell in path:
            row, col = cell
            ax.add_patch(Rectangle((col, rows - 1 - row), 1, 1, color="blue", alpha=0.5))

    # Draw gridlines and boundary
    ax.set_xticks(np.arange(0, cols + 1))
    ax.set_yticks(np.arange(0, rows + 1))
    ax.grid(color="grey", linestyle="-", linewidth=0.5)
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    plt.show()


# Map the grid
def map_grid(grid_size, scf, pc):
    rows, cols = grid_size
    grid_map = np.full((rows, cols), -1)  # -1 indicates unmapped cells

    # Configure logging for position and light detection
    log_conf = LogConfig(name="PositionAndLight", period_in_ms=100)
    log_conf.add_variable("stateEstimate.x", "float")
    log_conf.add_variable("stateEstimate.y", "float")
    log_conf.add_variable("stateEstimate.z", "float")
    log_conf.add_variable("motion.shutter", "uint16_t")

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(log_position_and_light_callback)
    log_conf.start()

    for row in range(rows):
        for col in range(cols):
            # Calculate real-world target position
            target_x = (col * GRID_RESOLUTION) - GRID_ORIGIN_OFFSET
            target_y = (row * GRID_RESOLUTION) - GRID_ORIGIN_OFFSET

            # Move the drone to the target position
            pc.go_to(target_x, target_y)
            time.sleep(1)

            # Map the cell based on light level
            if light_level > light_black_threshold:
                grid_map[row, col] = 1  # Obstacle
            elif light_level < light_white_threshold:
                grid_map[row, col] = 0  # Navigable
            else:
                grid_map[row, col] = -1  # Unmapped

    log_conf.stop()
    return grid_map


# Initialize light thresholds
def initialize_light_thresholds(scf):
    global light_white_threshold, light_black_threshold

    print("Hold the drone 10cm above a white cell and press Enter.")
    input()
    light_white_threshold = light_level
    print(f"Recorded white threshold: {light_white_threshold}")

    print("Hold the drone 10cm above a black cell and press Enter.")
    input()
    light_black_threshold = light_level
    print(f"Recorded black threshold: {light_black_threshold}")

    print("Place the drone in view of the Lighthouse stations and press 'E' to execute.")
    while input().strip().upper() != "E":
        pass


# Main Function
if __name__ == "__main__":
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        with PositionHlCommander(
            scf, default_height=FLYING_HEIGHT, default_velocity=DEFAULT_VELOCITY
        ) as pc:
            grid_size = (16, 16)  # 16x16 grid

            print("Initializing light thresholds...")
            initialize_light_thresholds(scf)

            print("Mapping grid...")
            grid_map = map_grid(grid_size, scf, pc)

            print("Grid mapped. Visualizing...")
            draw_grid(grid_map)

            start = (0, 0)  # Origin
            goal = (15, 15)  # Bottom-right corner

            print("Computing path using A*...")
            path = a_star(grid_map, start, goal)
            if path:
                print("Path found. Visualizing...")
                draw_grid(grid_map, path)
                print("Navigating path...")
                navigate_path(path, pc)
            else:
                print("No path found!")
