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
FLYING_HEIGHT = 0.25      # Default flying height in meters
DEFAULT_VELOCITY = 0.2    # Default velocity in m/s
GRID_RESOLUTION = 0.2     # Grid resolution in meters (size of each cell)
GRID_ORIGIN_OFFSET = 0.8  # Offset from origin to the edge of the grid (in meters)
LIGHT_THRESHOLD = 4000    # Fixed threshold for white/black distinction

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


# Function to check if logging is active
def wait_for_logging(scf, timeout=10):
    log_conf = LogConfig(name="PositionAndLight", period_in_ms=100)
    log_conf.add_variable("stateEstimate.x", "float")
    log_conf.add_variable("stateEstimate.y", "float")
    log_conf.add_variable("stateEstimate.z", "float")
    log_conf.add_variable("motion.shutter", "uint16_t")

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(log_position_and_light_callback)

    # Start logging and wait for it to activate
    log_conf.start()

    print("Waiting for logging to start...")
    start_time = time.time()
    while light_level == 0:
        time.sleep(0.5)
        if time.time() - start_time > timeout:
            print("Logging timeout. Please check the drone.")
            log_conf.stop()
            raise RuntimeError("Failed to start logging within timeout.")

    print("Logging started successfully.")
    return log_conf


# Function to move the drone to a specific grid point
def move_to(pc, grid_x, grid_y):
    target_x = (grid_x * GRID_RESOLUTION) - GRID_ORIGIN_OFFSET
    target_y = (grid_y * GRID_RESOLUTION) - GRID_ORIGIN_OFFSET
    pc.go_to(target_x, target_y)
    time.sleep(1)  # Allow time for movement


# Map the grid and navigate white tiles
def navigate_white_tiles(grid_size, scf, pc):
    rows, cols = grid_size
    grid_map = np.full((rows, cols), -1)  # -1 indicates unmapped cells
    log_conf = LogConfig(name="PositionAndLight", period_in_ms=100)
    log_conf.add_variable("stateEstimate.x", "float")
    log_conf.add_variable("stateEstimate.y", "float")
    log_conf.add_variable("stateEstimate.z", "float")
    log_conf.add_variable("motion.shutter", "uint16_t")
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(log_position_and_light_callback)
    log_conf.start()

    # Initial visualization setup
    fig, ax = plt.subplots(figsize=(8, 8))

    # Start navigation from (0, 0)
    stack = [(0, 0)]  # DFS stack
    visited = set()

    while stack:
        current_cell = stack.pop()
        grid_x, grid_y = current_cell

        # Skip if already visited
        if current_cell in visited:
            continue
        visited.add(current_cell)

        # Move the drone to the current cell
        move_to(pc, grid_x, grid_y)

        # Check light level and map the current cell
        cell_value = 1 if light_level > LIGHT_THRESHOLD else 0
        grid_map[grid_y, grid_x] = cell_value
        print(f"Visited: {current_cell}, Light Level: {light_level}, Cell: {'Black' if cell_value == 1 else 'White'}")

        # Update visualization
        draw_grid(grid_map, ax)

        # If black tile is hit, backtrack to the last white tile
        if cell_value == 1:  # Black
            print("Black tile hit! Backtracking...")
            continue

        # Add neighboring white tiles to the stack
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Directions: Left, Right, Up, Down
            neighbor = (grid_x + dx, grid_y + dy)
            if 0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows:  # Within bounds
                if neighbor not in visited:  # Not already visited
                    stack.append(neighbor)

    log_conf.stop()
    return grid_map


# Draw the grid
def draw_grid(grid, ax, path=None):
    ax.clear()
    rows, cols = grid.shape
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

    # Draw gridlines and boundary
    ax.set_xticks(np.arange(0, cols + 1))
    ax.set_yticks(np.arange(0, rows + 1))
    ax.grid(color="grey", linestyle="-", linewidth=0.5)
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    plt.pause(0.1)  # Refresh the plot


# Insert the initialize_grid function here
def initialize_grid(scf):
    global current_position
    print("Initialization step:")

    print("Place the drone at the endpoint of the path and press Enter.")
    input("Press Enter when ready...")
    end_position = current_position[:2]

    print("Place the drone at the start of the path and press Enter.")
    input("Press Enter when ready...")
    start_position = current_position[:2]

    print(f"Start position: {start_position}, End position: {end_position}")

    return start_position, end_position

# A* Algorithm Implementation
def a_star(grid_map, start, goal):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    rows, cols = grid_map.shape
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows and grid_map[neighbor[1], neighbor[0]] == 0:
                tentative_g_score = g_score[current] + 1

                if tentative_g_score < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []  # Return an empty path if no path exists


# Function to make the drone follow a path
def follow_path(pc, path):
    for point in path:
        grid_x, grid_y = point
        move_to(pc, grid_x, grid_y)
        print(f"Moving to: {point}")
    print("Path navigation complete.")


# Insert the initialize_grid function here
def initialize_grid(scf):
    global current_position
    print("Initialization step:")

    print("Place the drone at the endpoint of the path and press Enter.")
    input("Press Enter when ready...")
    end_position = current_position[:2]

    print("Place the drone at the start of the path and press Enter.")
    input("Press Enter when ready...")
    start_position = current_position[:2]

    print(f"Start position: {start_position}, End position: {end_position}")

    return start_position, end_position

# Replace the relevant part of the main function
if __name__ == "__main__":
    print("Initializing grid mapping...")
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        # Wait for logging to start
        log_conf = wait_for_logging(scf)

        # Initialize the grid with start and end positions
        start_position, end_position = initialize_grid(scf)

        with PositionHlCommander(
            scf, default_height=FLYING_HEIGHT, default_velocity=DEFAULT_VELOCITY
        ) as pc:
            # Calculate the grid size dynamically or use a predefined size
            grid_size = (16, 16)  # Example: 16x16 grid

            print(f"Start position: {start_position}, End position: {end_position}")
            print("Navigating white tiles...")

            # Modify the navigation to use the initialized start_position
            grid_map = navigate_white_tiles(grid_size, scf, pc)

            print("Navigation complete. Visualizing final map...")
            plt.show()

            print("Landing drone...")
            pc.land()

        # Stop logging
        log_conf.stop()