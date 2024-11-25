import heapq  # For priority queue in A*
import numpy as np
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
LIGHT_THRESHOLD = 3500  # Threshold for classifying a cell as "too dark" (obstacle)
GRID_RESOLUTION = 0.1  # Grid resolution in meters (size of each cell)
GRID_ORIGIN_OFFSET = 0.8  # Offset from origin to the edge of the grid (in meters)

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


# Grid Mapping with Lighthouse Positioning and Light Detection
def map_path(grid_size, scf, pc, start, goal):
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

    path = a_star(grid_map, start, goal)  # Initial path planning with unmapped cells

    while path:
        for cell in path:
            row, col = cell
            if grid_map[row, col] != -1:  # Skip already mapped cells
                continue

            # Calculate real-world target position
            target_x = (col * GRID_RESOLUTION) - GRID_ORIGIN_OFFSET
            target_y = (row * GRID_RESOLUTION) - GRID_ORIGIN_OFFSET

            # Move to the target position
            pc.go_to(target_x, target_y)
            time.sleep(1)  # Allow time for stabilization

            # Map the current cell based on light level
            if light_level > LIGHT_THRESHOLD:
                grid_map[row, col] = 1  # Obstacle
                print(f"Cell ({row}, {col}) -> Obstacle (Light Level: {light_level})")
            else:
                grid_map[row, col] = 0  # Navigable
                print(f"Cell ({row}, {col}) -> Navigable (Light Level: {light_level})")

            # Debugging: print current position
            print(
                f"Expected Position: ({target_x:.2f}, {target_y:.2f}), "
                f"Actual Position: ({current_position[0]:.2f}, {current_position[1]:.2f})"
            )

        # Re-plan the path based on updated grid
        path = a_star(grid_map, start, goal)

    log_conf.stop()
    return grid_map



# Navigate using the A* Path
def navigate_path(path, pc):
    for cell in path:
        x = (cell[1] * GRID_RESOLUTION) - GRID_ORIGIN_OFFSET
        y = (cell[0] * GRID_RESOLUTION) - GRID_ORIGIN_OFFSET
        pc.go_to(x, y)
        time.sleep(1)


# Main Function
if __name__ == "__main__":
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        with PositionHlCommander(
            scf, default_height=FLYING_HEIGHT, default_velocity=DEFAULT_VELOCITY
        ) as pc:
            grid_size = (16, 16)  # 16x16 grid
            start = (0, 0)  # Origin
            goal = (15, 15)  # Bottom-right corner

            print("Mapping path with dynamic updates...")
            grid_map = map_path(grid_size, scf, pc, start, goal)

            print("Final Grid Map:")
            print(grid_map)

            # Navigate along the final planned path
            path = a_star(grid_map, start, goal)
            if path:
                print("Path found:", path)
                print("Navigating path...")
                navigate_path(path, pc)
            else:
                print("No path found!")

