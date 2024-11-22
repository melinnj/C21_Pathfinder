import pygame
import logging
import heapq
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

# Initialize constants
GRID_SIZE = 16
CELL_SIZE = 40  # Size of each grid cell in pixels
WINDOW_SIZE = GRID_SIZE * CELL_SIZE
FPS = 10  # Frames per second for visualization
#uri = 'radio://0/12/2M/EE5C21CF11'  # Replace with your drone's URI
uri = 'radio://0/2/2M/EE5C21CF01'

# Initialize logging
logging.basicConfig(level=logging.ERROR)

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption("Drone Mapping and Pathfinding")
clock = pygame.time.Clock()

# Initialize grid
grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]  # 0 = unmapped, 1 = white, -1 = black, 2 = path
path_color = 2  # Color value for the A* path

# Simulated movement steps
movement_steps = [
    (x, y) for y in range(GRID_SIZE)
    for x in (range(GRID_SIZE) if y % 2 == 0 else reversed(range(GRID_SIZE)))
]

def simulate_floor_scan():
    """
    Simulates scanning the grid and assigns random black or white tiles.
    
    """
    global grid
    for step_x, step_y in movement_steps:
        color = 1 if np.random.rand() > 0.2 else -1  # 80% chance of white, 20% chance of black
        grid[step_y][step_x] = color
        print(f"Simulated move to ({step_x}, {step_y}) at ({(step_x - GRID_SIZE // 2) * 100}mm, "
              f"{(step_y - GRID_SIZE // 2) * -100}mm)")


# Constants
GRID_SIZE = 16
CELL_SIZE = 40  # Size of each cell in pixels
WINDOW_SIZE = GRID_SIZE * CELL_SIZE
FPS = 20  # Frame rate

# Colors
UNMAPPED_COLOR = (150, 0, 100)  # Default unmapped color
WHITE_COLOR = (255, 255, 255)  # White panel color
BLACK_COLOR = (50, 50, 50)  # Black panel color
EXPLORED_COLOR = (255, 255, 0)  # Yellow for explored paths
PATH_COLOR = (0, 255, 0)  # Green for the final path
GRID_COLOR = (255, 0, 0)  # Red grid lines

# Directions for movement
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption("A* Pathfinding Visualization")
clock = pygame.time.Clock()

# Initialize grid
grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]


def draw_grid():
    """
    Draw the grid and cells.
    """
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            color = UNMAPPED_COLOR
            if grid[y][x] == 1:
                color = WHITE_COLOR
            elif grid[y][x] == -1:
                color = BLACK_COLOR
            elif grid[y][x] == 3:
                color = EXPLORED_COLOR
            elif grid[y][x] == 2:
                color = PATH_COLOR

            pygame.draw.rect(
                screen,
                color,
                (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE),
            )
            pygame.draw.rect(
                screen,
                GRID_COLOR,
                (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE),
                1,
            )


def a_star_search(maze, start, end):
    """
    Perform A* pathfinding algorithm with real-time visualization.
    """
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        current_priority, current = heapq.heappop(open_set)
        x, y = current

        # Mark the node as explored
        if grid[y][x] != 2:  # Skip final path marking
            grid[y][x] = 3  # Mark as explored (yellow)
        draw_grid()
        pygame.display.flip()
        clock.tick(FPS)

        if current == end:
            # Rebuild the path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()

            # Highlight the final path in green
            for px, py in path:
                grid[py][px] = 2
                draw_grid()
                pygame.display.flip()
                clock.tick(FPS)
            return path

        for dx, dy in DIRECTIONS:
            neighbor = (x + dx, y + dy)
            nx, ny = neighbor

            if (
                0 <= nx < GRID_SIZE
                and 0 <= ny < GRID_SIZE
                and maze[ny][nx] == 1  # Only white tiles are passable
            ):
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, end)
                    heapq.heappush(open_set, (f_score, neighbor))

    return None  # No path found


def prepare_maze(grid):
    """
    Prepare the maze for A* based on the scanned grid.
    1 = walkable, 0 = obstacle.
    """
    return [[1 if cell == 1 else 0 for cell in row] for row in grid]


def main():
    """
    Main simulation and pathfinding function.
    """
    simulate_floor_scan()

    running = True
    while running:
        screen.fill((0, 0, 0))  # Clear the screen

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        draw_grid()
        pygame.display.flip()
        clock.tick(FPS)

    # Prepare maze for A*
    maze = prepare_maze(grid)

    # Predefined start and end points
    start = (0, 0)
    end = (GRID_SIZE - 1, GRID_SIZE - 1)

    print(f"Finding path from {start} to {end}...")
    path = a_star_search(maze, start, end)
    if path:
        print(f"Path found: {path}")
    else:
        print("No path found!")

    pygame.image.save(screen, "final_path_with_explored.png")
    print("Final path saved as 'final_path_with_explored.png'.")

    # Keep the window open after pathfinding
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return


if __name__ == "__main__":
    main()