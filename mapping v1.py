import pygame
import logging
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
pygame.display.set_caption("Drone Mapping Simulation")
clock = pygame.time.Clock()

# Initialize grid
grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

# Set drone's initial position to the center of the grid
origin_x, origin_y = GRID_SIZE // 2, GRID_SIZE // 2

# Global variables for logging values
globX, globY, globZ, globShut = 0, 0, 0, 0

def log_stab_callback(timestamp, data, logconf):
    """
    Callback to update global values for position and shutter data.
    """
    global globX, globY, globZ, globShut
    globX = data['stateEstimateZ.x']
    globY = data['stateEstimateZ.y']
    globZ = data['stateEstimateZ.z']
    globShut = data['motion.shutter']
    print(f"Callback - Shutter Value: {globShut}")  # Debugging output

def black_or_white():
    """
    Determines if the detected color is black or white based on the shutter value.
    """
    _, _, _, shut = grab_log_values()
    if shut > 4000:
        print("Detected Black!")  # Debugging output
        return "black"
    else:
        print("Detected White!")  # Debugging output
        return "white"

def grab_log_values():
    """
    Retrieves the current values of position and shutter.
    """
    global globX, globY, globZ, globShut
    return globX, globY, globZ, globShut

def draw_grid():
    """
    Draws the grid on the pygame screen.
    0 = black (unmapped), 1 = white (mapped white), -1 = black (mapped black)
    """
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            color = (0, 0, 0)  # Default color (black for unmapped)
            if grid[y][x] == 1:
                color = (255, 255, 255)  # White for mapped white
            elif grid[y][x] == -1:
                color = (50, 50, 50)  # Dark gray for mapped black

            pygame.draw.rect(
                screen,
                color,
                (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE),
            )
            pygame.draw.rect(
                screen,
                (255, 0, 0),  # Red grid lines
                (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE),
                1,
            )

def update_origin_square(color):
    """
    Updates the origin square based on the detected color.
    """
    grid[origin_y][origin_x] = 1 if color == "white" else -1
    print(f"Updated Origin Square at ({origin_x}, {origin_y}) to {color}")

def main():
    """
    Main loop for the drone mapping simulation.
    """
    global grid

    cflib.crtp.init_drivers()

    print("Connecting to the Crazyflie drone...")
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connection established.")

        # Set up a log configuration to retrieve sensor data
        log_config = LogConfig(name="Light_Level_At_Location", period_in_ms=200)
        log_config.add_variable('motion.shutter', 'uint16_t')
        log_config.add_variable('stateEstimateZ.x', 'float')
        log_config.add_variable('stateEstimateZ.y', 'float')
        log_config.add_variable('stateEstimateZ.z', 'float')

        scf.cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(log_stab_callback)
        log_config.start()

        running = True
        while running:
            screen.fill((0, 0, 0))  # Clear the screen

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Detect color at the origin square
            color = black_or_white()
            update_origin_square(color)

            # Draw the grid
            draw_grid()

            # Update the display
            pygame.display.flip()

            # Cap the frame rate
            clock.tick(FPS)

        log_config.stop()
        print("Mapping complete. Saving grid as 'mapped_area.png'...")
        save_grid_as_image(grid)

def save_grid_as_image(grid):
    """
    Saves the final grid as an image.
    """
    pygame_surface = pygame.Surface((WINDOW_SIZE, WINDOW_SIZE))
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            color = (0, 0, 0)  # Default color (black)
            if grid[y][x] == 1:
                color = (255, 255, 255)  # White for mapped white
            elif grid[y][x] == -1:
                color = (50, 50, 50)  # Dark gray for mapped black

            pygame.draw.rect(
                pygame_surface,
                color,
                (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE),
            )

    pygame.image.save(pygame_surface, "mapped_area.png")
    print("Mapped area saved as 'mapped_area.png'.")

if __name__ == "__main__":
    main()
