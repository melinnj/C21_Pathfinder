import sys
import time
import pickle
import numpy as np
from cflib.crazyflie import Crazyflie
import cflib.crtp

from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


def load_path(file_path='path_data.pkl'):
    """
    Loading path data from file
    """
    try:
        with open(file_path, 'rb') as f:
            path = pickle.load(f)
        print("Path data loaded successfully.")
        return path
    except FileNotFoundError:
        print(f"Path file '{file_path}' not found. Please run 'Map_Path' first.")
        sys.exit(1)


def calculate_relative_distance(current, target, scale=0.1):
    """
    Calculate relative distance scaled by a given factor.
    """
    dx = (target[0] * scale) - current[0]
    dy = (target[1] * scale) - current[1]
    return dx, dy


def fly_along_path(path, scale=0.1, default_height=0.5):
    """
    Let Crazyflie fly along the given path.
    """
    # Initialise communication
    cflib.crtp.init_drivers(enable_debug_driver=False)
    uri = 'radio://0/2/2M/EE5C21CF01'
    cf = Crazyflie(rw_cache='./cache')

    try:
        with SyncCrazyflie(uri, cf=cf) as scf:
            with MotionCommander(scf, default_height=default_height) as mc:
                # Initialize starting position (0,0)
                current_pos = (0.0, 0.0)

                # Calculate and move to the start of the path
                start_pos = path[0]
                dx, dy = calculate_relative_distance(current_pos, start_pos, scale)
                print(f"Moving to starting point: dx={dx}, dy={dy}")
                mc.move_distance(dx, dy, 0)

                # Update current position
                current_pos = (start_pos[0] * scale, start_pos[1] * scale)

                # Fly along the path
                for coord in path[1:]:
                    dx, dy = calculate_relative_distance(current_pos, coord, scale)
                    print(f"Flying to next point: dx={dx}, dy={dy}")
                    mc.move_distance(dx, dy, 0)

                    # Update current position
                    current_pos = (coord[0] * scale, coord[1] * scale)

                # Land after completing the path
                mc.land()
                print("Landing completed.")

    except Exception as e:
        print(f"Error during flight: {e}")
        raise


if __name__ == "__main__":
    path = load_path()
    fly_along_path(path)