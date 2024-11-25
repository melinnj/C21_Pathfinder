import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander

# Initialize the low-level drivers
cflib.crtp.init_drivers()

# URI to the Crazyflie to connect to
uri = 'radio://0/12/2M/EE5C21CF11'

# Default flight parameters
FLYING_HEIGHT = 0.4  # Meters
DEFAULT_VELOCITY = 0.25  # m/s

# Define the grid corners (in meters)
corners = [
    (-0.5, -0.5),  # Bottom-left corner
    (0.5, -0.5),   # Bottom-right corner
    (0.5, 0.5),    # Top-right corner
    (-0.5, 0.5)    # Top-left corner
]

# Logging configuration
lg_stab = LogConfig(name='PositionLogger', period_in_ms=50)
lg_stab.add_variable('stateEstimate.x', 'float')
lg_stab.add_variable('stateEstimate.y', 'float')
lg_stab.add_variable('stateEstimate.z', 'float')

# Callback for logging position
def log_position_callback(timestamp, data, _):
    x = data['stateEstimate.x']
    y = data['stateEstimate.y']
    z = data['stateEstimate.z']
    print(f"[{timestamp}] X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}")

# Setup logging
def setup_logging(scf):
    scf.cf.log.add_config(lg_stab)
    lg_stab.data_received_cb.add_callback(log_position_callback)
    lg_stab.start()
    print("Logging started.")

# Navigate to grid corners
def navigate_to_corners(commander):
    print("Navigating to grid corners...")
    for x, y in corners:
        print(f"Moving to corner -> X: {x}, Y: {y}")
        commander.go_to(x, y, FLYING_HEIGHT, velocity=DEFAULT_VELOCITY)
        time.sleep(3)  # Allow time to move and stabilize
    print("Navigation completed.")

# Main execution
if __name__ == "__main__":
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        setup_logging(scf)

        # The PositionHlCommander will automatically take off when initialized
        with PositionHlCommander(
            scf,
            default_height=FLYING_HEIGHT,
            default_velocity=DEFAULT_VELOCITY,
            default_landing_height=0.0,
        ) as commander:
            try:
                navigate_to_corners(commander)

                # Land the drone after completing navigation
                print("Landing...")
            except Exception as e:
                print(f"Error during mission: {e}")
            finally:
                commander.land(velocity=DEFAULT_VELOCITY)
                time.sleep(3)  # Ensure safe landing
            print("Mission complete.")
