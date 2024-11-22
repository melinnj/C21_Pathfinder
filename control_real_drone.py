# GNU General Public License

"""
This script interfaces with the drone. It is designed to be used with an absolute positioning system, such as the loco or lighthouse.
"""

import time
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
import logging
import atexit
import sys

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/2/2M/EE5C21CF01')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Python does not have constants.
LOG_CHECK_IN_MS = 50
LOG_CHECK = LOG_CHECK_IN_MS / 1000
WAIT_TIME = LOG_CHECK    # Time to wait before checking log.
FLYING_HEIGHT = 0.16 # In meters
DEFAULT_VELOCITY = 0.2 # In m/s

# We are using the FlowDeck V2, which is a motion sensor, to detect light. We can tell what light level it's outputting based on how fast the shutter is moving.
SHUTTER_SPEED_LIM = 3500     # Shutter speed is between 900 and 8000. If the shutter speed is above shutter_speed_lim, the color is black. Otherwise, white.
NICE_SHUTTER_SPEED_LIM = 4000 # A nicer version when we're not mapping the area. Stops accidentally marking crossable space as obstacles.

TOLERANCE_RADIUS = 0.02     # In meters. The desired drone coordinates must be within this radius.
ACCEL_TOL = 0.04            # In m/s. The drone will keep moving to the desired coordinates until it's velocity is below this threshold 

_shutter_speed_lim = SHUTTER_SPEED_LIM

def nice_shutter_speed():
    global _shutter_speed_lim
    _shutter_speed_lim = NICE_SHUTTER_SPEED_LIM

# Initialize the low-level drivers
cflib.crtp.init_drivers()

# Because we don't use the "with" statement for scf or pc, we need to manage scf and pc manually. This is not ideal.
# Add resources to the context manager for the duration of the script
context_resources = []
def add_to_context_manager(target):
    enter = type(target).__enter__
    exit = type(target).__exit__
    value = enter(target)
    hit_except = False
    item = None
    try:
        item = value
    except:
        hit_except = True
        if not exit(target, *sys.exc_info()):
            raise
    if not hit_except:
        context_resources.append(target)
        def leave_target():
            exit(target, None, None, None)
        # The target will only leave context manager if interrupted.
        atexit.register( leave_target )
    return item
# Close the resources in context manager when called
def leave_all_targets():
    # Exit all context managers manually
    while context_resources:
        target = context_resources.pop()
        exit = type(target).__exit__
        exit(target, *sys.exc_info())

# Connect to the Crazyflie
_scf = add_to_context_manager( SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) )
# Setup the position commander. Will fly up once initialized. Will land once the script exits.
_pc = add_to_context_manager( PositionHlCommander(_scf, controller=PositionHlCommander.CONTROLLER_PID) )
_pc.set_default_height(FLYING_HEIGHT) # Default height in meters
_pc.set_default_velocity(DEFAULT_VELOCITY) # Default velocity in m/s

# Make sure to manually exit, otherwise the script will keep running.
def manual_exit():
    if 'pc' in globals():
        _pc.__exit__(*sys.exc_info() )
    if 'scf' in globals():
        _scf.__exit__(*sys.exc_info() )
    leave_all_targets()
atexit.register( manual_exit )

##### Create log configuration
_are_we_logging = False
def is_logging():
    return _are_we_logging
_lg_stab = LogConfig(name='Light_Level_At_Location', period_in_ms=LOG_CHECK_IN_MS)
_lg_stab.add_variable('motion.shutter', 'uint16_t')
_lg_stab.add_variable('pm.batteryLevel', 'uint8_t')
_lg_stab.add_variable('stateEstimateZ.x', 'int16_t')
_lg_stab.add_variable('stateEstimateZ.y', 'int16_t')
_lg_stab.add_variable('stateEstimateZ.z', 'int16_t')
_lg_stab.add_variable('stateEstimateZ.vx', 'int16_t')
_lg_stab.add_variable('stateEstimateZ.vy', 'int16_t')

# Start logging
def start_log():
    _scf.cf.log.reset()
    _scf.cf.log.add_config(_logconf)
    _logconf.data_received_cb.add_callback(_logconf_callback)
    _logconf.start()

# Grab data and put into globals
def log_stab_callback(timestamp, data, _):
    global _x, _y, _z, _vx, _vy, _shut, _are_we_logging
    # Coords
    _x = np.int16(data['stateEstimateZ.x']) 
    _y = np.int16(data['stateEstimateZ.y'])
    _z = np.int16(data['stateEstimateZ.z'])
    # Velocity
    _vx = np.int16(data['stateEstimateZ.vx']) 
    _vy = np.int16(data['stateEstimateZ.vy'])
    # Shutter speed (Detects light)
    _shut = np.uint16(data['motion.shutter'])
    battery = np.uint8(data['pm.batteryLevel'])

    # At any point, if the battery is low, the drone will stop.
    if battery < 10:
        print("WARNING: Battery Level at ", battery, "%. Shutting Down...")
        manual_exit()
        raise Exception("LOW BATTERY")
    
    _are_we_logging = True # So the other script knows

_logconf = _lg_stab
_logconf_callback = log_stab_callback
start_log() # Start logging
######

## Grab globals and return them in the correct format
def grab_x_y():
    return np.float16(_x/1000), np.float16(_y/1000)
    
def grab_x_y_vx_vy_shut():
    return np.float16(_x/1000), np.float16(_y/1000), np.float16(_vx/1000), np.float16(_vy/1000), np.uint16(_shut)
# Move to the coordinate
def mov(new_x, new_y):
    _pc.go_to(new_x, new_y)
    time.sleep(WAIT_TIME)
    x, y, vx, vy, shut = grab_x_y_vx_vy_shut()
    while abs(x - new_x) > TOLERANCE_RADIUS or abs(y - new_y) > TOLERANCE_RADIUS or abs(vx) > ACCEL_TOL or abs(vy) > ACCEL_TOL:
        _pc.go_to(new_x, new_y)
        time.sleep(WAIT_TIME)
        x, y, vx, vy, shut = grab_x_y_vx_vy_shut()
    # If shut == 900, This normally means the battery is becoming low. Don't shut down though, just in case.
    if shut == 900:
        print("WARNING!!! Shutter speed stuck at 900! Please Restart Drone!")
    if shut > _shutter_speed_lim:
        color_black = True
        print("Light Lvl: ", shut, "\tBLACK")
    else:
        color_black = False
        print("Light Lvl: ", shut, "\tWHITE")
    return x, y, color_black

if __name__ == '__main__':
    print("You are NOT supposed to run this!")

def black():
     x, y, vx, vy, shut = grab_x_y_vx_vy_shut()
     print("x-axis",x,"y-axis",y)
    # If shut == 900, This normally means the battery is becoming low. Don't shut down though, just in case.
     if shut == 900:
        print("WARNING!!! Shutter speed stuck at 900! Please Restart Drone!")
     if shut > _shutter_speed_lim:
        color_black = True
        print("Light Lvl: ", shut, "\tBLACK")
     else:
        color_black = False
        print("Light Lvl: ", shut, "\tWHITE")
     return x, y, color_black