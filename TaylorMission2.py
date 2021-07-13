from pymavlink import mavutil
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import argparse
import math
import psutil


def connectSub():
    vehicle = connect("udpout:0.0.0.0:9000", wait_ready=True)
    return vehicle


# Create the connection

# Wait a heartbeat before sending commands


def wait_conn():
    """
    Sends a ping to stabilish the UDP communication and awaits for a response
    """
    msg = None
    while not msg:
        master.mav.ping_send(
            int(time.time() * 1e6),  # Unix time in microseconds
            0,  # Ping number
            0,  # Request ping of all systems
            0  # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)


def set_target_depth(depth):

    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=0xdfe,  # ignore everything except z position
        # (x, y WGS84 frame pos - not used), z [m]
        lat_int=0, long_int=0, alt=depth,
        vx=0, vy=0, vz=0,  # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )


def manualControl(x, y, z):
    master.mav.manual_control_send(
        master.target_system,
        x,  # x
        y,  # y
        z,  # z
        0,  # r
        0)  # buttons


master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')
wait_conn()
vehicle = connectSub()
boot_time = time.time()
print("<<<<<<CONNECTION ESTABLISHED>>>>>>")
master.wait_heartbeat()
print("<<<<<<<HEARTBEAT RECEIVED>>>>>>")


DEPTH_HOLD = 'ALT_HOLD'
DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    master.set_mode(DEPTH_HOLD)

current_depth = vehicle.location.global_relative_frame.alt

print("Depth before: ", current_depth)
# set a depth target
set_target_depth(-2)

time.sleep(0.3)
current_depth = vehicle.location.global_relative_frame.alt
print("Depth after: ", current_depth)


master.arducopter_disarm()
master.motors_disarmed_wait()
