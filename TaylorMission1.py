from pymavlink import mavutil
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import argparse
import math
import psutil


def connectSub():
    vehicle = connect("127.0.0.1:14550", wait_ready=True)
    return vehicle


# Create the connection

# Wait a heartbeat before sending commands

master = mavutil.mavlink_connection('127.0.0.1:14550')


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


def manualControl(x, y, z):
    master.mav.manual_control_send(
        master.target_system,
        x,  # x
        y,  # y
        z,  # z
        0,  # r
        0)  # buttons


def set_target_depth(depth):
    current_depth = vehicle.location.global_relative_frame.alt
    while current_depth > depth * 0.95:
        master.mav.set_position_target_global_int_send(int(1e3 * (time.time() - psutil.boot_time())), master.target_system,
                                                       master.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_INT, 0xdfe, 0, 0, depth, 0, 0, 0, 0, 0, 0, 0, 0)
        current_depth = current_depth = vehicle.location.global_relative_frame.alt
        print("CURRENT_DEPTH: ", current_depth)


wait_conn()
print("<<<<<<CONNECTION ESTABLISHED>>>>>>")
master.wait_heartbeat()
print("<<<<<<<HEARTBEAT RECEIVED>>>>>>")


# ARMING:
vehicle = connectSub()
vehicle.armed = False
time.sleep(1)
master.arducopter_arm()

print("<<<<<<ARMED>>>>>>")
DEPTH_HOLD = 'ALT_HOLD'
DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    master.set_mode(DEPTH_HOLD)

set_target_depth(-10)
print("TEST FINISHED")
vehicle.armed = False
