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
    print(current_depth)
    while current_depth > depth:
        manualControl(0, 0, 1000)
        current_depth = vehicle.location.global_relative_frame.alt
        print(current_depth)


wait_conn()
print("<<<<<<CONNECTION ESTABLISHED>>>>>>")
master.wait_heartbeat()
print("<<<<<<<HEARTBEAT RECEIVED>>>>>>")


# ARMING:
vehicle = connectSub()
vehicle.armed = False
time.sleep(1)
master.arducopter_arm()
time.sleep(1)
print("<<<<<<ARMED>>>>>>")

print(list(master.mode_mapping()))

mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)


print("<<<<<<MODE CHANGED TO GUIDED>>>>>>")
time.sleep(5)


set_target_depth(-10)
print("TEST FINISHED")

mode = 'POSHOLD'
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

vehicle.armed = False
