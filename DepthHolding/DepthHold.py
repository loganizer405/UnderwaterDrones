"""
Example of how to send MANUAL_CONTROL messages to the autopilot using
pymavlink.
This message is able to fully replace the joystick inputs.
"""

# Import mavutil
from pymavlink import mavutil
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import argparse


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


print("Watiing for connection")
master.wait_heartbeat()
print("HeatBeat received")

vehicle = connectSub()


def set_target_depth(depth):
    master.mav.set_position_target_global_int_sned()


def depthHold(depth, vehicle):
    current_depth = vehicle.location.global_relative_frame.alt

    if current_depth > depth*0.95:
        while current_depth > depth*0.95:
            current_depth = vehicle.location.global_relative_frame.alt
            print("Going Down!")
            manualControl(1000, 0, 1000)
            print("Current Depth: ", current_depth)

    if current_depth < depth * 0.95:
        while current_depth < depth * 0.95:
            current_depth = vehicle.location.global_relative_frame.alt
            manualControl(0, 0, -1000)
            print("Going uo !")
            print("Current Depth: ", current_depth)


depthHold(-10, vehicle)
