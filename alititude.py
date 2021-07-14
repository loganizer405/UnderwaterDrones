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

master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')


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


wait_conn()
vehicle = connectSub()
master.wait_heartbeat()

while True:
    current_depth = vehicle.location.global_relative_frame.alt
    print("CURRENT DEPTH: ", current_depth)

    time.sleep(0.3)
