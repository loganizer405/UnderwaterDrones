from pymavlink import mavutil
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import argparse
import math


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


def manualControl(x, y, z):
    master.mav.manual_control_send(
        master.target_system,
        x,  # x
        y,  # y
        z,  # z
        0,  # r
        0)  # buttons


def getDepth():
    depth = 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            data = str(msg)
            try:
                data = data.split(":")
                depth = data[5].split(",")[0]
            except:
                print('')
            print("Current Depth: ", depth)

        if not depth == 0:
            break
    return depth


def set_target_depth(depth, vehicle):
    current_depth = getDepth()
    print(current_depth)
    if current_depth > depth:
        while current_depth > depth:
            manualControl(0, 0, -5)
            current_depth = getDepth()
            print(current_depth)
            if (current_depth > 0.99 * depth):
                print("DEPTH WANTED", depth, " CURRENT DEPTH:", getDepth())
                break
    else:
        while current_depth < depth:
            manualControl(0, 0, 5)
            current_depth = getDepth()
            print(current_depth)
            if (current_depth < 0.95 * depth):
                break


wait_conn()
print("<<<<<<CONNECTION ESTABLISHED>>>>>>")
boot_time = time.time()
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

mode = 'MANUAL'
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)


print("<<<<<<MODE CHANGED TO ", mode, ">>>>>>")
time.sleep(5)


set_target_depth(-1, vehicle)
print("TEST FINISHED")

mode = 'ALT_HOLD'
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
