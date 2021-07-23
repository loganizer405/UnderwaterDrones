from pymavlink import mavutil
import time
import argparse
import math


# Create the connection

# Wait a heartbeat before sending commands
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
#master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')


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


def get_distance(array):
    get_velocity(array)
    distance = 0
    for i in range(len(array)):
        distance += array[i] * 0.001

    return distance


def get_velocity(array):
    velocity = 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            data = str(msg)
            try:
                data = data.split(":")
                speed = data[2].split(",")[0]
            except:
                print('')

            velocity = float(speed)
        if not velocity == 0:
            break

    array.append(velocity)
    return velocity


def travel_in_x(xThrottle, to):
    mode = 'MANUAL'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    print("<<<<<<MODE CHANGED TO ", mode, ">>>>>>")
    velocity_array = []
    for i in range(10000):
        manualControl(xThrottle, 0, 500)
        print("RECORDED DISTANCE: ", get_distance(velocity_array))
        if to < get_distance(velocity_array):
            print("VELOCITY ARRAY:", velocity_array)
            break

    print("REACHED DESIRED DISTANCE: ", get_distance(velocity_array))

    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)


wait_conn()
print("<<<<<<CONNECTION ESTABLISHED>>>>>>")
boot_time = time.time()
master.wait_heartbeat()
print("<<<<<<<HEARTBEAT RECEIVED>>>>>>")


master.arducopter_arm()
time.sleep(1)
print("<<<<<<ARMED>>>>>>")

travel_in_x(10000, 10)
