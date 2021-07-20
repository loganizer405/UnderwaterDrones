from pymavlink import mavutil
import time
import argparse
import math


# Create the connection

# Wait a heartbeat before sending commands
# master = mavutil.mavlink_connection(udpin:0.0.0.0:14550)
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


def get_distance(array):
    get_velocity(array)
    time = len(array) * 0.1
    distance = 0
    for i in range(array):
        distance += array[i] * 0.1

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


def set_target_depth(desired_depth):
    current_depth = getDepth()
    print(current_depth)
    if current_depth > desired_depth:
        while True:
            manualControl(0, 0, -5)
            current_depth = getDepth()
            print(current_depth)
            if (current_depth < 0.95 * desired_depth):
                print("REACHED: DEPTH WANTED", desired_depth,
                      " CURRENT DEPTH:", getDepth())
                break
    else:
        while True:
            manualControl(0, 0, 5)
            current_depth = getDepth()
            print(current_depth)
            if (current_depth > 0.95 * desired_depth):
                print("REACHED: DEPTH WANTED", desired_depth,
                      " CURRENT DEPTH:", getDepth())
                break


wait_conn()
print("<<<<<<CONNECTION ESTABLISHED>>>>>>")
boot_time = time.time()
master.wait_heartbeat()
print("<<<<<<<HEARTBEAT RECEIVED>>>>>>")


master.arducopter_arm()
time.sleep(1)
print("<<<<<<ARMED>>>>>>")
# Setting the mode to manual
mode = 'MANUAL'
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)


print("<<<<<<MODE CHANGED TO ", mode, ">>>>>>")
time.sleep(0.2)

recorded_distance = get_distance()
while 6 > recorded_distance:
    manualControl(5, 0, 0)
    time.sleep(0.1)
    recorded_distance = get_distance()

print("REACHED DESIRED DISTANCE: ", get_distance())
