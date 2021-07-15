import sys
from pymavlink import mavutil
from pymavlink.mavextra import altitude
import time

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()
mode = 'GUIDED'

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

mode_id = master.mode_mapping()[mode]

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

while True:
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Check if command in the same in `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break

# arm ArduSub autopilot and wait until confirmed
master.arducopter_arm()
master.motors_armed_wait()


def move(Z):
    master.mav.set_position_target_local_ned_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask=0b0000111111111000,
        x=0, y=0, z=Z,
        vx=0, vy=0, vz=0,  # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
    )


print("Please enter your desired depth")
A = input()

Z = float(A)

move(Z)
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
            print('Depth not found')

    print("Current Depth: ", depth)

    positive_depth = float(depth)
    positive_depth = positive_depth*-1
    print("POSITIVE DEPTH: ", positive_depth, " WANTED DEPTH: ", Z)
    if positive_depth < Z:
        time.sleep(0.1)
    else:
        print("Finished")
        break


print("--------------Desired depth has been reached!-------------------")
print("--------------Changing to depth-hold mode... ")

# set the desired operating mode
DEPTH_HOLD = 'ALT_HOLD'
DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    master.set_mode(DEPTH_HOLD)
print("Depth Hold Mode Activated")
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
        print("Holding Depth At: ", depth)
