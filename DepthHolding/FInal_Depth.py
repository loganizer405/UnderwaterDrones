import sys
from pymavlink import mavutil
from pymavlink.mavextra import altitude
import time

# Create the connection
master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')
boot_time = time.time()


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
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask=0b0000111111111000, 
        x=0 , y=0 , z=Z, 
        vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
    )
print("Please enter your desired depth")
A = input()

Z = float(A)

move(Z)
l =0
while True :
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
        print("Current Depth: ",depth)
        l=float(depth)
        l=l*-1
    print("THE Z", Z, "WHERE WE WANT", l)
    if l > Z * 0.95:
        print("Reached")
        break

