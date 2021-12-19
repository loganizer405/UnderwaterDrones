import time
from pymavlink import mavutil, mavwp


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


master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')
wait_conn()
lat = (45.93036680, 45.93142227, 45.93233196)
lon = (-119.29152888, -119.29056006, -119.29314315)
alt = (0, 0, 0)
N = len(lat)

msg = None

# wait for autoplit connection
while msg is None:
    msg = master.recv_msg()
print(msg)

master.mav.heartbeat_send(
    6,  # type
    8,  # autopilot
    192,  # base_mode
    0,  # custom_mode
    4,  # system_status
    3  # mavlink_version
)

master.wait_heartbeat()  # DO NOT REMOVE

time.sleep(3)

# ARM
master.mav.command_long_send(
    1,  # autopilot system ID
    1,  # autopilot component ID
    400,  # command ID, ARM/DISARM
    0,  # confirmation
    1,  # ARM
    0, 0, 0, 0, 0, 0  # unused parameters
)

# set mode
mavutil.mavfile.set_mode(master,
                         3,  # auto mode selected
                         0, 0)

wp = mavwp.MAVWPLoader()
seq = 1
frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
radius = 10

for i in range(1):
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                                                        master.target_component,
                                                        seq,
                                                        frame,
                                                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                        0, 0, 0, radius, 0, 0,
                                                        1, 1, 1))

    seq += 1

master.waypoint_clear_all_send()
master.waypoint_count_send(wp.count())

for i in range(wp.count()):
    msg = master.recv_match(type=['MISSION_REQUEST'], blocking=True)
    master.mav.send(wp.wp(msg.seq))
    print('Sending waypoint {0}'.format(msg.seq))

time.sleep(10)

# DISARM
master.mav.command_long_send(
    1,  # autopilot system ID
    1,  # autopilot component ID
    400,  # command ID, ARM/DISARM
    0,  # confirmation
    1,  # ARM
    0, 0, 0, 0, 0, 0  # unused parameters
)
