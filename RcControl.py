"""
Example of how to send MANUAL_CONTROL messages to the autopilot using
pymavlink.
This message is able to fully replace the joystick inputs.
"""

# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
master = mavutil.mavlink_connection('udpout:0.0.0.0:9000')
wait_conn()
# Wait a heartbeat before sending commands


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


master.wait_heartbeat()

for i in range(20):
    master.mav.manual_control_send(
        master.target_system,
        1000,  # x
        0,  # y
        500,  # z
        0,  # r
        0)  # buttons
    time.sleep()
