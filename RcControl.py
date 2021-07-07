"""
Example of how to send MANUAL_CONTROL messages to the autopilot using
pymavlink.
This message is able to fully replace the joystick inputs.
"""

# Import mavutil
from pymavlink import mavutil
import time
import argparse


parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--xThrottle')
parser.add_argument('--yThrottle')
parser.add_argument('--zThrottle')
args = parser.parse_args()

x_Throttle = args.xThrottle
y_Throttle = args.yThrottle
z_Throttle = args.zThrottle


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


def manualControl(x, y, z):
    master.mav.manual_control_send(
        master.target_system,
        x,  # x
        y,  # y
        z,  # z
        0,  # r
        0)  # buttons


master.wait_heartbeat()

for i in range(10000):
    manualControl(x_Throttle, y_Throttle, z_Throttle)
