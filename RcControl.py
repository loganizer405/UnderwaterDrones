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
# Wait a heartbeat before sending commands
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
