import time
from pymavlink import mavutil

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
def rotation(d):
    t=int(d*0.81)
    for i in range(0,t):
        master.mav.manual_control_send(
         master.target_system,
             0,
             0,
             5,
             255,
             0)

rotation(90)


