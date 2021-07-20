
import time

# Import mavutil
from pymavlink import mavutil

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


def manualControl(x, y, z):
    master.mav.manual_control_send(
        master.target_system,
        x,  # x
        y,  # y
        z,  # z
        0,  # r
        0)  # buttons
while True :
    manualControl(1000,0,0)
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
        print("DAta: ",speed)

