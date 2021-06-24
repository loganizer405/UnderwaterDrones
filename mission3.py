import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import exceptions
import argparse
from pymavlink import mavutil



def connectSub():
	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')

	args = parser.parse_args()

	connection_string = args.connect
	vehicle = connect(connection_string, wait_ready=True)
	return vehicle 

def wait_conn():


	msg = None  
	while not msg:
		vehicle.send_mavlink(vehicle.message_factory.system_time_encode(int(time.time() * 1e6), 0))
		msg = vehicle.parameters[b'SURFACE_DEPTH']
		time.sleep(0.5)


def arm_protocol(desiredAltitude):
	#Checking whether we can arm the vehicle
	

	while not vehicle.is_armable:
		print("Waiting to arm")
		time.sleep(1)

	# Always arm motors in Guided mode, then arm the vehicle
	print("Arming motors")
	vehicle.mode = VehicleMode("GUIDED")

	while not vehicle.mode.name == "GUIDED":
		print "CHANGING MODES ..."
		time.sleep(1)

	vehicle.armed = True

	vehicle.mode = VehicleMode("MANUAL")
	while not vehicle.mode.name == "MANUAL":
		print "CHANGING MODES"
		time.sleep(1)

	print "current mode %s" % vehicle.mode.name
	#If the vehicle hasn't armed for some reason

	while not vehicle.armed:
		print("Vehicle has had a problem with arming")
		time.sleep(1)


	while True:
		send_local_ned_velocity(500,-500,250)
		altitude = vehicle.location.global_relative_frame.alt
		print("Current altitude: ", altitude)

		if altitude <= 0.95* desiredAltitude:
			break
		else:
			time.sleep(1)


def dissarm_protocol():
	print "dissarm protocol"
	while True:

		send_local_ned_velocity(-1000,1000,500)
		altitude = vehicle.location.global_relative_frame.alt
		print("Current altitude: ", altitude)

		if altitude > -0.4:
			break
		else:
			time.sleep(1)

	
	vehicle.armed = False
	
	vehicle.mode = VehicleMode("AUTO")	



def send_local_ned_velocity(x, y, z):
	msg = vehicle.message_factory.manual_control_encode(0, x, y, z, 500, 0)
	vehicle.send_mavlink(msg)


vehicle = connectSub()
wait_conn()
arm_protocol(-10)
dissarm_protocol()



print("Close vehicle object")
vehicle.close()



