from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil # Needed for command message definitions
import time
import time
import math
import argparse

def drone_connect(connection_str):
    # Connect to the vehicle
    print('Connecting to vehicle on: %s' % connection_str)
    vehicle = connect(connection_str, wait_ready = True)
    return vehicle

def arm_and_takeoff(vehicle, aTargetAltitude):
    print("Basic pre-arm checks")
    # Ensures user does not try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)
    
    print("Arming motors")
    # Copter should arm in guided mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for vehicle to arm...")
        time.sleep(1)

    print("Vehicle is taking off!")
    # Take off to specified altitude
    vehicle.simple_takeoff(aTargetAltitude)

    """
    Wait until vehicle has reached specified altitude before processing next command
    Any command directly after Vehicle.simple_takeoff will execute immediately 
    """
    while True:
        print("Altitude: %f "% (vehicle.location.global_relative_frame.alt))
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached  target altitude")
            break
        time.sleep(1)


def send_ned_velocity( vehicle, velocityX, velocityY, velocityZ):
	"""
	Move vehicle in direction specified by velocity vectors
	Helper method 
	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,      # time_boot_ms (not used)
		0, 0,   # target system, target component
		mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
		0b0000111111000111, # type_mask (ensures speed and not position is enabled)
		0, 0, 0,    # position x, y, z (not used)
		velocityX, velocityY, velocityZ, # velocity for x, y, z in m/s
		0, 0, 0,    # accelleration (not used/not supported)
		0, 0)       # yaw (also not supported)
	# send command to vehicle
	vehicle.send_mavlink(msg)
	vehicle.flush()

def moveUp(vehicle, t):
	"""
	moves vehicle up a given length of time (in seconds)
	"""
	print("Moving up")
	send_ned_velocity(vehicle, 0,0,-1)
	print(vehicle.velocity)
	print("Alt %f "% vehicle.location.global_relative_frame.alt)
	time.sleep(t)
	send_ned_velocity(vehicle, 0,0,0)
	print(vehicle.velocity)
	print("Alt %f "% vehicle.location.global_relative_frame.alt)
		
def moveDown(vehicle, t):
	"""
	moves vehicle down a given length of time (in seconds)
	"""
	print("Moving Down")
	send_ned_velocity(vehicle, 0,0,1)
	print(vehicle.velocity)
	print("Alt %f "% vehicle.location.global_relative_frame.alt)
	time.sleep(t)
	send_ned_velocity(vehicle, 0,0,0)
	print(vehicle.velocity)
	print("Alt %f "% vehicle.location.global_relative_frame.alt)

def moveLeft(vehicle, t):
	"""
	moves vehicle left a given length of time (in seconds)
	"""
	print("Moving Left")
	send_ned_velocity(vehicle, 0,-1,0)
	print ("Velocity:")
	print((vehicle.velocity))
	time.sleep(t)
	send_ned_velocity(vehicle,0,0,0)
	print ("Velocity:")
	print((vehicle.velocity))

def moveRight(vehicle, t):
	"""
	moves vehicle right a given length of time (in seconds)
	"""
	print("Moving Right")
	send_ned_velocity(vehicle,0,1,0)
	print ("Velocity:")
	print((vehicle.velocity))
	time.sleep(t)
	send_ned_velocity(vehicle,0,0,0)
	print ("Velocity:")
	print((vehicle.velocity))
	
def moveBack(vehicle, t):
	"""
	moves vehicle back a given length of time (in seconds)
	"""
	print("Moving Back")
	send_ned_velocity(vehicle,-1,0,0)
	print ("Velocity:")
	print((vehicle.velocity))
	time.sleep(t)
	send_ned_velocity(vehicle,0,0,0)
	print ("Velocity:")
	print((vehicle.velocity))

def moveForward(vehicle, t):
	"""
	moves vehicle forward a given length of time (in seconds)
	"""
	print("Moving Forward")
	send_ned_velocity(vehicle,1,0,0)
	print ("Velocity:")
	print((vehicle.velocity))
	time.sleep(t)
	send_ned_velocity(vehicle,0,0,0)
	print ("Velocity:")
	print((vehicle.velocity))


def land(vehicle ):
	vehicle.mode = VehicleMode("LAND")
	while vehicle.location.global_relative_frame.alt > 0.95:
		print("Now Landing\tAltitude: %f" % (vehicle.location.global_relative_frame.alt))
		time.sleep(1)

def complete_mision(vehicle):
    print("Close vehicle object")
    vehicle.close()
    print("Mission Complete")
	
	
	
	
def test_motion(vehicle):
    land = False
    while not land:
        print("Type: L for Left, R for Right, Up for Up, D for Down, F for Forward, B for Back and S for STOP")
        user_in = input("Command:")
        if user_in =="S":
            land = True
        elif user_in == "U":
            moveUp(vehicle, 0.5)
        elif user_in == "D":
            moveDown(vehicle, 0.5)
        elif user_in == "L":
            moveLeft(vehicle, 0.5)
        elif user_in == "R":
            moveRight(vehicle, 0.5)
        elif user_in == "F":
            moveForward(vehicle, 0.5)
        elif user_in == "B":
            moveBack(vehicle, 0.5)
        

def get_distance_metres(aLocation1, aLocation2):
    dLat = aLocation2.lat - aLocation1.lat
    dLon = aLocation2.lon - aLocation1.lon
    return math.sqrt((dLat * dLat) + (dLon * dLon)) * 1.113195e5

"""
Returns the bearing between the two LocationGlobal objects passed as parameters.
This method is an approximation, and may not be accurate over long distances and
close earths poles.
"""

def field_scan(vehicle, waypoints):
	detected = False

	waypoint_list = waypoints[:, [8,9,10]]
	print(waypoint_list)
	i = 0
	for waypoint in waypoint_list:
		targetLocation  = LocationGlobalRelative(waypoint[0],waypoint[1],waypoint[2])
		vehicle.simple_goto(targetLocation)
		while vehicle.mode.name == "GUIDED":
			remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
			print("Distance to target: ", remainingDistance)
			if remainingDistance <= 1:
				print("Target %d reached" % i)
				i=  i + 1
				break

	return detected
	

def path_planner(boundary_p,grid_unit,height):
    conversion_m_lat  = (0.00001/ 1.11)
    conversion_lat_m  = (1.11/0.00001 )
    
    boundary = boundary_p[:, [8,9]]
    top = max(boundary[0,0],boundary[1,0])
    bottom = min(boundary[0,0],boundary[1,0])
    left = min(boundary[0,1],boundary[1,1])
    right = max(boundary[0,1],boundary[1,1])
    lat_diff = abs(top-bottom)
    lon_diff = abs(right- left)
    lat_distance = lat_diff* conversion_lat_m
    lon_distance = lon_diff * conversion_lat_m
    lat_units = int(lat_distance/grid_unit)
    lon_units = int(lon_distance/grid_unit)
    waypoints = []
    start_lat = bottom
    start_lon = left
    waypoints.append([start_lat, start_lon, height])
    print(boundary)
    print(lat_units)
    print(lon_units)

    for  x in range (1,lat_units):
        lat = start_lat + x*conversion_m_lat*grid_unit
        if x%2 ==0:
            reverse = True
            
        else:
            reverse = False

        for y in range (1,lon_units):
            if not reverse:
                waypoints.append([lat ,start_lon + y*conversion_m_lat*grid_unit, height])
            else:
                 waypoints.append([lat ,start_lon + ( lon_units -y)*conversion_m_lat*grid_unit, height])
    
    return waypoints


