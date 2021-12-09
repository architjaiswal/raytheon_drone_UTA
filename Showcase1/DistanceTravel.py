#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

import argparse

parser = argparse.ArgumentParser(description = 'Control Copter and send commands in guided mode')
parser.add_argument('--connect',
                    help = 'Vehicle connection target string. If not specified, SITL will automatically start and be used') 
args = parser.parse_args()

connection_string = args.connect
sitl = None

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready = True)

"""
Arms vehicle and takes off to specified altitude
"""
def arm_and_takeoff(aTargetAltitude):
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
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached  target altitude")
            break
        time.sleep(1)
    
# Arm and take off to altitude of 5 meters
arm_and_takeoff(5)

"""
Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
specified `original_location`. The returned LocationGlobal has the same `alt` value
as `original_location`.

The function is useful when you want to move the vehicle around specifying locations relative to 
the current vehicle position.

The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
"""

def get_location_metres(original_location, dNorth, dEast):
    # 'Spherical' radius of earth
    earth_radius = 6378137.0
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius * math.cos(math.pi * original_location.lat/180))

    # New position in decimal degrees
    newLat = original_location.lat + (dLat * 180/math.pi)
    newLon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation = LocationGlobal(newLat, newLon, original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(newLat, newLon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation

"""
Returns the ground distance in metres between two LocationGlobal objects.

This method is an approximation, and will not be accurate over large distances and close to the 
earth's poles.
"""
def get_distance_metres(aLocation1, aLocation2):
    dLat = aLocation2.lat - aLocation1.lat
    dLon = aLocation2.lon - aLocation1.lon
    return math.sqrt((dLat * dLat) + (dLon * dLon)) * 1.113195e5

"""
Returns the bearing between the two LocationGlobal objects passed as parameters.
This method is an approximation, and may not be accurate over long distances and
close earths poles.
"""
def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing

"""
Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.
"""
def goto_position_target_global_int(aLocation):

    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        int(aLocation.lat*1e7), # lat_int - X Position in WGS84 frame in 1e7 * meters
        int(aLocation.lon*1e7), # lon_int - Y Position in WGS84 frame in 1e7 * meters
        int(aLocation.alt), # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, 0, 0, # X,Y,Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    print(msg)
    vehicle.send_mavlink(msg)

"""
Moves the vehicle to a postiion dNorth meters North and dEast meters East
of the current position.
The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
the target position. This allows it to be called with different position-setting commands.
This method reports the distance to target every two seconds
"""
def goto(dNorth, dEast, gotoFunction = vehicle.simple_goto):
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    #Stop action if we are no longer in guided mode
    while vehicle.mode.name == "GUIDED":
        remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance <= targetDistance*0.01:
            print("Target reached")
            break
        time.sleep(2)

print("Fly straight line path to 30 yard line")
print("Setting groundspeed to 5 m/s")
vehicle.groundspeed = 5
goto(27.41,0,goto_position_target_global_int)

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")

print("Close vehicle object")
vehicle.close()

if sitl is not None:
    sitl.stop()

print("Mission Complete")

