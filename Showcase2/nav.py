import argparse
from driver import *
from aruco_tracker import *
from aruco_landing import *
from scan_track import *
 
if __name__ == '__main__':
 
    tracker_id  = 4
    tracker_size  = 29.8 #- [cm]
 
    # path to the camera matrix and distortion.
    # Our Raspberry pi HQ camera doesn't have much distoration but these values are absolutely required to detect the marker
    # as without these will be passed later to the detector fuction. 
    mtx   = np.loadtxt('calib/cameraMatrix.txt', delimiter=',')
    dst   = np.loadtxt('calib/cameraDistortion.txt', delimiter=',')
    boundary = np.loadtxt('Section.waypoints',delimiter='\t',skiprows=2) 
    #print(waypoints)
    grid_size = 4
    height = 5
    parser = argparse.ArgumentParser(description = 'Control Copter and send commands in guided mode')
    parser.add_argument('--connect',
                    help = 'Vehicle connection target string. If not specified, SITL will automatically start and be used') 
    args = parser.parse_args()
    connection_string = args.connect

    # Connection to the vehicle
    vehicle = drone_connect(connection_string)

    hovering_height = 5  # In meters
    vehicle.groundspeed =1
    arm_and_takeoff(vehicle, hovering_height)
    waypoints = path_planner(boundary,grid_size,height)
    print(waypoints) 
    #field_scan(vehicle,waypoints)
    scanner = ScanTrack(vehicle=vehicle,tracker_id=tracker_id, tracker_size=tracker_size, mtx=mtx, dst=dst, waypoints = waypoints)
    
    detected = scanner.scan_track()

    if detected:
        lander = ArucoLanding(vehicle = vehicle, tracker_id=tracker_id, tracker_size=tracker_size, mtx=mtx, dst=dst)
        lander.track_and_land()

    if vehicle.mode.name == "LAND":
        print("Landed Successfully")

    complete_mision(vehicle)
       
