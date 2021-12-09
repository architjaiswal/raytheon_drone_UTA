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
    waypoints = np.loadtxt('test.waypoints',delimiter='\t',skiprows=2) 
    #print(waypoints)
    
    parser = argparse.ArgumentParser(description = 'Control Copter and send commands in guided mode')
    parser.add_argument('--connect',
                    help = 'Vehicle connection target string. If not specified, SITL will automatically start and be used') 
    args = parser.parse_args()
    connection_string = args.connect

    # Connection to the vehicle
    vehicle = drone_connect(connection_string)

    hovering_height = 3  # In meters
    arm_and_takeoff(vehicle, hovering_height)
    #field_scan(vehicle,waypoints)
    scanner = ScanTrack(vehicle=vehicle,tracker_id=tracker_id, tracker_size=tracker_size, mtx=mtx, dst=dst, waypoints = waypoints)
    
    detected = scanner.scan_track()

    if detected:
        lander = ArucoLanding(vehicle = vehicle, tracker_id=tracker_id, tracker_size=tracker_size, mtx=mtx, dst=dst)
        lander.track_and_land()
    else:
        land(vehicle)

    if vehicle.mode.name == "LAND":
        print("Landed Successfully")

    complete_mision(vehicle)
       
