
from aruco_tracker import *
from driver import *
from aruco_landing import *


class ScanTrack():
    def __init__(self, vehicle,tracker_id, tracker_size, mtx, dst, waypoints):

        # Marker information
        self.vehicle = vehicle
        self.waypoints = waypoints
        self.detected = False
        self.tracker_id     = tracker_id
        self.tracker_size    = tracker_size
        self.mtx = mtx
        self.dst = dst


    def scan_track(self):
        self.detected = False
        waypoint_list = self.waypoints
        i = 0
        aruco_tracker = ArucoTracker(tracker_id=self.tracker_id, tracker_size=self.tracker_size, gui= False, mtx=self.mtx, dst=self.dst)

        for waypoint in waypoint_list:
            targetLocation = LocationGlobalRelative(
            waypoint[0], waypoint[1], waypoint[2])
            self.vehicle.simple_goto(targetLocation)
            while self.vehicle.mode.name == "GUIDED":
                current = self.vehicle.location.global_relative_frame
                remainingDistance = get_distance_metres(
                self.vehicle.location.global_relative_frame, targetLocation)
                print("Distance to target: ", remainingDistance)
                marker_found, x_cm, y_cm, z_cm, a  = aruco_tracker.big_track(loop=False)
                if marker_found:
                    self.detected = True
                    self.vehicle.simple_goto(current)
                    aruco_tracker.stop()
                    return True

                if remainingDistance <= 1.0:
                    print("Target %d reached " % i)
                    i = i + 1
                    break

        return False


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='')
    args = parser.parse_args()

    print('Connecting...')
    vehicle = connect(args.connect)

    # --------------------------------------------------
    tracker_id = 4
    tracker_size = 29.8  # - [cm]

    mtx = np.loadtxt('calib/cameraMatrix.txt', delimiter=',')
    dst = np.loadtxt('calib/cameraDistortion.txt', delimiter=',')
    waypoints = np.loadtxt('test.waypoints', delimiter='\t', skiprows=2) 

    hovering_height = 4  # In meters
    arm_and_takeoff(vehicle, hovering_height)
    
    scanner = ScanTrack(vehicle=vehicle,tracker_id=tracker_id, tracker_size=tracker_size, mtx=mtx, dst=dst, waypoints = waypoints)
    
    detected = scanner.scan_track()

    if detected:
        lander = ArucoLanding(vehicle = vehicle, tracker_id=tracker_id, tracker_size=tracker_size, mtx=mtx, dst=dst)
        lander.track_and_land()

    if vehicle.mode.name == "LAND":
        print("Landed Successfully")

    complete_mision(vehicle)
