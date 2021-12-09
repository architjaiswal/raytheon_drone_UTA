
from aruco_tracker import *
from driver import *


class ArucoLanding():
    def __init__(self, vehicle, tracker_id, tracker_size, mtx, dst ):
        
        #Marker information
        self.vehicle = vehicle
        self.current_alt = vehicle.location.global_relative_frame.alt*100.0

        self.a_tracker = ArucoTracker(tracker_id=tracker_id, tracker_size=tracker_size, gui= False, mtx=mtx, dst=dst)
    

        # distance units are in cm.
        self.landing_altitude = 150.0
        self.landing_speed = 60.0
        #self.track_diff_alt = 150.0
        # angle is in radian
        self.angle_descend = 20 * math.pi / 180
        self.time_0 = time.time()
        
        # this calls the aruco tracker function with the provided frequency.
        # Changed freq to agressively chage drone's position 
        self.freq = 2
        self.rad_2_deg   = 180.0/math.pi
        self.deg_2_rad   = 1.0/self.rad_2_deg 


    def get_location_metres(self, original_location, dNorth, dEast):

        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    
        print ("dlat, dlon %d , %d", (dLat, dLon))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        return(newlat, newlon)

    def marker_position_to_angle(self, x, y, z):
    
        angle_x = math.atan2(x,z)
        angle_y = math.atan2(y,z)
    
        return (angle_x, angle_y)
    
    def camera_to_uav(self, x_cam, y_cam):
        x_uav =-y_cam
        y_uav = x_cam
        return(x_uav, y_uav)
    
    def uav_to_ne(self, x_uav, y_uav, yaw_rad):
        c       = math.cos(yaw_rad)
        s       = math.sin(yaw_rad)

        north   = x_uav*c - y_uav*s
        east    = x_uav*s + y_uav*c 
        return(north, east)
    
    def check_angle_descend(self, angle_x, angle_y, angle_desc):
        return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)
        

    def track_and_land(self):

        current_location = self.vehicle.location.global_relative_frame
        last_known = current_location
        last_found = time.time()
        last_corrected = time.time()
        while True:

            if self.vehicle.location.global_relative_frame.alt*100 <= self.landing_altitude:
                if self.vehicle.mode == "GUIDED":
                    print(" -->>Drone Landing <<")
                    self.vehicle.mode = "LAND"
                    return True

            marker_found =False
            x_cm = 0.0
            y_cm = 0.0
            z_cm = 0.0
            a =0.0
            #if self.current_alt > self.track_diff_alt :
            #    marker_found, x_cm, y_cm, z_cm = self.a_tracker.big_track(loop=False)
            #else:
            #    marker_found, x_cm, y_cm, z_cm = self.a_tracker.small_track(loop=False)
            # Mod Added ignoring small tracker. 04/19/2021
            big_marker_found, b_x_cm, b_y_cm, b_z_cm, a = self.a_tracker.big_track(loop=False)
            small_marker_found, x_cm, y_cm, z_cm, a = self.a_tracker.small_track(loop=False)
            
            if (big_marker_found or small_marker_found):
                marker_found = True
                if (big_marker_found):
                    x_cm = b_x_cm
                    y_cm = b_y_cm
                    z_cm = b_z_cm

            if marker_found:
                x_cm, y_cm          = self.camera_to_uav(x_cm, y_cm)
                uav_location        = self.vehicle.location.global_relative_frame
                print("bookmarked co-ord: %s " % (uav_location))
                current_location = uav_location
                last_found = time.time()
                last_corrected = last_found
                #-- If high altitude, use baro rather than visual
                if uav_location.alt >= 5.0:
                    z_cm = uav_location.alt*100.0
                    self.current_alt = z_cm
            
                angle_x, angle_y    = self.marker_position_to_angle(x_cm, y_cm, z_cm)

        
                if time.time() >= self.time_0 + 1.0/self.freq:
                    self.time_0 = time.time()
            
                    print (" ")
                    print ("Altitude = %.0fcm" % z_cm)
                    print ("Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x*self.rad_2_deg, angle_y*self.rad_2_deg))
            
                    north, east  = self.uav_to_ne(x_cm, y_cm, self.vehicle.attitude.yaw)
                    print ("Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg"%(north, east, self.vehicle.attitude.yaw*self.rad_2_deg))
            
                    marker_lat, marker_lon  = self.get_location_metres(uav_location, north*0.01, east*0.01)  
                    #-- If angle is good, descend
                    if self.check_angle_descend(angle_x, angle_y, self.angle_descend):
                        print ("Low error: descending")
                        location_marker  = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(self.landing_speed*0.01/self.freq))
                    else:
                        location_marker  = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
                
                    self.vehicle.simple_goto(location_marker)
                    print ("UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon))
                    print ("Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon))
                
                self.current_alt = self.vehicle.location.global_relative_frame.alt*100.0
                
                #--- Command to land
                if self.current_alt <= self.landing_altitude:
                    if self.vehicle.mode == "GUIDED":
                        print(" -->>Drone Landing <<")
                        self.vehicle.mode = "LAND"
                        return True
            elif (not marker_found) and (time.time()-last_corrected) > 20.0:
                #t = time.time()
                conversion_m_lat  = (0.00001/ 1.11)
                correct_location = self.vehicle.location.global_relative_frame
                correct = [[correct_location.lat + conversion_m_lat*1 , correct_location.lon, correct_location.alt]
                        ,[correct_location.lat - conversion_m_lat*1, correct_location.lon, correct_location.alt],
                        [correct_location.lat, correct_location.lon + conversion_m_lat*1, correct_location.alt],
                        [correct_location.lat, correct_location.lon - conversion_m_lat*1, correct_location.alt],
                        [correct_location.lat- conversion_m_lat*1, correct_location.lon- conversion_m_lat*1, correct_location.alt],
                        [correct_location.lat+ conversion_m_lat*1, correct_location.lon + conversion_m_lat*1, correct_location.alt],
                        [correct_location.lat, correct_location.lon, correct_location.alt]]
                for idx in range(0,7):

                    correct_location = self.vehicle.location.global_relative_frame
                    targetLocation = LocationGlobalRelative(correct[idx][0], correct[idx][1], correct[idx][2])
                    self.vehicle.simple_goto(targetLocation)
                    remainingDistance = get_distance_metres(
                    self.vehicle.location.global_relative_frame, targetLocation)
                    remainingDistance = get_distance_metres(
                    self.vehicle.location.global_relative_frame, targetLocation)
                    marker_found = False
                    while(remainingDistance >=0.5):
                        print("Distance to target: ", remainingDistance)
                        big_marker_found, b_x_cm, b_y_cm, b_z_cm, a = self.a_tracker.big_track(loop=False)
                        small_marker_found, x_cm, y_cm, z_cm, a = self.a_tracker.small_track(loop=False)

                        if (big_marker_found or small_marker_found):
                            marker_found = True
                            last_found = self.vehicle.location.global_relative_frame
                            self.vehicle.simple_goto(last_found)
                    

                        if (big_marker_found):
                            x_cm = b_x_cm
                            y_cm = b_y_cm
                            z_cm = b_z_cm

                        remainingDistance = get_distance_metres( self.vehicle.location.global_relative_frame, targetLocation)
                        if marker_found:
                            self.detected = True
                            break
                    if marker_found:
                        break           
            
            if time.time()- last_found > 40.0:
                print('Can not detect or correct the precise landing.')
                print('Landing in the last known location of logo.')
                #self.vehicle.simple_goto(current_location)
                self.vehicle.mode  = "LAND"
                return True


                    
        
if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default = '')
    args = parser.parse_args()
    

    print('Connecting...')
    vehicle = connect(args.connect)  

    #--------------------------------------------------
    tracker_id      = 4
    tracker_size     = 29.8 #- [cm]

    mtx   = np.loadtxt('calib/cameraMatrix.txt', delimiter=',')
    dst   = np.loadtxt('calib/cameraDistortion.txt', delimiter=',') 

    # creating our aruco landing object.                                     
    aruco_landing = ArucoLanding(vehicle = vehicle, tracker_id=tracker_id, tracker_size=tracker_size, mtx=mtx, dst=dst)
    
    # intializing landing for the specific id of the logo.
    aruco_landing.track_and_land()
