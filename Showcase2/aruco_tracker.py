# CSE Drone Team 2020


import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

class ArucoTracker():
    def __init__(self, tracker_id, tracker_size, mtx, dst, camera_size=[640,480], gui=False):
        
        #Marker information
        self.tracker_id     = tracker_id
        self.tracker_size    = tracker_size

        #Aruco dictionary
        self._aruco_dict  = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self._parameters  = aruco.DetectorParameters_create()

        #Camera Configuration
        self._cap = cv2.VideoCapture(0)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_size[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_size[1])
        self._mtx = mtx
        self._dst = dst

        #Helper attributes
        self.font = cv2.FONT_HERSHEY_PLAIN
        self._gui   = gui
        self._t_read      = time.time()
        self._t_detect    = self._t_read
        self.is_detected    = False
        self._kill          = False
 

 
    def stop(self):
        self._kill = True
        self._cap.release()


    def track(self, loop=True, gui=None):
        
        self._kill = False
        if gui is None: gui = self._gui
        
        # initalizing marker tracking.
        detected = False
        x = y = z = a = 0
        
        while not self._kill:
            
            #Reading camera input from rpi camera
            ret, frame = self._cap.read()

            
            if np.shape(frame) ==():
                print("Camera error!")
                self._cap.release()
                exit()
            #-- Converting image frame into gray scale 
            gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 

            #-- Detects all the aruco markers based upon the provided parameters and aruco dictionary.
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self._aruco_dict, 
                            parameters=self._parameters,
                            cameraMatrix=self._mtx, 
                            distCoeff=self._dst)
            idx = -1
            c =0
            if not ids is None:
                for x in ids:
                  
                    if self.tracker_id == x[0]:
                        idx = c
                    c = c + 1
            if idx != -1:
                detected = True
                
                ret_data = aruco.estimatePoseSingleMarkers(corners[idx], self.tracker_size, self._mtx, self._dst)

                #We would need tvec from here as the position vectors are needed to get the markers position with reference to the drone.
                rvec, tvec = ret_data[0][0,0,:], ret_data[1][0,0,:]
                
                # These are the marker position vectors that is required to navigate the drone towards the UTA logo.
                x = tvec[0]
                y = tvec[1]
                z = tvec[2]
                
                angle = math.atan((corners[idx][0][2][1]-corners[idx][0][0][1])/(corners[idx][0][2][0]-corners[idx][0][0][0])) * (180/math.pi)
                yaw_angle = angle
                if angle < 0:
                    yaw_angle = angle + 90
                else:
                    yaw_angle = angle - 90
                a = yaw_angle
                #Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners)
                aruco.drawAxis(frame, self._mtx, self._dst, rvec, tvec, 10)

               
                print ("Marker X = %.1f  Y = %.1f  Z = %.1f  "%(tvec[0], tvec[1], tvec[2]))
                font = self.font
                if gui:

                    #-- Print the tag position in camera frame
                    str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f a = %3.1f "%(tvec[0], tvec[1], tvec[2],a)
                    cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)        
                    

            else:
                print("Nothing detected ")
            
            # displaing iamge in the screen can be computationally heavy for pi when ardupilot is also running on parallel.
            # make sure this is off when the drone isd flying.
            if gui:
                #--- Display the frame
                cv2.imshow('frame', frame)

                #--- use 'q' to quit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self._cap.release()
                    cv2.destroyAllWindows()
                    break
            
            if not loop:
                 # returning the positions of the logo
                self._kill =True
                
        return (detected, x, y, z, a)
    
    def small_track(self, loop=False, gui=None):

        self.tracker_id  = 15
        self.tracker_size = 10.1
        (detected, x, y, z, a) =  self.track(loop=loop, gui =gui)
        return (detected, x, y, z ,a)


    def big_track(self, loop=False, gui=None):

        self.tracker_id  = 4
        self.tracker_size = 35.0
        (detected, x, y, z ,a) =  self.track(loop=loop, gui =gui)
        return (detected, x, y, z, a)
           

if __name__ == "__main__":

    tracker_id  = 15
    tracker_size  = 10.1 #- [cm]


    # path to the camera matrix and distortion.
    # Our Raspberry pi HQ camera doesn't have much distoration but these values are absolutely required to detect the marker
    # as without these will be passed later to the detector fuction. 
    mtx   = np.loadtxt('calib/cameraMatrix.txt', delimiter=',')
    dst   = np.loadtxt('calib/cameraDistortion.txt', delimiter=',') 

    # creating our aruco tracker object.                                     
    aruco_tracker = ArucoTracker(tracker_id=tracker_id, tracker_size=tracker_size, gui= True, mtx=mtx, dst=dst)
    
    # intializing tracker for the specific id of the logo.
    aruco_tracker.track()
