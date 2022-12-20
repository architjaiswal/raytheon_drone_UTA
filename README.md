Yes, we won the competition! Here is the drone performance video from the competition: https://youtu.be/lTXVziEVrIc

Make sure the drone's parameter are set respectively from the mission planner before your first test flight.
Tune your drone using tuning instructions from ardupilot. Be sure to properly tune the settings in Mission Planner by following instructions given on ArduCopeter as per propeller size. Here are more details about the tuning process: https://ardupilot.org/copter/docs/tuning-process-instructions.html 



Typical execution:

$ python3 nav.py --connect /dev/ttyACM0
( povided that you are in the current working directory and pixhawk is connected as a device at /dev/ttyACM0)



Code Structure:

Showcase 1: Commands the drone to take off and move to a distance of 30 yards at its current orientation.
            Make sure to install dronekit, mavlink python libraries

Showcase 2: Commands the drone to scan a section of the field respective to the geofence given during the execution of the program.
            Make sure to python opencv and setup and connect raspberry pi to the pixhawk or any availabel flight controller.
            Make sure the camera is calibrated and correct camera marix and distortion coefficients are passed
            Make sure the correct geofence cordinates are passed.( Modify the Section.waypoints)
            For geofence, select any two corner points of the section of an area you want to scan from mission planner and place in the working directory.
            For my test run, I selected top right and bottom left geo-cordinates.
            To change the orientation of the scan path_planner function in drover.py should be modified.
            driver.py consists necessay navigation function
            aruco_tracker.py consists OpenCV stuff to track the logo
            aruco_landing.py consists of precision tracking, descending and landing
            scan_track.py consists of navigation and tracking at the same time.
            nav.py is the main point of execution of the whole program


Showcase 3: Setup Obstacle avoidance form the mission planner before executing this script.
            Make sure the drone detects and acts accoridingly when it sees and obstacles
            Follow everything in the Showcase 2 for execution.


Resources: 

Ardupilot Documentation
Python Dronekit API
OpenCV
