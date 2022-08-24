# SURE MULE
## Basic Overview
This is the github repo containing the ROS packages created for the rover. Currently the the package is organied into six nodes in the object_tracking ROS package:

- target_finder
- gimbal_controller
- obstacle_detector
- local_planner
- navigator
- logger

Target finder locates the person using jetson inference posenet and a gimbal mounted Intel Realsense d415.
The gimbal controller both rotates the d415 to center the person within the image and transforms the the target position to be relative to the robot instead of relative to the camera
Obstacle detector uses the Intel Realsense d435i camera to scan for obstacle and broadcasts the imu readings produced by the d435. 
The local planner fuses this information to produce a trajectory for the robot to follow. 
Navigator takes this data and uses a pid loop, odometry information, and eventually velocity estimates for the person to compute the direct commands to give to the robot. 
A logger additionally exists to log the info in a csv file for later analysis.

There is also another package, test_logs, used to contain some test scripts. Particularly grabDepth.py allows the user to play around with how the obstacle detection works.


## Requirements
ROS melodic:
- rospy
- ds4_driver
- geometry_msgs
- robot_localization
- roverrobotics_driver

Non-ROS requirements
- pyrealsense2
- numpy
- cv2
- jetson inference to be installed from source
- custom nvidia cuda code (not yet document (or on github for that matter)

## Install

Clone this directory into current catkin workspace
in catkin workspace root folder call:

> catkin_make

Follow guide to install gpu code https://github.com/msstateACosby/cuda_obstacle_detection/blob/main/README.md

## Run

Start the obstacle detector:

> roslaunch object_tracking obstacle_detector.launch

Once that is running (wait about 15 seconds to make sure it doesnt crash because the camera failed to return a frame):

> roslaunch object_tracking obj_tracking.launch

Once those are both running (the gimbal will be tracking) press triangle on the controller to start running.
Press square to stop.

The robot will log its data into new folders created at ~/logs/{time_starting}. Due to not all nodes starting at same time this will likely be spread across multiple folders

To change the folder logs are stored at, when calling roslaunch both times add: folder:="your folder name choice"

To change which path planner is used, add: path_planner:="whatever path planner you want"
Current options are apf and vfh


## More Detailed Node Descriptions

### Target Finder Node

Reads data from the D415 camera and processes it with jetson inference
to find the location of a person within each frame.

Subscribes To:
- /status : PS4 controller status

Publishes:
- /raw_target_loc : the location of the person relative to the camera
  - coordinate space:
    - x : -left +right
    - y : -up +down
    - z : +forwards -backwards

### Gimbal Controller Node

Controls two servos to attempt to center the target within the D415 camera tracking the target

Subscribes To:
- /status: the status of the PS4 controller
- /raw_target_loc : the location of the person relative to the camera

Publishes:
- /target_loc_rotation_corrected : the location of the person relative to the robot
  - coordiante space:
    - x: left-right
    - y: up-down
    - z: forwards-backwards
- /camera_rot: the yaw and pitch rotation of the camera in radians

### Obstacle Detector Node

Uses the D435 camera to find obstacles in front of the robot
there is definitely something wierd in this script. 
Also broadcasts the imu data from the d435i camera

It sometimes crashes with errors that don't really make sense
it seems like there might be a cuda function called by pyrealsense2 that fails, 
I dont know, maybe my cuda usage is somehow modifies some cuda state 
that pyrealsense2 doesn't expect to be modified?
hasnt happened in a while though...

Subscribes To:
- /status: the status of the PS4 controller

Publishes:
- /obstacles: a list of circlular obstacles relative to the rover
  - coordinate space:
    - x: -left +right
    - z: +forwards -backwards
- /d435_imu: imu data from the d435i
    
### Local Planner Node

Combines the target location data and obstacle location data
produces a trajectory for the robot to head towards

Subscribes To:
- /status : PS4 controller status
- /target_loc_rotation_corrected : the position of the target relative to the robot
- /obstacles : the collection of obstacles in front of the robot

Publishes:
- /trajectory : a msg describing a relative angle the robot should turn towards and a how far to move linearly
                
### Navigator Node

Controls PID loops to control robot wheel motors

Subscribes To:
- /status: the status of the PS4 controller
- /trajectory: the destination to head towards
- /odom/ekf/enk_imu: the filtered odometry info of the robot

Publishes:
- /cmd_vel: motor commands to be interpreted by the motor driver
  - coordinate system:
    - x: +forward -backward
    - y: +right - left
    - z: +up -down
  
### Logger Node

Collects data from a lot of places and logs it to a csv

Subscribes To:
- /status: PS4 controller status
- /raw_target_loc: target location relative to camera
- /trajectory: error to desired distance and angle
- /cmd_vel: robot commanded velocity
- /odom/ekf/enk_imu: kalman-filtered robot odometry
- /camera_rot: rotation of the d415 camera

### Other Helpful Information
Almost every node collects information it recieves from callbacks into a dictionary named container.
This basically is there to allow the node to access all of the data from one place instead of worry about really wierd callback schenanigans.

Parameters fed to nodes as a whole are not contained within launch files for the most part but instead exist within launch/params. 
There you should be able to find a file for each node with its related parameters. 
You might notice that some nodes require parameters from multiple files, for instance the local planner not only needs local_planner.yaml but also obstacle_detector.yaml.
This is because the local planner would also like some information about how obstacles are detected (in this the max depth they are detected at).

The logger node can be expanded to log more information quite easily. 
For constants, which are likely parameters, add the get_param() for the constant in the logger and assign it to a variable. Then scroll down till you see a long set of strings being written to a csv file. Add whatever you want the name of the constant to be logged as to the set. 
Then scroll down further and add the actual variable assigned with get_param() to the corresponding location in the next list.
Make sure they are in the same relative position or else any variables afterwwards may not line up with their names in the file.

For time series data, write a callback to recieve the message and store it in the 'container' dictionary. In run() where 'container' is created, add the data entry there as well along with adding the dictionary key to the list called fieldnames.

## Misc
The custom badge detection model expects to find it at /home/rover/jetson-inference/python/training/detection/ssd/models/test/ssd-mobilenet.onnx currently

