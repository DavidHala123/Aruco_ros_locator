![rosgraph](https://github.com/DavidHala123/Aruco_ros_locater/assets/78861269/ac68c29a-6ac8-4eb7-b0c7-79495d76db0b)# About aruco_ros_locater
This project is in stage of ongoing development as part of master thesis regarding robot localization using aruco_ros library.

## Important information
This project is based on aruco_ros library by pal-robotics (version humble-devel). In order to run this code it is necessary to follow theese steps:
+ Download OpenCV library v4.2.0 from [here](https://github.com/opencv/opencv/releases/tag/4.2.0) and install it using cmake (REQUIRED)
+ Download aruco_ros library from [here](https://github.com/pal-robotics/aruco_ros)
+ Place the aruco_ros library in "your_ros_workspace/src"
+ Replace CmakeLists.txt in "your_ros_workspace/src/aruco_ros-humble-devel/aruco_ros" with CmakeLists file located in this repository
+ Add tf_publisher.cpp from this repository to "your_ros_workspace/src/aruco_ros-humble-devel/aruco_ros/src"
+ Add tf_publisher_launch.py from this repository to "your_ros_workspace/src/aruco_ros-humble-devel/aruco_ros/launch"
+ Add diplomova_prace folder inside "your_ros_workspace/src"
+ Colcon build the workspace

Please make sure that camera input broadcasts data to '/image_raw' topic and camera calibration information to '/camera_info' topic

## How to use
1. `ros2 run rviz2 rviz2`
2. `ros2 run diplomova_prace tf_subscriber`
3. `ros2 run diplomova_prace static_tf_broadcast` (broadcasts static marker, body and camera positions)
4. Run your prefered camera node with correct output topics
5. `ros2 launch aruco_ros tf_publisher_launch.py` (scan the image for markers and outputs result image to '/aruco_tf/result' topic and transformation to '/tf' topic)

## RQT_GRAPH




## KNOWN BUGS
1. Low framerate when marker detected
