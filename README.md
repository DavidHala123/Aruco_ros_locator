# About aruco_ros_locator
This project is in stage of ongoing development as part of master thesis regarding robot localization using aruco_ros library.

## Important information
This project is based on aruco_ros library by pal-robotics (version humble-devel). In order to run this code it is necessary to follow theese steps:
+ Download OpenCV library v4.2.0 from [here](https://github.com/opencv/opencv/releases/tag/4.2.0) and install it using cmake (REQUIRED)
+ Download aruco_ros library from [here](https://github.com/pal-robotics/aruco_ros)
+ Place both aruco_ros and this project in "your_ros_workspace/src"
+ Colcon build the workspace

Please make sure that camera input broadcasts data to '/image_raw' topic and camera calibration information to '/camera_info' topic

## RQT_GRAPH

//![rosgraph](https://github.com/DavidHala123/Aruco_ros_locator/assets/78861269/bc95afa2-1207-4c78-9f66-d78489a36397)
