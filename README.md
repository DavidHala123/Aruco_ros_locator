# About aruco_ros_locater
This project is in stage of ongoing development as part of master thesis regarding robot localization using aruco_ros library.

## Important information
This project is based on aruco_ros library by pal-robotics (version humble-devel). In order to run this code it is necessary to follow theese steps:
- Download aruco_ros library from [here](https://github.com/pal-robotics/aruco_ros) 
* Place the whole downloaded library in your_ros_workspace/src
+ Replace CmakeLists.txt in your_ros_workspace/src/aruco_ros-humble-devel/aruco_ros with CmakeLists file located in this repository
+ Add tf_publisher.cpp from this repository to your_ros_workspace/src/aruco_ros-humble-devel/aruco_ros/src
+ Add tf_publisher.cpp from this repository to your_ros_workspace/src/aruco_ros-humble-devel/aruco_ros/launch

Keep in mind that for this project to run it is required to have openCV v4.2.0 installed on your computer. You can install the library from [here](https://github.com/opencv/opencv/releases/tag/4.2.0)

Please make sure that camera input broadcasts data to '/image_raw' topic and camera calibration information to '/camera_info' topic
