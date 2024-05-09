# About aruco_ros_locator
This project was created under the auspices of [BUT FEKT](https://www.vut.cz/) as part of the master thesis regarding robot localization using aruco_ros library.

## Important information
This project is based on aruco_ros library by pal-robotics (version humble-devel). In order to run this code it is necessary to follow theese steps:
+ Download OpenCV library v4.2.0 from [here](https://github.com/opencv/opencv/releases/tag/4.2.0) and install it using cmake (REQUIRED)
+ Download aruco_ros library from [here](https://github.com/pal-robotics/aruco_ros)
+ Place aruco_ros, this project and your prefered cam node to "your_ros_workspace/src"
+ Colcon build the workspace

Please make sure that camera input broadcasts data to '/image_raw' topic and camera calibration information to '/camera_info' topic

This library was developed and tested with following libraries: [aruco_ros](https://github.com/pal-robotics/aruco_ros), [usb_cam](https://github.com/ros-drivers/usb_cam), [image_pipeline](https://github.com/ros-perception/image_pipeline)

## ----HOW TO USE----

# locator

Main node that returns transform between ref_frame and cam_frame/child_frame
+ Specify 'broadcast_tf' (if camera node is to be broadcasted to /tf)
+ Specify reference and camera frame
+ Specify child frame (if empty, tf will be calculated from ref_frame to cam_frame)
+ Specify 'mode' of localization
+ Specify covariance matrix
+ Launch locator

# static_tf_broadcaster

The essence of this node is to populate tf tree.
+ Create setup file - [example](https://github.com/DavidHala123/Aruco_ros_locator/blob/main/aruco_ros_locator/data/setup.txt)
+ + "name" "Tx" "Ty" "Tz" "Rx" "Ry" "Rz" 
+ Specify 'resend_when_subs_changed' (if True, static tf will be resent if subscribers count increases)
+ Launch static_tf_subscriber

# Accuracy_meas

The essence of this node is the calculation of statistical variables that make it easier to calculate the covariance matrix and other variables. The output of this node contains std, RMSE and worksheet with all the measured values to process the eigenvariables.
+ Record all the measurements with ros_bag
+ Create a folder that contains all the rosbag files. All bag files in created folder must be sorted into folders whose names correspond to the measurement distances
+ Specify the root folder (with rosbag files) in launch file
+ Specify the 'folders_corresponds_to_axis' variable (specifies which axis corresponds to the name of folders with bagfiles)
+ Specify real values for x, y, z, Rx, Ry, Rz (you can leave the 'folders_corresponds_to_axis' axis empty, other empty axes will be considered 0)
+ Specify camera info path (calibration file) ->Added due to incosistency with accepting cam_info from rosbag file<-
+ Specify ref, camera frame in locator (if you are measuring distance from marker, use marker frame name as reference)
+ Populate tree (for instance static_tf_broadcaster)
+ Run marker_detector (aruco_ros)
+ Run locator (aruco_ros_locator)
+ Launch accuracy_meas

## ----RQT_GRAPH----

![rosgraph](https://github.com/DavidHala123/Aruco_ros_locator/assets/78861269/bc95afa2-1207-4c78-9f66-d78489a36397)
