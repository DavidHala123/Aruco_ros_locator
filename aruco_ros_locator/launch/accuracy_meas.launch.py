from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    accuracy_meas_params = {
        'folders_corresponds_to': LaunchConfiguration('folders_corresponds_to'),
        'rosbag_path': LaunchConfiguration('rosbag_path'),
        'real_x': LaunchConfiguration('real_x'),
        'real_y': LaunchConfiguration('real_y'),
        'real_z': LaunchConfiguration('real_z'),
        'real_rx': LaunchConfiguration('real_rx'),
        'real_ry': LaunchConfiguration('real_ry'),
        'real_rz': LaunchConfiguration('real_rz'),
        'cam_info_path': LaunchConfiguration('cam_info_path'),
    }

    accuracy_meas = Node(
        package='aruco_ros_locator',
        executable='accuracy_meas',
        parameters=[accuracy_meas_params],
    )

    return [accuracy_meas]


def generate_launch_description():

    folders_corresponds_to_arg = DeclareLaunchArgument(
        'folders_corresponds_to', default_value='z',
        description='Specify which axis folder names corresponds to. '
    )

    rosbag_path_arg = DeclareLaunchArgument(
        'rosbag_path', default_value='/home/david/mereni',
        description='Specify the parent folder of all rosbag files. '
    )

    cam_info_path_arg = DeclareLaunchArgument(
        'cam_info_path', default_value='/home/david/ros2_ws_old/src/usb_cam-ros2/config/ost.yaml',
        description='Specify measured x, if empty the array will be considered as having zeros. '
    )

    real_x_arg = DeclareLaunchArgument(
        'real_x', default_value='',
        description='Specify measured x, if empty the array will be considered as having zeros. '
    )

    real_y_arg = DeclareLaunchArgument(
        'real_y', default_value='',
        description='Specify measured y, if empty the array will be considered as having zeros. '
    )

    real_z_arg = DeclareLaunchArgument(
        'real_z', default_value='',
        description='Specify measured z, if empty the array will be considered as having zeros. '
    )

    real_rx_arg = DeclareLaunchArgument(
        'real_rx', default_value='',
        description='Specify measured x, if empty the array will be considered as having zeros. '
    )

    real_ry_arg = DeclareLaunchArgument(
        'real_ry', default_value='',
        description='Specify measured x, if empty the array will be considered as having zeros. '
    )

    real_rz_arg = DeclareLaunchArgument(
        'real_rz', default_value='',
        description='Specify measured x, if empty the array will be considered as having zeros. '
    )

    
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(folders_corresponds_to_arg)
    ld.add_action(rosbag_path_arg)
    ld.add_action(real_x_arg)
    ld.add_action(real_y_arg)  
    ld.add_action(real_z_arg)
    ld.add_action(real_rx_arg)
    ld.add_action(real_ry_arg)  
    ld.add_action(real_rz_arg)
    ld.add_action(cam_info_path_arg)


    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
