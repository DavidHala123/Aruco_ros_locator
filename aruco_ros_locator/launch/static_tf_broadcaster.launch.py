from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    aruco_locator_params = {
        'file_has_child_frame': True,
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': LaunchConfiguration('camera_frame'),
        'child_frame': LaunchConfiguration('child_frame'),
        'mode': LaunchConfiguration('mode'),
        'cov_matrix': LaunchConfiguration('cov_matrix'),
    }

    aruco_locator = Node(
        package='aruco_ros_locator',
        executable='locator',
        parameters=[aruco_locator_params],
    )

    return [aruco_locator]

def generate_launch_description():

    file_path_arg = DeclareLaunchArgument(
        'setup_file_path', default_value='/home/david/ros2_ws/src/aruco_ros_locator/data/setup.txt',
        description='Path to where is file located. '
    )

    ref_frame_arg = DeclareLaunchArgument(
        'ref_frame_name', default_value='map',
        description='Name of reference frame. '
    )

    camera_frame_arg = DeclareLaunchArgument(
        'cam_frame_name', default_value='camera',
        description='Name of camera frame'
    )

    child_frame_arg = DeclareLaunchArgument(
        'child_frame_name', default_value='body',
        description="Name of child frame (only needed if 'file_has_child_frame' is set to true)"
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(file_path_arg)
    ld.add_action(ref_frame_arg)
    ld.add_action(camera_frame_arg)
    ld.add_action(child_frame_arg)  

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
