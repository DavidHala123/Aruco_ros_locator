from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    aruco_locator_params = {
        'broadcast_tf': True,
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': LaunchConfiguration('camera_frame'),
        'child_frame': LaunchConfiguration('child_frame'),
        'mode': LaunchConfiguration('mode'),
        'cov_matrix': LaunchConfiguration('cov_matrix'),
        'input_topic': LaunchConfiguration('input_topic'),
        'output_topic': LaunchConfiguration('output_topic'),
    }

    aruco_locator = Node(
        package='aruco_ros_locator',
        executable='locator',
        parameters=[aruco_locator_params],
    )

    return [aruco_locator]


def generate_launch_description():

    ref_frame_arg = DeclareLaunchArgument(
        'reference_frame', default_value='map',
        description='Reference frame for TF. '
    )

    cam_frame_arg = DeclareLaunchArgument(
        'camera_frame', default_value='camera',
        description='Camera frame for TF. '
    )

    child_fram_arg = DeclareLaunchArgument(
        'child_frame', default_value='',
        description='Leave empty if TF is suposed to be computed towards camera. '
    )

    mode_arg = DeclareLaunchArgument(
        'mode', default_value='NEAREST',
        description='Mode of final pose calculation (NEAREST / AVERAGE). '
    )

    cov_matrix_arg = DeclareLaunchArgument(
        'cov_matrix', default_value='0.1, 0.0, 0.0, 0.0, 0.0, 0.0,' +
                                    '0.0, 0.1, 0.0, 0.0, 0.0, 0.0,' +
                                    '0.0, 0.0, 0.1, 0.0, 0.0, 0.0,' +
                                    '0.0, 0.0, 0.0, 1.0, 0.0, 0.0,' +
                                    '0.0, 0.0, 0.0, 0.0, 1.0, 0.0,' +
                                    '0.0, 0.0, 0.0, 0.0, 0.0, 1.0',
        description='Covariance matrix. '
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic', default_value='/marker_publisher/markers',
        description='Specify the name of input topic for message type MarkerArray (aruco_ros package). '
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic', default_value='/aruco_ros_locator/pose',
        description='Specify the name of output topic for message of type PoseWithCovariance. '
    )
    
    
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(ref_frame_arg)
    ld.add_action(cam_frame_arg)
    ld.add_action(child_fram_arg)
    ld.add_action(mode_arg)  
    ld.add_action(cov_matrix_arg)
    ld.add_action(input_topic_arg)
    ld.add_action(output_topic_arg)


    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld