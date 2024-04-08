import math
import ast
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster, TransformListener, Buffer, TransformException

from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, TransformStamped, Vector3, Pose



class Locator(Node):

    def __init__(self):
        super().__init__('tf_subscriber')
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("camera_frame", "camera")
        self.declare_parameter("child_frame", "body")
        self.declare_parameter("mode", "AVERAGE")
        self.declare_parameter("cov_matrix", 
                            "[0.1, 0.0, 0.0, 0.0, 0.0, 0.0," +
                            "0.0, 0.1, 0.0, 0.0, 0.0, 0.0," +
                            "0.0, 0.0, 0.1, 0.0, 0.0, 0.0," +
                            "0.0, 0.0, 0.0, 1, 0.0, 0.0," +
                            "0.0, 0.0, 0.0, 0.0, 1, 0.0," +
                            "0.0, 0.0, 0.0, 0.0, 0.0, 1 ]")
        
        self.ref_frame = self.get_parameter("reference_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.mode = self.get_parameter("mode").value
        self.cov_matrix = self.get_parameter("cov_matrix").get_parameter_value().string_value
        self.cov_matrix = ast.literal_eval(self.cov_matrix)


        self.marker_frame = ""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcast = TransformBroadcaster(self)
        self.final_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/aruco_ros_locator/pose', 10)
        self.marker_listener = self.create_subscription(MarkerArray, '/marker_publisher/markers', self.markerListener_callback, 10)
        self.marker_listener



    def calculate_pose(self, mArray: MarkerArray) -> Pose:
        outputPose = Pose()

        if self.mode == "NEAREST": 
            closestZ = 100.0
            for marker in mArray:
                if marker.pose.pose.position.z < closestZ:
                    self.marker_frame = "Id" + str(marker.id)
                    closestZ = marker.pose.pose.position.z
                    outputPose = marker.pose.pose

        elif self.mode == "AVERAGE":
            self.marker_frame = self.ref_frame
            for marker in mArray:
                outputPose.position.x += marker.pose.pose.position.x
                outputPose.position.y += marker.pose.pose.position.y
                outputPose.position.z += marker.pose.pose.position.z

                outputPose.orientation.x += marker.pose.pose.orientation.x
                outputPose.orientation.y += marker.pose.pose.orientation.y
                outputPose.orientation.z += marker.pose.pose.orientation.z

            outputPose.position.x /= len(mArray)
            outputPose.position.y /= len(mArray)
            outputPose.position.z /= len(mArray)

            outputPose.orientation.x /= len(mArray)
            outputPose.orientation.y /= len(mArray)
            outputPose.orientation.z /= len(mArray)
        else:
            self.get_logger().error(f"Mode was specified as: {self.mode} but such a mode doesnt exist!")

        return outputPose



    def markerListener_callback(self, msg: MarkerArray):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = self.camera_frame

        poseO = self.calculate_pose(msg.markers)

        t.header.frame_id = self.marker_frame
        t.transform.translation = Vector3(x=poseO.position.x, y=poseO.position.y, z=poseO.position.z)
        t.transform.rotation = poseO.orientation
        self.tf_broadcast.sendTransform(t)
        
        try:
            if(self.child_frame != ""):
                t = self.tf_buffer.lookup_transform(self.child_frame, self.ref_frame, rclpy.time.Time())
            else:
                t = self.tf_buffer.lookup_transform(self.camera_frame, self.ref_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.ref_frame} to {self.child_frame}: {ex}')

        oMsg = PoseWithCovarianceStamped()
        oMsg.header.stamp = t.header.stamp
        oMsg.header.frame_id = t.header.frame_id
        oMsg.pose.pose = poseO
        oMsg.pose.covariance = self.cov_matrix
        self.final_pose_pub.publish(oMsg)
        


def main():
    rclpy.init()
    node = Locator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()