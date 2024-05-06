import math
import ast
import asyncio

import rclpy
from rclpy import Future
import rclpy.duration
from rclpy.node import Node

import rclpy.time
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, TransformException

from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, TransformStamped, Vector3, Pose


import numpy as np
from numpy.linalg import inv



class Locator(Node):

    def __init__(self):
        super().__init__('locator')      
        self.declare_parameter('input_topic', "/marker_publisher/markers")
        self.declare_parameter('output_topic', "/aruco_ros_locator/pose")
        self.declare_parameter('broadcast_tf', True)
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
        
        self.broadcast_tf = self.get_parameter('broadcast_tf').value
        self.ref_frame = self.get_parameter("reference_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.mode = self.get_parameter("mode").value
        self.cov_matrix = self.get_parameter("cov_matrix").get_parameter_value().string_value
        self.cov_matrix = ast.literal_eval(self.cov_matrix)
        self.outTopic = self.get_parameter("output_topic").value
        self.inMarkerTopic = self.get_parameter("input_topic").value

        self.get_logger().info(f"Aruco ros locator node was started with arguments:")
        self.get_logger().info(f"Mode: '{self.mode}'")
        self.get_logger().info(f"broadcast tf?: '{self.broadcast_tf}'")


        self.A = np.eye(7,7)
        self.H = np.eye(7,7)
        self.R = self.cov_matrix
        self.P0 = self.cov_matrix
        self.Q = 0.0001 * np.eye(7,7)
        self.x0_kalm = 0


        self.marker_frame = ""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.broadcast_tf:
            self.tf_broadcast = TransformBroadcaster(self)
        self.final_pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.outTopic, 10)
        self.marker_listener = self.create_subscription(MarkerArray, self.inMarkerTopic, self.markerListener_callback, 10)
        self.marker_listener

    async def get_transform_from__marker_pose(self, marker_pose: Pose, calculate_to_cam=False, override_broadcast_tf=False, fromFrame="", toFrame="") -> TransformStamped:
        #This function is suposed to take pose from aruco_msgs Marker message, propagate 
        #transform to the TF tree and calculate the pose of camera/child frame in regards to the reference frame

        #calculate_to_cam specifies if output should be computed to the camera or child frame
        #override_broadcast_tf specifies if the transform should be sent via /tf or stay locally in tf_buffer (not propagate to all nodes)
        #fromFrame and toFrame can be used to override frames to which should the pose be calculated

        #CREATE MESSAGE
        _t = TransformStamped()
        _t.header.stamp = self.get_clock().now().to_msg()

        #PROPAGATE TRANSFORM FROM MARKER TO CAMERA
        if override_broadcast_tf:
            propagate_tf = False
        else:
            propagate_tf = self.broadcast_tf

        if toFrame == "":
            _toFrame = self.camera_frame
        else:
            _toFrame = toFrame
        
        if calculate_to_cam or self.child_frame == "":
            _toFrame_ret = self.camera_frame
        else:
            _toFrame_ret = self.child_frame

        if fromFrame == "":
            _fromFrame = self.marker_frame
            _fromFrame_ret = self.ref_frame
        else:
            _fromFrame = self.marker_frame
            _fromFrame_ret = fromFrame

        _t.child_frame_id = _toFrame
        _t.header.frame_id = _fromFrame
        _t.transform.translation = Vector3(x=marker_pose.position.x, y=marker_pose.position.y, z=marker_pose.position.z)
        _t.transform.rotation.x = -marker_pose.orientation.x
        _t.transform.rotation.y = -marker_pose.orientation.y
        _t.transform.rotation.z = -marker_pose.orientation.z
        _t.transform.rotation.w = marker_pose.orientation.w
        _t = self.rotateTVec(_t)
        if propagate_tf:
            self.tf_broadcast.sendTransform(_t)
        else:
            self.tf_buffer.set_transform(_t, 'default_authority')


        #RETRIEVE TRANSFORM TO REF_FRAME
        getTransform_future = self.tf_buffer.wait_for_transform_async(_toFrame, _fromFrame, rclpy.time.Time(nanoseconds=0))
        asyncio.wait_for(getTransform_future, timeout=1.0)

        if getTransform_future.done():
            try:
                _t = self.tf_buffer.lookup_transform(_fromFrame_ret, _toFrame_ret, rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(f'Could not transform from {_fromFrame_ret} to {_toFrame_ret}: {ex}')

        return _t


    def average_quaternions(self, _quatArray):

        _qNormalized = []
        for q in _quatArray:
            qArr = [q.x, q.y, q.z, q.w]
            qArr /= np.linalg.norm(qArr)
            _qNormalized.append(qArr)

        _qNormalized = np.array(_qNormalized)

        qMean = np.mean(_qNormalized, axis=0)

        qMean /= np.linalg.norm(qMean)

        return qMean


    def make_average(self, _x, _y, _z, _qArray, _t: TransformStamped):
 
        qOut = _t.transform.rotation
        _t.transform.translation.x = _x / len(_qArray)
        _t.transform.translation.y = _y / len(_qArray)
        _t.transform.translation.z = _z / len(_qArray)
        if(len(_qArray) > 1):
            qOut.x, qOut.y, qOut.z, qOut.w = self.average_quaternions(_qArray)
        else:
            qOut = _qArray[0]

        _t.transform.rotation = qOut

        return _t
    
    async def lookup_transform_async(self, t: TransformStamped, fromFrame, toFrame) -> TransformStamped:
        getTransform_future = self.tf_buffer.wait_for_transform_async(fromFrame, self.camera_frame, rclpy.time.Time(nanoseconds=0))
        asyncio.wait_for(getTransform_future, timeout=1.0)
        if getTransform_future.done():
            try:
                t = self.tf_buffer.lookup_transform(fromFrame, toFrame, rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(f'Could not transform from {fromFrame} to {toFrame}: {ex}')

        return t
    
    def rotateTVec(self, t: TransformStamped) -> TransformStamped:
        trans = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        quat = t.transform.rotation
        xx = quat.x * quat.x
        yy = quat.y * quat.y
        zz = quat.z * quat.z
        xy = quat.x * quat.y
        xz = quat.x * quat.z
        yz = quat.y * quat.z
        wx = quat.w * quat.x
        wy = quat.w * quat.y
        wz = quat.w * quat.z
        ww = quat.w * quat.w
        
        rotation_matrix = np.array([[1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
                            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
                            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]])
        
        rotated_Tvec = np.dot(rotation_matrix, trans)
        t.transform.translation.x = -rotated_Tvec[0]
        t.transform.translation.y = -rotated_Tvec[1]
        t.transform.translation.z = -rotated_Tvec[2]


        
        return t


    async def markerListener_callback(self, msg: MarkerArray):
        t = TransformStamped()
        poseO = Pose()
        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = self.camera_frame
        t.header.frame_id = self.ref_frame

        if len(msg.markers) == 1:

            self.marker_frame = str(msg.markers[0].id)
            t = await self.get_transform_from__marker_pose(msg.markers[0].pose.pose)

        elif self.mode == "NEAREST": 
            closestZ = 100.0
            for marker in msg.markers:
                if marker.pose.pose.position.z < closestZ:
                    self.marker_frame = str(marker.id)
                    closestZ = marker.pose.pose.position.z
                    poseO = marker.pose.pose

            t = await self.get_transform_from__marker_pose(poseO)



        elif self.mode == "AVERAGE":
            qArray = []
            xAvg = 0
            yAvg = 0
            zAvg = 0
            for marker in msg.markers:
                self.marker_frame = str(marker.id)
                tMarker = await self.get_transform_from__marker_pose(marker.pose.pose, True, True)
                xAvg += tMarker.transform.translation.x
                yAvg += tMarker.transform.translation.y
                zAvg += tMarker.transform.translation.z
                qArray.append(tMarker.transform.rotation)
   
            self.make_average(xAvg, yAvg, zAvg, qArray, t)
            if self.broadcast_tf:
                self.tf_broadcast.sendTransform(t)

            if not self.child_frame == "":
                t = await self.lookup_transform_async(t, self.ref_frame, self.child_frame)

        else:
            self.get_logger().error(f"Mode '{self.mode}' doesnt exist!")


        oMsg = PoseWithCovarianceStamped()
        oMsg.header.stamp = t.header.stamp
        oMsg.header.frame_id = t.header.frame_id
        oMsg.pose.pose.position.x = t.transform.translation.x
        oMsg.pose.pose.position.y = t.transform.translation.y
        oMsg.pose.pose.position.z = t.transform.translation.z
        oMsg.pose.pose.orientation = t.transform.rotation
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