import rclpy
from rclpy.node import Node
import os
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
import math
import numpy as np

class static_tf_broadcast(Node):

    def __init__(self):
        super().__init__('static_tf_broadcast')
        self.declare_parameter('setup_file_path', "")
        self.declare_parameter('file_has_child_frame', False)
        self.declare_parameter("ref_frame_name", "map")
        self.declare_parameter('cam_frame_name', "camera")
        
        self.path = self.get_parameter('setup_file_path').value
        self.ref_frame = self.get_parameter('ref_frame_name').value
        self.cam_frame = self.get_parameter('cam_frame_name').value
        self.file_has_child_frame = self.get_parameter('file_has_child_frame').value

        self.tf_broadcast = StaticTransformBroadcaster(self)
        self.get_logger().info(f"Populating tf tree:")

        self.populate_tree()


    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q



    def sendTransform(self, pose, id):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.ref_frame

        t.child_frame_id = pose[0]
        quat = self.quaternion_from_euler(float(pose[4]), float(pose[5]), float(pose[6]))

        t.transform.translation = Vector3(x = float(pose[1]), y = float(pose[2]), z = float(pose[3]))
        t.transform.rotation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])
        self.tf_broadcast.sendTransform(t)
        self.get_logger().info(f"Created transform of frame '{t.child_frame_id}' connected to '{t.header.frame_id}' frame")


    def create_cam_frame(self, pose):
        
        t = TransformStamped()
        self.get_logger().info("CREATING CAM")
        t.header.stamp = self.get_clock().now().to_msg()

        if self.file_has_child_frame:
            
            t.header.frame_id = self.cam_frame
            t.child_frame_id = pose[0]
            quat = self.quaternion_from_euler(float(pose[4]), float(pose[5]), float(pose[6]))
            t.transform.translation = Vector3(x = float(pose[1]), y = float(pose[2]), z = float(pose[3]))
            t.transform.rotation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])
        else:
            t.child_frame_id = self.cam_frame
            t.transform.translation = Vector3(x = 0.0, y = 0.0, z = 0.0)
            t.transform.rotation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 0.0) 

        self.tf_broadcast.sendTransform(t)
        self.get_logger().info(f"Created transform of frame '{t.child_frame_id}' connected to '{t.header.frame_id}' frame")

        

    def populate_tree(self):
        with open(self.path, 'r') as file:
            line_nr = 1
            for line in file:
                pose = line.split()

                if len(pose) == 7:
                    if line_nr == 1:
                        self.create_cam_frame(pose)
                        
                        if not self.file_has_child_frame:
                            self.sendTransform(pose, line_nr)
                    else:
                        self.sendTransform(pose, line_nr)

                else:
                    self.get_logger().error(f"File has wrong formatting on line: {line_nr}")

                line_nr += 1


def main():
    rclpy.init()
    node = static_tf_broadcast()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()