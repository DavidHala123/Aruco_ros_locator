import rclpy
from rclpy.node import Node
import os
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
import math
import numpy as np
import time

class static_tf_broadcast(Node):

    def __init__(self):
        super().__init__('static_tf_broadcast')
        self.declare_parameter('resend_when_subs_changed', True)
        self.declare_parameter('setup_file_path', "")
        self.declare_parameter("ref_frame_name", "map")
        
        self.resend_when_subs_changed = self.get_parameter('resend_when_subs_changed').value
        self.path = self.get_parameter('setup_file_path').value
        self.ref_frame = self.get_parameter('ref_frame_name').value

        self.tArray = []
        self.tf_broadcast = StaticTransformBroadcaster(self)
        self.get_logger().info(f"Populating tf tree:")

        self.populate_tree()

        if self.resend_when_subs_changed:
            self.get_logger().info("Transforms will be resent if subscriber count increases")
            self.old_subsCt = self.tf_broadcast.pub_tf.get_subscription_count()
            self.timer = self.create_timer(2, self.resend_tf)





    def resend_tf(self):
        subs = self.tf_broadcast.pub_tf.get_subscription_count()
        
        if subs > self.old_subsCt:
            for t in self.tArray:
                self.tf_broadcast.sendTransform(t)
                self.get_logger().info(f"Resent transform of frame '{t.child_frame_id}' connected to '{t.header.frame_id}' frame")
                time.sleep(0.2)

            self.old_subsCt = subs

        elif subs < self.old_subsCt:
            self.old_subsCt = subs

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

        if self.resend_when_subs_changed:
            self.tArray.append(t)
        
        self.get_logger().info(f"Created transform of frame '{t.child_frame_id}' connected to '{t.header.frame_id}' frame")


        

    def populate_tree(self):
        with open(self.path, 'r') as file:
            line_nr = 1
            for line in file:
                pose = line.split()

                if len(pose) == 7:
                    self.sendTransform(pose, line_nr)
                    time.sleep(0.2)

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