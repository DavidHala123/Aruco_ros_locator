import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrameListener(Node):

    def __init__(self):
        super().__init__('tf_subscriber')
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("child_frame", "body")
        self.ref_frame = self.get_parameter("reference_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)


    def on_timer(self):
        from_frame_rel = self.child_frame
        to_frame_rel = self.ref_frame
        print(from_frame_rel)
        print(to_frame_rel)
        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
            self.get_logger().info(f"Translation: {t.transform.translation}\nRotation: {t.transform.rotation}")
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {from_frame_rel} to {to_frame_rel}: {ex}')



def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()