import math
import sys
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import os
import numpy as np
import rclpy
from rclpy import publisher
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from yaml import load


class accuracyMeas(Node):

    def __init__(self):
        super().__init__('Accuracy_measurements')
        self.startDistance = 0.75
        self.distanceDifference = 0.25
        self.maxDistance = 2.00
        self.nrOfImages = 50
        self.img_path = "/home/david/ros2_ws/Presnost_dataset/DATASET/HD_WEBCAM"
        self.ref_frame = "Id0"
        self.child_frame = "body"

        self.tf_buffer = Buffer()
        self.pub = self.create_publisher(Image, '/image_raw2', 10)
        self.subs = TransformListener(self.tf_buffer, self)
        self.distance = self.startDistance
        self.i = 0
        self.imageNr = 1
        self.allowDataRecieved = False
        self.timer = self.create_timer(1, self.publishImage)


    def publishImage(self):

        if(self.i > self.nrOfImages):
            self.i = self.i + 1

        self.distance = self.distance + self.i * self.distanceDifference

        if(self.distance <= self.maxDistance):
            path = self.img_path + "/" "{:.2f}".format(self.distance) + "m" + f'/{self.imageNr}.png'
            if os.path.isfile(path):
                img = cv2.imread(path)
                bridge = CvBridge()
                img_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
                self.pub.publish(img_msg)
            else:
                self.get_logger().info(f'Could not transform find a file: {path}')

            self.getTransform()



    def getTransform(self):

        if(self.allowDataRecieved):
            from_frame_rel = self.child_frame
            to_frame_rel = self.ref_frame
            try:
                t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
                self.get_logger().info(f"Translation for {self.distance},{self.imageNr}: {t.transform.translation}\nRotation: {t.transform.rotation}")
                self.allowDataRecieved = False
                self.imageNr = self.imageNr + 1

            except TransformException as ex:
                self.get_logger().info(f'Could not transform {from_frame_rel} to {to_frame_rel}: {ex}')


def main():
    rclpy.init()
    node = accuracyMeas()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()