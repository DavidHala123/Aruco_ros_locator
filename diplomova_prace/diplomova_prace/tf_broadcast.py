import math
import sys
from geometry_msgs.msg import TransformStamped
import os
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from std_msgs.msg import String



def quaternion_from_euler(ai, aj, ak):
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


class tfFramePublisher(Node):
    def __init__(self, transformation):
        super().__init__('tfPublisher')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.i = 0
        for arr in transformation:
            print(arr)
            self.make_transforms(arr)

    def make_transforms(self, transformation):
        try:
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = transformation[0]
            t.child_frame_id = transformation[1]

            t.transform.translation.x = float(transformation[2])
            t.transform.translation.y = float(transformation[3])
            t.transform.translation.z = float(transformation[4])
            quat = quaternion_from_euler(
                float(transformation[5]), float(transformation[6]), float(transformation[7]))
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            self.tf_static_broadcaster.sendTransform(t)
            return True
        
        except Exception as e:
            return False




#def find_index(array_of_strings, substring):
#    for index, string in enumerate(array_of_strings):
#        if substring in string:
#            return index  # Return the index of the first string with the substring
#    return -1  # Return -1 if no string contains the substring



def main():
    i = 0

    file_path = os.path.join(os.path.join(os.path.dirname(os.path.realpath(__file__)), "_data", "setup.txt"))
    with open(file_path, "r") as file_handler:
        raw_data = file_handler.readlines()
    
    sArgs = [['0']* 8]*len(raw_data)
    for arr in raw_data:
        sArgs[i] = arr.split()
        if i != 0:
            sArgs[i].insert(0,'map')
        i += 1
    sArgs[0].insert(1,'body')
    logger = rclpy.logging.get_logger('logger')

    if len(sArgs[0]) != 8 or len(sArgs[1]) != 8:
        logger.info('Invalid number of parameters.')
        sys.exit(1)

    # pass parameters and initialize node
    rclpy.init()
    node = tfFramePublisher(sArgs)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()