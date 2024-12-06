#!/usr/bin/env python3

import sys
import os
import ctypes
import numpy as np
from matplotlib import pyplot as plt

# Import ros stuff
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# OpenCV
import cv2

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError


class VxsSensorSubscriber(Node):

    def __init__(self):
        super().__init__("vxs_py_subscriber")
        camera_info_CB_group = MutuallyExclusiveCallbackGroup()
        self.calibration_sub = self.create_subscription(
            CameraInfo,
            "/depth/camera_info",
            self.CameraInfoCB,
            qos_profile=1,
            callback_group=camera_info_CB_group,
        )
        self.bridge = CvBridge()
        depth_image_CB_group = MutuallyExclusiveCallbackGroup()
        self.image_sub = self.create_subscription(
            Image,
            "/depth/image",
            self.DepthCB,
            qos_profile=1,
            callback_group=depth_image_CB_group,
        )
        self.K = None

    def DepthCB(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        (rows, cols, channels) = cv_image.shape
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    def CameraInfoCB(self, data):
        if self.k is not None:
            return
        self.K = np.array(data.k, dtype=np.float32).reshape(3, 3)
        self.P = np.array(data.p, dtype=np.float(32)).reshape(3, 4)
        self.R = np.array(data.r, dtype=np.float(32)).reshape(3, 3)
        self.d = np.array(data.d, dtype=np.float(32))
        print("Calibration acquired!")


def main(args):
    rclpy.init(args=args)
    ic = VxsSensorSubscriber()

    try:
        rclpy.spin(ic)
    except KeyboardInterrupt:
        print("Shutting down... exiting")
    ic.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
