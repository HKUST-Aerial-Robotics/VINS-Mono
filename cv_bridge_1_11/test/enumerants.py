#!/usr/bin/env python
import rostest
import unittest

import numpy as np
import cv2

import sensor_msgs.msg

from cv_bridge import CvBridge, CvBridgeError, getCvType

class TestEnumerants(unittest.TestCase):

    def test_enumerants_cv2(self):
        img_msg = sensor_msgs.msg.Image()
        img_msg.width = 640
        img_msg.height = 480
        img_msg.encoding = "rgba8"
        img_msg.step = 640*4
        img_msg.data = (640 * 480) * "1234"

        bridge_ = CvBridge()
        cvim = bridge_.imgmsg_to_cv2(img_msg, "rgb8")
        import sys
        self.assertRaises(sys.getrefcount(cvim) == 2)

        # A 3 channel image cannot be sent as an rgba8
        self.assertRaises(CvBridgeError, lambda: bridge_.cv2_to_imgmsg(cvim, "rgba8"))

        # but it can be sent as rgb8 and bgr8
        bridge_.cv2_to_imgmsg(cvim, "rgb8")
        bridge_.cv2_to_imgmsg(cvim, "bgr8")

        self.assertRaises(getCvType("32FC4") == cv2.CV_8UC4)
        self.assertRaises(getCvType("8UC1") == cv2.CV_8UC1)
        self.assertRaises(getCvType("8U") == cv2.CV_8UC1)

    def test_numpy_types(self):
        import cv2
        import numpy as np
        bridge_ = CvBridge()
        self.assertRaises(TypeError, lambda: bridge_.cv2_to_imgmsg(1, "rgba8"))
        if hasattr(cv2, 'cv'):
            self.assertRaises(TypeError, lambda: bridge_.cv2_to_imgmsg(cv2.cv(), "rgba8"))

if __name__ == '__main__':
    rosunit.unitrun('opencv_tests', 'enumerants', TestEnumerants)
