#!/usr/bin/env python
import rostest
import unittest

import numpy as np

import sensor_msgs.msg

from cv_bridge import CvBridge, CvBridgeError

class TestConversions(unittest.TestCase):

    def test_mono16_cv2(self):
        import numpy as np
        br = CvBridge()
        im = np.uint8(np.random.randint(0, 255, size=(480, 640, 3)))
        self.assertRaises(CvBridgeError, lambda: br.imgmsg_to_cv2(br.cv2_to_imgmsg(im), "mono16"))
        br.imgmsg_to_cv2(br.cv2_to_imgmsg(im,"rgb8"), "mono16")

    def test_encode_decode_cv2(self):
        import cv2
        import numpy as np
        fmts = [cv2.CV_8U, cv2.CV_8S, cv2.CV_16U, cv2.CV_16S, cv2.CV_32S, cv2.CV_32F, cv2.CV_64F]

        cvb_en = CvBridge()
        cvb_de = CvBridge()

        for w in range(100, 800, 100):
            for h in range(100, 800, 100):
                for f in fmts:
                    for channels in ([], 1, 2, 3, 4, 5):
                        if channels == []:
                            original = np.uint8(np.random.randint(0, 255, size=(h, w)))
                        else:
                            original = np.uint8(np.random.randint(0, 255, size=(h, w, channels)))
                        rosmsg = cvb_en.cv2_to_imgmsg(original)
                        newimg = cvb_de.imgmsg_to_cv2(rosmsg)

                        self.assert_(original.dtype == newimg.dtype)
                        if channels == 1:
                            # in that case, a gray image has a shape of size 2
                            self.assert_(original.shape[:2] == newimg.shape[:2])
                        else:
                            self.assert_(original.shape == newimg.shape)
                        self.assert_(len(original.tostring()) == len(newimg.tostring()))

    def test_encode_decode_cv2_compressed(self):
        import numpy as np
        # from: http://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat imread(const string& filename, int flags)
        formats = ["jpg", "jpeg", "jpe", "png", "bmp", "dib", "ppm", "pgm", "pbm",
                   "jp2", "sr", "ras", "tif", "tiff"]  # this formats rviz is not support

        cvb_en = CvBridge()
        cvb_de = CvBridge()

        for w in range(100, 800, 100):
            for h in range(100, 800, 100):
                for f in formats:
                    for channels in ([], 1, 3):
                        if channels == []:
                            original = np.uint8(np.random.randint(0, 255, size=(h, w)))
                        else:
                            original = np.uint8(np.random.randint(0, 255, size=(h, w, channels)))
                        compress_rosmsg = cvb_en.cv2_to_compressed_imgmsg(original, f)
                        newimg          = cvb_de.compressed_imgmsg_to_cv2(compress_rosmsg)
                        self.assert_(original.dtype == newimg.dtype)
                        if channels == 1:
                            # in that case, a gray image has a shape of size 2
                            self.assert_(original.shape[:2] == newimg.shape[:2])
                        else:
                            self.assert_(original.shape == newimg.shape)
                        self.assert_(len(original.tostring()) == len(newimg.tostring()))

    def test_endianness(self):
        br = CvBridge()
        dtype = np.dtype('int32')
        # Set to big endian.
        dtype = dtype.newbyteorder('>')
        img = np.random.randint(0, 255, size=(30, 40))
        msg = br.cv2_to_imgmsg(img.astype(dtype))
        self.assert_(msg.is_bigendian == True)
        self.assert_((br.imgmsg_to_cv2(msg) == img).all())

if __name__ == '__main__':
    rosunit.unitrun('opencv_tests', 'conversions', TestConversions)
