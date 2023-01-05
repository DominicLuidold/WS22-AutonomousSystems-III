#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('token_detector')
import sys
import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

class image_viewer:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback)
    self.image_sub2 = rospy.Subscriber("/camera/rgb/image_raw", Image, self.simulated_callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
      cv2.imshow("Image window", cv_image)
      cv2.waitKey(3)
    except CvBridgeError as e:
      print(e)
  
  def simulated_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv2.imshow("Image window", cv_image)
      cv2.waitKey(3)
    except CvBridgeError as e:
      print(e)


def main(args):
  ic = image_viewer()
  rospy.init_node('image_viewer', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)