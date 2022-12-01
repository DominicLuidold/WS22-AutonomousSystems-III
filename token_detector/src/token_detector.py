#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('token_detector')
import sys
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class token_detector:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.detect_token)
    self.tokens = []

  def detect_token(self,data):
    try:
      cv_image_bgr = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
      mask = self.get_token_mask(cv_image_bgr)
      mask = self.remove_noise(mask)
      contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
      if contours:
        self.tokens = self.calculate_centers(contours)
        #self.print_contours(mask, contours)
      else:
        self.tokens = []
      
    except CvBridgeError as e:
      print(e)
  
  def get_token_mask(self, bgr_image):
    """
    Build a mask that shows where tokens are (white=token)
    """
    hsv = cv.cvtColor(bgr_image, cv.COLOR_BGR2HSV)
    token_color_lower_1 = np.array([0, 50, 100]) #red ranges from 345 to 15 halved by opencv -> 0-15 && 165-180
    token_color_upper_1 = np.array([15, 255, 255])
    token_color_lower_2 = np.array([165, 50, 100])
    token_color_upper_2 = np.array([180, 255, 255])
    mask1 = cv.inRange(hsv, token_color_lower_1, token_color_upper_1)
    mask2 = cv.inRange(hsv, token_color_lower_2, token_color_upper_2)
    mask = np.zeros(mask1.shape)
    mask = cv.bitwise_or(mask1, mask2, mask)
    return mask

  def remove_noise(self, mask):
    kernel = cv.getStructuringElement(cv.MORPH_RECT,(5,5))
    opened = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    closed = cv.morphologyEx(opened, cv.MORPH_CLOSE, kernel)
    return closed

  def calculate_centers(self, contours):
    return [cv.convexHull(contour).mean(axis=0, dtype=np.int32)[0] for contour in contours]

  def print_contours(self, mask, contours):
    if contours:
      img = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
      hulls = []
      for contour in contours:
        hull = cv.convexHull(contour)
        center = hull.mean(axis=0, dtype=np.int32)[0]
        cv.circle(img, center, 10, (0,0,255), -1)
        hulls.append(hull)
      img = cv.drawContours(img, hulls, -1, (0,255,0), 3)
      cv.imshow('m', img)
      cv.waitKey(0)



def main(args):
  token_detector()
  rospy.init_node('token_detector', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)