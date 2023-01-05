#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('token_detector')
import sys
import rospy
import cv2 as cv
import numpy as np
import math
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class Turtle:

  def __init__(self):
    rospy.init_node('token_detector', anonymous=True)
    self.__pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    self.__behaviors = [TargetReached(), MoveTowardsToken()]
    self.__token_detector = TokenDetector()


  def keep_movin(self):
    while not rospy.is_shutdown():
      if len(self.__token_detector.tokens):
        for b in self.__behaviors:
          if b.isApplicable(self.__token_detector.tokens):
            b.execute(self, self.__token_detector.tokens)
            break
      else:
        self.move(0,0)
      rospy.Rate(10).sleep()
  

  def move(self, linear, angular):
    move = Twist()
    move.linear.x = linear
    move.angular.z = angular
    print(f'pub {move.linear.x} {move.angular.z}')
    self.__pub.publish(move)



class TokenDetector:

  def __init__(self):
    self.__bridge = CvBridge()
    self.__image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.detect_token)
    self.tokens = [] # [x,y,y-distance[%],angle[rad]]


  def detect_token(self,data):
    try:
      cv_image_bgr = self.__bridge.compressed_imgmsg_to_cv2(data, "bgr8")
      self.raspicam_base = np.array([cv_image_bgr.shape[1]//2, cv_image_bgr.shape[0]])
      mask = self.get_token_mask(cv_image_bgr)
      mask = self.remove_noise(mask)
      contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
      if contours:
        self.tokens = self.calculate_all_token_centers(contours)
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


  def calculate_all_token_centers(self, contours):
    """
    TODO improve accuracy by either just using left/rightmost bzw hightst/lowest points middle of contour or approximating down to 4 points like in ba2
    """
    tokens = np.empty((0, 4), float)
    for contour in contours:
      center = self.calculate_token_center(cv.convexHull(contour))
      tokens = np.append(tokens, np.array([center]), axis=0)
    tokens = self.sort_tokens_by_distance(tokens)
    tokens = self.set_angle_of_tokens(tokens)
    return tokens


  def calculate_token_center(self, hull):
    min_x, min_y, max_x, max_y = 10_000, 10_000, 0, 0
    for point in hull:
      min_x = min(min_x, point[0][0])
      min_y = min(min_y, point[0][1])
      max_x = max(max_x, point[0][0])
      max_y = max(max_y, point[0][1])
    return np.array([min_x + (max_x - min_x)//2, min_y + (max_y - min_y)//2, 0, 0])


  def sort_tokens_by_distance(self, tokens):
    """
    sort only by difference on y-axis as image is distorted
    """
    distances = [self.raspicam_base[1] - token[1] for token in tokens]
    for i in range(len(tokens)):
      tokens[i][2] = distances[i] / self.raspicam_base[1]
    indexes = np.argsort(distances)
    return tokens[indexes]


  def set_angle_of_tokens(self, tokens):
    """
    Calculate the angle in radians along the y-axis! -> Directly in front = 0, slightly to the right is -0.x, left = +0.x
    """
    for token in tokens:
      token[3] = math.atan2(self.raspicam_base[0] - token[0], self.raspicam_base[1] - token[1]) # atan2(phi) = gk/ak = x-x/y-y
    return tokens


  def print_contours(self, mask, contours):
    if contours:
      img = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
      hulls = []
      for contour in contours:
        hull = cv.convexHull(contour)
        center = self.calculate_token_center(hull)
        cv.circle(img, (center[0], center[1]), 10, (0,0,255), -1)
        hulls.append(hull)
      img = cv.drawContours(img, hulls, -1, (0,255,0), 3)
      cv.imshow('m', img)
      cv.waitKey(0)



class TargetReached:
  """
  Time to celebrate!
  """
  def isApplicable(self, tokens):
    """
    if scan and scan.ranges[0] < target_reached_dist:
      return True
    return False
    """
    return False

  def execute(self, token_detector):
    print('Target reached!!!')
    #token_detector.publish_move(0,0)


class MoveTowardsToken:
  """
  Target is ahead and in sight
  """
  def isApplicable(self, tokens):
    return True

  def execute(self, killerrobot, tokens):
    clst_tkn = tokens[0]
    linear_velocity = 0.26 * clst_tkn[2] / 2
    angular_velocity = clst_tkn[3] * 0.3
    killerrobot.move(linear_velocity, angular_velocity)
    """
    closest_dist = min(turtle.scan.ranges)
    closest_index = turtle.scan.ranges.index(closest_dist)
    distance_percent = min((closest_dist - min_detection_dist) / (0.5 - min_detection_dist), 1)
    linear_velocity = max_speed * distance_percent
    angular_velocity = 0
    if closest_index < 180: # target is on left side -> turn left
      angular_velocity = closest_index / 20 * 0.5
    else:
      angular_velocity = (360 - closest_index) / 20 * -0.5
    print('towards linear {} angular {} min distance {} closest index {}'.format(linear_velocity, angular_velocity, closest_dist, closest_index))
    turtle.publish_move(linear_velocity, angular_velocity)
    """
    pass


def main(args):
  tb = Turtle()
  tb.keep_movin()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)