#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image

ignore_tokens_above_pixels_y = 55
class RaspicamDetector:

  def __init__(self):
    rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.detect_token)
    rospy.Subscriber("/camera/rgb/image_raw", Image, self.detect_simulated_token)
    self.tokens = [] # [x,y,y-distance[%],angle[rad]]
    self._bridge = CvBridge()


  def detect_token(self, data):
    """ detect tokens from raspicam image """
    try:
      cv_image_bgr = self._bridge.compressed_imgmsg_to_cv2(data, "bgr8")
      self.raspicam_base = np.array([cv_image_bgr.shape[1]//2, cv_image_bgr.shape[0]])
      mask = self.get_token_mask(cv_image_bgr)
      mask = self.remove_noise(mask)
      contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
      if contours:
        self.tokens = self.calculate_all_token_centers(contours)
      else:
        self.tokens = []
      self.print_contours(mask, contours, cv_image_bgr)
    except CvBridgeError as e:
      print(e)

  def detect_simulated_token(self, data):
    """ detect tokens from simulated world in gazebo """
    try:
      cv_image_bgr = self._bridge.imgmsg_to_cv2(data, "bgr8")
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
    """ Build a mask that shows where tokens are (white=token) """
    hsv = cv.cvtColor(bgr_image, cv.COLOR_BGR2HSV)
    token_color_lower_1 = np.array([0, 50, 100]) #red ranges from 345 to 15 halved by opencv -> 0-15 && 165-180
    token_color_upper_1 = np.array([15, 255, 255])
    token_color_lower_2 = np.array([155, 50, 100])
    token_color_upper_2 = np.array([180, 255, 255])
    mask1 = cv.inRange(hsv, token_color_lower_1, token_color_upper_1)
    mask2 = cv.inRange(hsv, token_color_lower_2, token_color_upper_2)
    mask = np.zeros(mask1.shape)
    mask = cv.bitwise_or(mask1, mask2, mask)
    return mask


  def remove_noise(self, mask):
    kernel = cv.getStructuringElement(cv.MORPH_RECT,(2,2))
    opened = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    closed = cv.morphologyEx(opened, cv.MORPH_CLOSE, kernel)
    return closed


  def calculate_all_token_centers(self, contours):
    """ Calculate center, sort by estimated distance and calculate angle of tokens """
    tokens = np.empty((0, 4), float)
    for contour in contours:
      center = self.calculate_token_center(cv.convexHull(contour))
      if center[1] < ignore_tokens_above_pixels_y:
        continue
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
    """ sort only by difference on y-axis as image is distorted """
    distances = [self.raspicam_base[1] - token[1] for token in tokens]
    for i in range(len(tokens)):
      tokens[i][2] = distances[i] / self.raspicam_base[1]
    indexes = np.argsort(distances)
    return tokens[indexes]


  def set_angle_of_tokens(self, tokens):
    """ Calculate the angle in radians along the y-axis! -> Directly in front = 0, slightly to the right is -0.x, left = +0.x """
    for token in tokens:
      token[3] = math.atan2(self.raspicam_base[0] - token[0], self.raspicam_base[1] - token[1]) # atan2(phi) = gk/ak = x-x/y-y
    return tokens


  def print_contours(self, mask, contours, image):
    """ helper to show mask with detected tokens and their centers """
    if contours:
      img = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
      hulls = []
      for contour in contours:
        hull = cv.convexHull(contour)
        center = self.calculate_token_center(hull)
        cv.circle(img, (center[0], center[1]), 10, (0,0,255), -1)
        hulls.append(hull)
      image = cv.drawContours(image, hulls, -1, (0,255,0), 3)
    cv.imshow('m', image)
    cv.waitKey(3)