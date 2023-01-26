#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('token_detector')
import rospy
import cv2 as cv
from geometry_msgs.msg import Twist
from behaviors.capture_token import CaptureToken
from behaviors.find_wall import FindWall
from behaviors.follow_wall import WallFollower
from behaviors.move_towards_token import MoveTowardsToken
from behaviors.step_onto_token import StepOntoToken

# Launch arguments
DEBUG = rospy.get_param('debug')
IGNORE_TOKEN = rospy.get_param('ignore_token')

class TokenDetector:

  def __init__(self):
    self.__pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    if IGNORE_TOKEN:
      self.__behaviors = [WallFollower(self), FindWall(self)]
    else:
      self.__behaviors = [StepOntoToken(self), CaptureToken(self), MoveTowardsToken(self), WallFollower(self), FindWall(self)]
    self.__tokens = [] # (x,y) coordinates of tokens
    self.max_speed = 0.22
    self._isovertoken = False

  def keep_movin(self):
    while not rospy.is_shutdown():
      for b in self.__behaviors:
        if b.isApplicable():
          b.execute()
          break
      rospy.Rate(50).sleep()

  def move(self, linear, angular):
    move = Twist()
    move.linear.x = linear
    move.angular.z = angular
    rospy.logdebug(f'pub {move.linear.x} {move.angular.z}')
    self.__pub.publish(move)

def main() -> None:
  # Init
  log_level = rospy.DEBUG if DEBUG else rospy.INFO
  rospy.init_node('token_detector', log_level=log_level)

  td = TokenDetector()
  td.keep_movin()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main()
