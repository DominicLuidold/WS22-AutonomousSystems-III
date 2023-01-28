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
NUM_TOKENS = rospy.get_param('num_tokens')

class TokenDetector:
  def __init__(self):
    self._movement_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    if IGNORE_TOKEN:
      self._behaviors = [WallFollower(self), FindWall(self)]
    else:
      self._behaviors = [StepOntoToken(self), CaptureToken(self), MoveTowardsToken(self), WallFollower(self), FindWall(self)]
    self.tokens = [] # (id,x,y,angle) info of tokens
    self.max_speed = 0.22
    self.isovertoken = False

  def keep_movin(self):
    while not rospy.is_shutdown() and NUM_TOKENS > len(self.tokens):
      for b in self._behaviors:
        if b.isApplicable():
          b.execute()
          break
      rospy.Rate(50).sleep()

  def move(self, linear: float, angular: float) -> None:
    move = Twist()
    move.linear.x = linear
    move.angular.z = angular
    rospy.logdebug(f'pub {move.linear.x} {move.angular.z}')
    self._movement_publisher.publish(move)

def main() -> None:
  log_level = rospy.DEBUG if DEBUG else rospy.INFO
  rospy.init_node('token_detector', log_level=log_level)

  td = TokenDetector()
  try:
    td.keep_movin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main()
