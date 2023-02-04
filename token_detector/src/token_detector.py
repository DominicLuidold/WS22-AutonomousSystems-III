#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('token_detector')
import rospy
import os
import cv2 as cv
from geometry_msgs.msg import Twist
from behaviors.capture_token import CaptureToken
from behaviors.find_wall import FindWall
from behaviors.follow_wall import WallFollower
from behaviors.move_towards_token import MoveTowardsToken
from behaviors.step_onto_token import StepOntoToken
from current_pos.msg import PoseTF

# Launch arguments
DEBUG = rospy.get_param('debug')
SKIP_ROUNDTRIP = rospy.get_param('skip_roundtrip')
NUM_TOKENS = rospy.get_param('num_tokens')

class TokenDetector:
  def __init__(self):
    self.max_speed = 0.22
    self.isovertoken = False
    self.tokens = [] # (id,x,y,angle) info of tokens
    self.pose = None
    self.roundtrip_complete = False
    self._wall_follower = WallFollower(self)
    self._wall_finder = FindWall(self)
    self._behaviors = [self._wall_follower, self._wall_finder]
    if SKIP_ROUNDTRIP:
      self.complete_initial_roundtrip()
    self._movement_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('pose_tf', PoseTF, self._process_pose)
    rospy.on_shutdown(self.save_map)

  def _process_pose(self, pose: PoseTF):
    self.pose = pose.mapPose

  def keep_movin(self):
    while not rospy.is_shutdown() and (NUM_TOKENS > len(self.tokens) or not self.roundtrip_complete):
      rospy.logdebug(f'registered tokens: {self.tokens}')
      for b in self._behaviors:
        if b.isApplicable():
          b.execute()
          break
      rospy.Rate(10).sleep()

  def move(self, linear: float, angular: float) -> None:
    move = Twist()
    move.linear.x = linear
    move.angular.z = angular
    rospy.logdebug(f'pub {move.linear.x} {move.angular.z}')
    self._movement_publisher.publish(move)

  def register_token(self, tag_id, x, y, angle):
    self.tokens.append((tag_id, x, y, angle))
    self.save_map()

  def complete_initial_roundtrip(self):
    self.save_map()
    self.roundtrip_complete = True
    self._behaviors = [StepOntoToken(self), CaptureToken(self), MoveTowardsToken(self), self._wall_follower, self._wall_finder]
    rospy.loginfo('Initial roundtrip complete! Switched to different behaviors')
  
  def save_map(self):
    rospy.loginfo("Saving map!")
    os.system("rosrun map_server map_saver -f /killerrobot/saved-map")


def main() -> None:
  log_level = rospy.DEBUG if DEBUG else rospy.INFO
  rospy.init_node('token_detector', log_level=log_level)
  td = TokenDetector()
  try:
    td.keep_movin()
  except KeyboardInterrupt:
    print("Keyboard Interrupt")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main()
