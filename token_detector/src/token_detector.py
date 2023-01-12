#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('token_detector')
import rospy
import cv2 as cv
from geometry_msgs.msg import Twist
from raspicam_detector import RaspicamDetector
#from behaviors.capture_token import CaptureToken
#from behaviors.find_wall import FindWall
#from behaviors.follow_wall import FollowWall
#from behaviors.move_towards_token import MoveTowardsToken
#from behaviors.step_onto_token import StepOntoToken

class Turtle:

  def __init__(self):
    rospy.init_node('token_detector', anonymous=True)
    self.__pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #self.__behaviors = [CaptureToken(), StepOntoToken(), MoveTowardsToken(), FollowWall(), FindWall()]
    self.__tokens = [] # (x,y) coordinates of tokens
    self.__respicam_detector = RaspicamDetector()


  def keep_movin(self):
    pass
    """while not rospy.is_shutdown():
      if len(self.__respicam_detector.tokens):
        for b in self.__behaviors:
          if b.isApplicable(self.__respicam_detector.tokens):
            b.execute(self, self.__respicam_detector.tokens)
            break
      else:
        self.move(0,0)
      rospy.Rate(10).sleep()
      """
  

  def move(self, linear, angular):
    move = Twist()
    move.linear.x = linear
    move.angular.z = angular
    print(f'pub {move.linear.x} {move.angular.z}')
    self.__pub.publish(move)



if __name__ == '__main__':
  tb = Turtle()
  tb.keep_movin()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()