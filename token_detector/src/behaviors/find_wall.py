#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class FindWall:
  """ Move towards wall where turtlebot left it to capture a token """
  
  def __init__(self, killerrobot) -> None:
    rospy.Subscriber('scan', LaserScan, self.__process_scan)
    self._killerrobot = killerrobot

  def __process_scan(self, data: LaserScan) -> None:
    self._scan = data

  def isApplicable(self) -> bool:
    """ always """
    return True

  def execute(self) -> None:
    """ 
    move towards point in wall where follower was exited.
    first, turn left 
    """
    rospy.logdebug('behavior: find_wall')
    self._killerrobot.move(0.1, 0.1)
