
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class FindWall:
  """ Move towards wall where turtlebot left it to capture a token """
  
  def __init__(self, killerrobot) -> None:
    self.__killerrobot = killerrobot
    rospy.Subscriber('scan', LaserScan, self.__process_scan)
    rospy.Subscriber('odom', Odometry, self.__process_odom)

  def __process_scan(self, data: LaserScan):
    self._scan = data

  def __process_odom(self, data: Odometry):
    pass

  def isApplicable(self):
    """ always """
    return True

  def execute(self):
    """ 
    move towards point in wall where follower was exited.
    first, turn left 
    """
    rospy.loginfo('behavior: find_wall')
    pass