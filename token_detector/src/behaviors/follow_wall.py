import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

max_speed = 0.13
min_detection_dist = 0.03
max_detection_dist = 0.3
MAX_TARGET_DISTANCE = 0.2


def filtered_min(ranges: list, min_value: float) -> float:
  """ returns minimum value in ranges that is not lower than min_value """
  values_greather_range_min = [i for i in ranges if i > min_value]
  if (len(values_greather_range_min) == 0):
      return math.inf
  return min(values_greather_range_min)

def isNearStart(x, y, distance):
    return abs(x) < distance and abs(y) < distance


class WallFollower:
  """ Follow wall on the left side """
  
  def __init__(self, killerrobot):
    rospy.Subscriber('scan', LaserScan, self.__process_scan)
    rospy.Subscriber('odom', Odometry, self.__process_odom)
    self.__killerrobot = killerrobot
    self._behaviors = [CompleteRoundtrip(), TurnTowardsWall(), FollowWall()]
    self._scan = None
    self._odom = None
    self._started = False

  def __process_scan(self, data: LaserScan):
    self._scan = data

  def __process_odom(self, data: Odometry):
    self._odom = data
    #check only as long as it is initially in the starting area
    if not self._started:
      self._started = not isNearStart(data.pose.pose.position.x, data.pose.pose.position.y, MAX_TARGET_DISTANCE + 0.05)
      #rospy.loginfo("Leaving start area, will return soon")


  def isApplicable(self):
    """ if wall is somewhere at front """
    return any([b.isApplicable(self._scan, self._odom, self._started) for b in self._behaviors])

  def execute(self):
    rospy.logdebug('behavior: follow wall')
    for b in self._behaviors:
      if b.isApplicable(self._scan, self._odom, self._started):
        b.execute(self._scan, self.publish_move)
        break

  def publish_move(self, linear, angular):
    self.__killerrobot.move(linear, angular)


class CompleteRoundtrip:

  def isApplicable(self, scan: LaserScan, odom: Odometry, started: bool):
    #check when odom value is entered and if it has left the starting area
    if odom is not None and started:
        #check if turtlebot has reached its initial starting position
        #rospy.loginfo("Check: x: " + str(turtle._odom.pose.pose.position.x) + " , y: " + str(turtle._odom.pose.pose.position.y))
        if isNearStart(odom.pose.pose.position.x, odom.pose.pose.position.y, MAX_TARGET_DISTANCE):
            return True
    return False

  def execute(self, scan: LaserScan, publish_move):
      rospy.loginfo("Finished")

class TurnTowardsWall:
  def isApplicable(self, scan: LaserScan, odom: Odometry, started: bool):
    """
    Applicable as soon as the wall bends away from the turtle (it moved past the corner)
    Stops and turns left.
    """
    if scan:
      ranges = scan.ranges
      range_min = scan.range_min
      # order of sensors: front, front_left, left, behind_left
      dist = [filtered_min(ranges[:15] + ranges[-15:], range_min), filtered_min(ranges[30:60], range_min), filtered_min(ranges[75:105], range_min), filtered_min(ranges[120:150], range_min)]
      if dist[0] > max_detection_dist and dist[1] > max_detection_dist and dist[2] <= max_detection_dist:
        return True
    return False

  def execute(self, scan: LaserScan, publish_move):
    ranges = scan.ranges
    range_min = scan.range_min
    dist = [filtered_min(ranges[:15] + ranges[-15:], range_min), filtered_min(ranges[30:60], range_min), filtered_min(ranges[75:105], range_min), filtered_min(ranges[120:150], range_min)]
    angular_k = 1 # the further away, the sharper the turn
    closeness_percent = (1 - (dist[2] - min_detection_dist) / (max_detection_dist - min_detection_dist))
    linear_velocity = max_speed * closeness_percent # the further away, the slower
    angular_velocity = closeness_percent
    rospy.logdebug('turn linear {} angular {} sensors {}'.format(linear_velocity, angular_velocity, dist))
    publish_move(linear_velocity, angular_velocity) # turn left towards lost wall


class FollowWall:
  """
  Behavior that follows a wall (keeps the wall on its left side)
  Takes into account the sensors in front, left front and on the left hand side
  and calculates the linear and angular velocity.
  """
  def isApplicable(self, scan: LaserScan, odom: Odometry, started: bool):
    if scan and any(item < max_detection_dist for item in scan.ranges):
      return True
    return False

  def execute(self, scan: LaserScan, publish_move):
    ranges = scan.ranges
    range_min = scan.range_min
    # order of sensors: front, front_left, left
    dist = [filtered_min(ranges[:15] + ranges[-15:], range_min), filtered_min(ranges[30:60], range_min), filtered_min(ranges[75:105], range_min)]
    linear_k = [1.5 * max_speed, 0, 0]
    angular_k = [-1, -0.3, 0.2]
    sensitivity_dist = [1.5 * max_detection_dist, max_detection_dist, max_detection_dist]
    linear_velocity = max_speed
    angular_velocity = 0
    for i in range(3):
      if dist[i] <= sensitivity_dist[i]:
        linear_velocity -= linear_k[i] * (1 - (dist[i] - min_detection_dist) / (sensitivity_dist[i] - min_detection_dist))
        angular_velocity += angular_k[i] * (1 - (dist[i] - min_detection_dist) / (sensitivity_dist[i] - min_detection_dist))
    rospy.logdebug('follow linear {} angular {} sensors {}'.format(linear_velocity, angular_velocity, dist))
    publish_move(linear_velocity, angular_velocity)
