#!/usr/bin/env python
import os
import rospy
from helpers.helper import filtered_min
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Config
MAX_SPEED = 0.13
MIN_DETECTION_DIST = 0.03
MAX_DETECTION_DIST = 0.3
MAX_TARGET_DISTANCE = 0.2

def isNearStart(x: float, y: float, distance: float) -> bool:
    return abs(x) < distance and abs(y) < distance


class WallFollower:
  """ Follow wall on the left side """
  
  def __init__(self, killerrobot):
    # Init
    rospy.loginfo("WallFollower behavior initialized!")

    self.__killerrobot = killerrobot
    self._behaviors = [CompleteRoundtrip(), TurnTowardsWall(), FollowWall()]
    self._scan = None
    self._odom = None
    self._started = False
    self._map_saved = False

    # Subscribed topics
    rospy.Subscriber('scan', LaserScan, self.__process_scan)
    rospy.Subscriber('odom', Odometry, self.__process_odom)

  def __process_scan(self, data: LaserScan) -> None:
    self._scan = data

  def __process_odom(self, data: Odometry) -> None:
    self._odom = data

    # Check only as long as it is initially in the starting area
    if not self._started:
      self._started = not isNearStart(data.pose.pose.position.x, data.pose.pose.position.y, MAX_TARGET_DISTANCE + 0.05)
      rospy.loginfo("Leaving start area...")

  def isApplicable(self) -> bool:
    """ if wall is somewhere at front """
    return any([b.isApplicable(self._scan, self._odom, self._started) for b in self._behaviors])

  def execute(self) -> None:
    rospy.logdebug('behavior: follow wall')
    for b in self._behaviors:
      if b.isApplicable(self._scan, self._odom, self._started):
        b.execute(self._scan, self.publish_move, self.mark_map_as_saved, self._map_saved)
        break

  def publish_move(self, linear, angular) -> None:
    self.__killerrobot.move(linear, angular)

  def mark_map_as_saved(self):
    self._map_saved = True


class CompleteRoundtrip:
  def isApplicable(self, scan: LaserScan, odom: Odometry, started: bool) -> None:
    # Check when odom value is entered and if it has left the starting area
    if odom is not None and started:
        rospy.logdebug("Check: x: " + str(odom.pose.pose.position.x) + " , y: " + str(odom.pose.pose.position.y))

        #check if turtlebot has reached its initial starting position
        if isNearStart(odom.pose.pose.position.x, odom.pose.pose.position.y, MAX_TARGET_DISTANCE):
            return True

    return False

  def execute(self, scan: LaserScan, publish_move, mark_map_as_saved, map_saved) -> None:
      if not map_saved:
        rospy.loginfo("Saving map!")
        os.system("rosrun map_server map_saver -f ~/catkin_ws/map")
        mark_map_as_saved()
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
      if dist[0] > MAX_DETECTION_DIST and dist[1] > MAX_DETECTION_DIST and dist[2] <= MAX_DETECTION_DIST:
        return True
    return False

  def execute(self, scan: LaserScan, publish_move, mark_map_as_saved, map_saved):
    ranges = scan.ranges
    range_min = scan.range_min
    dist = [filtered_min(ranges[:15] + ranges[-15:], range_min), filtered_min(ranges[30:60], range_min), filtered_min(ranges[75:105], range_min), filtered_min(ranges[120:150], range_min)]
    angular_k = 1 # the further away, the sharper the turn
    closeness_percent = (1 - (dist[2] - MIN_DETECTION_DIST) / (MAX_DETECTION_DIST - MIN_DETECTION_DIST))
    linear_velocity = MAX_SPEED * closeness_percent # the further away, the slower
    angular_velocity = closeness_percent
    rospy.logdebug('turn linear {} angular {} sensors {}'.format(linear_velocity, angular_velocity, dist))
    publish_move(linear_velocity, angular_velocity) # turn left towards lost wall


class FollowWall:
  """
  Behavior that follows a wall (keeps the wall on its left side)
  Takes into account the sensors in front, left front and on the left hand side
  and calculates the linear and angular velocity.
  """
  def isApplicable(self, scan: LaserScan, odom: Odometry, started: bool) -> bool:
    if scan and any(item < MAX_DETECTION_DIST for item in scan.ranges):
      return True
    return False

  def execute(self, scan: LaserScan, publish_move, mark_map_as_saved, map_saved) -> None:
    ranges = scan.ranges
    range_min = scan.range_min
    # order of sensors: front, front_left, left
    dist = [filtered_min(ranges[:15] + ranges[-15:], range_min), filtered_min(ranges[30:60], range_min), filtered_min(ranges[75:105], range_min)]
    linear_k = [1.5 * MAX_SPEED, 0, 0]
    angular_k = [-1, -0.3, 0.2]
    sensitivity_dist = [1.5 * MAX_DETECTION_DIST, MAX_DETECTION_DIST, MAX_DETECTION_DIST]
    linear_velocity = MAX_SPEED
    angular_velocity = 0
    for i in range(3):
      if dist[i] <= sensitivity_dist[i]:
        linear_velocity -= linear_k[i] * (1 - (dist[i] - MIN_DETECTION_DIST) / (sensitivity_dist[i] - MIN_DETECTION_DIST))
        angular_velocity += angular_k[i] * (1 - (dist[i] - MIN_DETECTION_DIST) / (sensitivity_dist[i] - MIN_DETECTION_DIST))
    rospy.logdebug('follow linear {} angular {} sensors {}'.format(linear_velocity, angular_velocity, dist))
    publish_move(linear_velocity, angular_velocity)
