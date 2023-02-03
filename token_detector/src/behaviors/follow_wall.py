#!/usr/bin/env python
import os
import rospy
from helpers.helper import filtered_min
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

NUM_TOKENS = rospy.get_param('num_tokens')

# Config
MAX_SPEED = 0.13
MIN_DETECTION_DIST = 0.03
MAX_DETECTION_DIST = 0.3
MAX_TARGET_DISTANCE = 0.2

def isNearStart(x: float, y: float, distance: float) -> bool:
    return abs(x) < distance and abs(y) < distance


class WallFollower:
  """ Follow wall on the left side """
  
  def __init__(self, killerrobot) -> None:
    self._killerrobot = killerrobot
    self._behaviors = [CompleteRoundtrip(), TurnTowardsWall(), FollowWall()]
    self._scan = None
    self._odom = None
    self._started = False
    self.dist = None
    rospy.Subscriber('scan', LaserScan, self.__process_scan)
    rospy.Subscriber('odom', Odometry, self.__process_odom)

  def __process_scan(self, data: LaserScan) -> None:
    self._scan = data

  def __process_odom(self, data: Odometry) -> None:
    self._odom = data
    if not self._started: # Check only as long as it is initially in the starting area
      self._started = not isNearStart(data.pose.pose.position.x, data.pose.pose.position.y, MAX_TARGET_DISTANCE + 0.05)

  def isApplicable(self) -> bool:
    """ if wall is somewhere at front - left """
    if self._scan:
      ranges = self._scan.ranges
      range_min = self._scan.range_min
      self.dist = {'front': filtered_min(ranges[:15] + ranges[-15:], range_min), \
                   'front_left': filtered_min(ranges[30:60], range_min), \
                   'left': filtered_min(ranges[75:105], range_min)}
      return any([b.isApplicable(self.dist, self._odom, self._started, self._killerrobot) for b in self._behaviors])
    return False

  def execute(self) -> None:
    rospy.logwarn('behavior: follow wall')
    for b in self._behaviors:
      if b.isApplicable(self.dist, self._odom, self._started, self._killerrobot):
        b.execute(self.dist, self.publish_move, self._killerrobot)
        break

  def publish_move(self, linear: float, angular: float) -> None:
    self._killerrobot.move(linear, angular)


class CompleteRoundtrip:
  def isApplicable(self, distances, odom: Odometry, started: bool, killerrobot) -> None:
    # Check when odom value is entered and if it has left the starting area
    if odom is not None and started:
        rospy.logdebug("Check: x: " + str(odom.pose.pose.position.x) + " , y: " + str(odom.pose.pose.position.y))

        #check if turtlebot has reached its initial starting position
        if isNearStart(odom.pose.pose.position.x, odom.pose.pose.position.y, MAX_TARGET_DISTANCE):
          if not killerrobot.map_saved:
            return True
          else:
            if NUM_TOKENS <= len(killerrobot.tokens):
              return True

    return False

  def execute(self, distances, publish_move, killerrobot) -> None:
      if not killerrobot.map_saved:
        rospy.loginfo("Saving map!")
        os.system("rosrun map_server map_saver -f /killerrobot/saved-map")
        killerrobot.map_saved = True
      rospy.loginfo("Finished")

class TurnTowardsWall:
  def isApplicable(self, dist, odom: Odometry, started: bool, killerrobot) -> bool:
    """
    Applicable as soon as the wall bends away from the turtle (it moved past the corner)
    Stops and turns left.
    """
    return dist['front'] > MAX_DETECTION_DIST and dist['front_left'] > MAX_DETECTION_DIST and dist['left'] <= MAX_DETECTION_DIST

  def execute(self, dist, publish_move, killerrobot) -> None:
    angular_k = 1 # the further away, the sharper the turn
    closeness_percent = (1 - (dist['left'] - MIN_DETECTION_DIST) / (MAX_DETECTION_DIST - MIN_DETECTION_DIST))
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
  def isApplicable(self, dist, odom: Odometry, started: bool, killerrobot) -> bool:
    return any(distance < MAX_DETECTION_DIST for distance in dist.values())

  def execute(self, distance, publish_move, killerrobot) -> None:
    linear_k = [1.5 * MAX_SPEED, 0, 0]
    angular_k = [-1, -0.4, 0.2]
    sensitivity_dist = [1.5 * MAX_DETECTION_DIST, MAX_DETECTION_DIST, MAX_DETECTION_DIST]
    linear_velocity = MAX_SPEED
    angular_velocity = 0
    dist = [distance['front'], distance['front_left'], distance['left']]
    for i in range(3):
      if dist[i] <= sensitivity_dist[i]:
        linear_velocity -= linear_k[i] * (1 - (dist[i] - MIN_DETECTION_DIST) / (sensitivity_dist[i] - MIN_DETECTION_DIST))
        angular_velocity += angular_k[i] * (1 - (dist[i] - MIN_DETECTION_DIST) / (sensitivity_dist[i] - MIN_DETECTION_DIST))
    rospy.logdebug('follow linear {} angular {} sensors {}'.format(linear_velocity, angular_velocity, dist))
    publish_move(linear_velocity, angular_velocity)
