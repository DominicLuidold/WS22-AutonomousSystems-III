#!/usr/bin/env python
import math
import rospy
from helpers.helper import filtered_min
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

NUM_TOKENS = rospy.get_param('num_tokens')

# Config
MAX_SPEED = 0.13
MIN_DETECTION_DIST = 0.03
MAX_DETECTION_DIST = 0.3
MAX_TARGET_DISTANCE = 0.15

class WallFollower:
  """ Follow wall on the left side """
  
  def __init__(self, killerrobot) -> None:
    self._killerrobot = killerrobot
    self._behaviors = [TurnTowardsWall(killerrobot), FollowWall(killerrobot)]
    self._scan = None
    self._dist = None # distances to wall dictionary with keys front, front_left, left
    self._roundtrip_monitor = RoundtripMonitor(killerrobot)
    rospy.Subscriber('scan', LaserScan, self._process_scan)

  def _process_scan(self, data: LaserScan) -> None:
    self._scan = data

  def isApplicable(self) -> bool:
    """ if wall is somewhere at front - left """
    if self._scan:
      ranges = self._scan.ranges
      range_min = self._scan.range_min
      self._dist = {'front': filtered_min(ranges[:15] + ranges[-15:], range_min), \
                   'front_left': filtered_min(ranges[30:60], range_min), \
                   'left': filtered_min(ranges[75:105], range_min)}
      return any([b.isApplicable(self._dist) for b in self._behaviors])
    return False

  def execute(self) -> None:
    if not self._killerrobot.roundtrip_complete:
      self._roundtrip_monitor.check_roundtrip(self._killerrobot.pose)
    for b in self._behaviors:
      if b.isApplicable(self._dist):
        b.execute(self._dist)
        break

class RoundtripMonitor:

  def __init__(self, killerrobot) -> None:
    self._initial_pose = None # determine where killerrobot initially made contact with the wall to complete its first roundtrip
    self._ever_left_initial_area = False
    self._roundtrip_complete = False
    self._killerrobot = killerrobot

  def check_roundtrip(self, current_pose):
    if current_pose:
      if not self._initial_pose: # set first wall contact point once on first call
        self._initial_pose = current_pose
        rospy.logdebug(f'initial pose {current_pose.x} {current_pose.y}')
      if self._is_near_initial_pose(current_pose):
        if self._ever_left_initial_area:
          self._killerrobot.complete_initial_roundtrip()
      else:
        self._ever_left_initial_area = True # register when robot leaves the area of first wall contact point to find it later again

  def _is_near_initial_pose(self, current_pose) -> bool:
    eucl_dist = math.sqrt((current_pose.x - self._initial_pose.x)**2 \
       + (current_pose.y - self._initial_pose.y)**2)
    return eucl_dist < MAX_TARGET_DISTANCE


class TurnTowardsWall:

  def __init__(self, killerrobot) -> None:
    self._killerrobot = killerrobot

  def isApplicable(self, dist) -> bool:
    """
    Applicable as soon as the wall bends away from the turtle (it moved past the corner)
    Stops and turns left.
    """
    return dist['front'] > MAX_DETECTION_DIST and dist['front_left'] > MAX_DETECTION_DIST and dist['left'] <= MAX_DETECTION_DIST

  def execute(self, dist) -> None:
    rospy.logdebug('behavior: turn towards wall')
    closeness_percent = max((1 - (dist['left'] - MIN_DETECTION_DIST) / (MAX_DETECTION_DIST - MIN_DETECTION_DIST)), 0.1)
    linear_velocity = MAX_SPEED * closeness_percent # the further away, the slower
    angular_velocity = closeness_percent
    rospy.logdebug('turn linear {} angular {} closeness {} sensors {}'.format(linear_velocity, angular_velocity, closeness_percent, dist))
    self._killerrobot.move(linear_velocity, angular_velocity) # turn left towards lost wall


class FollowWall:
  """
  Behavior that follows a wall (keeps the wall on its left side)
  Takes into account the sensors in front, left front and on the left hand side
  and calculates the linear and angular velocity.
  """

  def __init__(self, killerrobot) -> None:
    self._killerrobot = killerrobot

  def isApplicable(self, dist) -> bool:
    return any(distance < MAX_DETECTION_DIST for distance in dist.values())

  def execute(self, distance) -> None:
    rospy.logdebug('behavior: follow wall')
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
    self._killerrobot.move(linear_velocity, angular_velocity)
