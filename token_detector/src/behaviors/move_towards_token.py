from camera_integrators.raspicam_detector import RaspicamDetector
from sensor_msgs.msg import LaserScan
from helpers.helper import filtered_min
import rospy
import math

max_detection_dist = 0.3
min_detection_dist = 0.03

class MoveTowardsToken:
  """ Target is ahead and in sight """

  def __init__(self, killerrobot):
    self._killerrobot = killerrobot
    self._raspicam_detector = RaspicamDetector()
    rospy.Subscriber('scan', LaserScan, self.__process_scan)
    self._ranges = []
    self._range_min = math.inf

  def __process_scan(self, data: LaserScan):
    self._ranges = data.ranges
    self._range_min = data.range_min

  def isApplicable(self):
    return len(self._raspicam_detector.tokens) > 0

  def execute(self):
    """
    linear velocity: the closer to the token, the slower
    angular velocity: angle of token or if it gets too close to the wall drive along wall
    """
    clst_tkn = self._raspicam_detector.tokens[0]
    linear_velocity = 0.1 * clst_tkn[2] + 0.05
    angular_velocity = clst_tkn[3] * 0.35    
    clst_wall_front_left = filtered_min(self._ranges[30:60], self._range_min)
    if clst_wall_front_left < max_detection_dist:
      # order of sensors: front, front_left, left
      dist = [filtered_min(self._ranges[:15] + self._ranges[-15:], self._range_min), filtered_min(self._ranges[30:60], self._range_min), filtered_min(self._ranges[75:105], self._range_min)]
      max_speed = self._killerrobot.max_speed
      linear_k = [1.5 * max_speed, 0, 0]
      angular_k = [-1, -0.3, 0.2]
      sensitivity_dist = [1.5 * max_detection_dist, max_detection_dist, max_detection_dist]
      angular_velocity = 0
      for i in range(3):
        if dist[i] <= sensitivity_dist[i]:
          linear_velocity -= linear_k[i] * (1 - (dist[i] - min_detection_dist) / (sensitivity_dist[i] - min_detection_dist))
          angular_velocity += angular_k[i] * (1 - (dist[i] - min_detection_dist) / (sensitivity_dist[i] - min_detection_dist))
        rospy.loginfo(f'{linear_velocity}, {angular_velocity}')
    self._killerrobot.move(linear_velocity, angular_velocity)