#!/usr/bin/env python
import math
import rospy
from camera_integrators.raspicam_detector import RaspicamDetector
from helpers.helper import filtered_min
from current_pos.msg import PoseTF
from sensor_msgs.msg import LaserScan

# Config
MAX_DETECTION_DIST = 0.3
MIN_DETECTION_DIST = 0.03
ignore_tokens_above_pixels_y = 150
# cam point grid format: (pixel x, pixel y, map coord x, map coord y), map coords assume that robot pose angle is 0 -> robot stands along x axis
cam_point_grid = [(660,850,4,0), (650,600,6,0), (640,450,8,0), (635,355,10,0), (630,295,12,0), (630,245,14,0), (620,210,16,0), (248,853,4,1.5), (350,596,6,1.5), (407,448,8,1.5), (445,353,10,1.5), (473,290,12,1.5), (490,243,14,1.5), (503,210,16,1.5), (173,444,8,3), (253,352,10,3), (309,284,12,3), (350,242,14,3), (378,207,16,3), (150,280,12,4.5), (200,242,14,4.5), (247,202,16,4.5), (129,199,16,6), (1082,851,4,-1.5), (948,599,6,-1.5), (871,454,8,-1.5), (824,360,10,-1.5), (788,297,12,-1.5), (767,251,14,-1.5), (745,214,16,-1.5), (1099,456,8,-3), (1015,367,10,-3), (946,301,12,-3), (905,253,14,-3), (870,216,16,-3), (1111,304,12,-4.5), (1043,256,14,-4.5), (989,222,16,-4.5), (1105,222,16,-6)]

class MoveTowardsToken:
  """ Unregistered Token is in sight of Raspicam and is approached by Killerrobot. """

  def __init__(self, killerrobot) -> None:
    self._killerrobot = killerrobot
    self._raspicam_detector = RaspicamDetector()
    rospy.Subscriber('scan', LaserScan, self._process_scan)
    rospy.Subscriber('pose_tf', PoseTF, self._process_pose)
    self._ranges = []
    self._range_min = math.inf
    self._unrecognized_tokens = []
    self._pose = None


  def _process_scan(self, data: LaserScan) -> None:
    self._ranges = data.ranges
    self._range_min = data.range_min


  def _process_pose(self, pose: PoseTF):
    self._pose = pose.mapPose


  def isApplicable(self) -> bool:
    """ Raspicam detects tokens that are not yet registered """
    self._unrecognized_tokens = self.filter_unrecognized_tokens()
    return len(self._unrecognized_tokens) > 0


  def execute(self) -> None:
    """
    linear velocity: the closer to the token, the slower
    angular velocity: angle of token or if it gets too close to the wall, drive along wall
    """
    clst_tkn = self._unrecognized_tokens[0]
    linear_velocity = 0.1 * clst_tkn[2] + 0.05
    angular_velocity = clst_tkn[3] * 0.35    
    clst_wall_front_left = filtered_min(self._ranges[30:60], self._range_min)
    if clst_wall_front_left < MAX_DETECTION_DIST:
      # order of sensors: front, front_left, left
      dist = [filtered_min(self._ranges[:15] + self._ranges[-15:], self._range_min), filtered_min(self._ranges[30:60], self._range_min), filtered_min(self._ranges[75:105], self._range_min)]
      max_speed = self._killerrobot.max_speed
      linear_k = [1.5 * max_speed, 0, 0]
      angular_k = [-1, -0.3, 0.2]
      sensitivity_dist = [1.5 * MAX_DETECTION_DIST, MAX_DETECTION_DIST, MAX_DETECTION_DIST]
      angular_velocity = 0
      for i in range(3):
        if dist[i] <= sensitivity_dist[i]:
          linear_velocity -= linear_k[i] * (1 - (dist[i] - MIN_DETECTION_DIST) / (sensitivity_dist[i] - MIN_DETECTION_DIST))
          angular_velocity += angular_k[i] * (1 - (dist[i] - MIN_DETECTION_DIST) / (sensitivity_dist[i] - MIN_DETECTION_DIST))
    self._killerrobot.move(linear_velocity, angular_velocity)


  def filter_unrecognized_tokens(self):
    """ Estimates position of recognized tokens of raspicam and compares to already registered. """
    unrecognized_tokens = []
    for detected_token in self._raspicam_detector.tokens:
      closest_point_in_grid = self.get_closest_grid_point(detected_token)
      estimated_token_map_pose = self.estimate_map_pose_with_angle(closest_point_in_grid)
      if not self.any_registered_token_within_distance(estimated_token_map_pose, 2): # dist 1 = 5cm
        unrecognized_tokens.append(detected_token)
    return unrecognized_tokens


  def get_closest_grid_point(self, token):
    """ Compare euclidean distance of given token to all grid points and return the closest """
    clst_point = (0,0,math.inf)
    for (px,py,mx,my) in cam_point_grid:
      eucl_dist = math.sqrt((px - token[0])**2 + (py - token[1])**2)
      if eucl_dist < clst_point[2]:
        clst_point = (px,py,eucl_dist)
    return {'x': clst_point[0], 'y': clst_point[1], 'distance': clst_point[2]}

  
  def estimate_map_pose_with_angle(self, grid_point):
    """ 
    Calculate the estimated map pose of the given grid point. 
    The grid point specifies how far away it is in map coordinates if the roboter's angle is 0
    """
    new_x = self._pose.x + grid_point['x'] * math.cos(self._pose.angle) + grid_point['y'] * math.cos(math.pi / 2 + self._pose.angle)
    new_y = self._pose.y + grid_point['x'] * math.sin(self._pose.angle) + grid_point['y'] * math.sin(math.pi / 2 + self._pose.angle)
    return {'x': new_x, 'y': new_y }



  def any_registered_token_within_distance(self, estimated_token_map_pose, distance_threshold):
    """ Check if any already registered token is closer in euclidean distance to the estimated token than the given distance threshold """
    for registered_token in self._killerrobot.tokens:
      eucl_dist = math.sqrt((registered_token[1] - estimated_token_map_pose['x'])**2 + (registered_token[2] - estimated_token_map_pose['y'])**2)
      if eucl_dist < distance_threshold:
        return True
    return False
