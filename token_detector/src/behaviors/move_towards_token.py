#!/usr/bin/env python
import math
import rospy
from camera_integrators.raspicam_detector import RaspicamDetector
from helpers.helper import filtered_min
from sensor_msgs.msg import LaserScan
from helpers.helper import any_registered_token_within_distance
from current_pos.msg import PoseInMap

# Config
MAX_DETECTION_DIST = 0.3
MIN_DETECTION_DIST = 0.03
ignore_tokens_above_pixels_y = 150
# cam point grid format: (pixel x, pixel y, map coord x, map coord y), map coords assume that robot pose angle is 0 -> robot stands along x axis
cam_point_grid_1280x960 = [(660,850,4,0), (650,600,6,0), (640,450,8,0), (635,355,10,0), (630,295,12,0), (630,245,14,0), (620,210,16,0), (248,853,4,1.5), (350,596,6,1.5), (407,448,8,1.5), (445,353,10,1.5), (473,290,12,1.5), (490,243,14,1.5), (503,210,16,1.5), (173,444,8,3), (253,352,10,3), (309,284,12,3), (350,242,14,3), (378,207,16,3), (150,280,12,4.5), (200,242,14,4.5), (247,202,16,4.5), (129,199,16,6), (1082,851,4,-1.5), (948,599,6,-1.5), (871,454,8,-1.5), (824,360,10,-1.5), (788,297,12,-1.5), (767,251,14,-1.5), (745,214,16,-1.5), (1099,456,8,-3), (1015,367,10,-3), (946,301,12,-3), (905,253,14,-3), (870,216,16,-3), (1111,304,12,-4.5), (1043,256,14,-4.5), (989,222,16,-4.5), (1105,222,16,-6)]
cam_point_grid_410x308 = [(209,264,0.2,0), (209,190,0.3,0), (209,144,0.4,0), (210,115,0.5,0), (211,93,0.6,0), (212,80,0.7,0), (210,67,0.8,0), (76,264,0.2,0.075), (115,186,0.3,0.075), (138,141,0.4,0.075), (151,112,0.5,0.075), (162,92,0.6,0.075), (168,78,0.7,0.075), (173,67,0.8,0.075), (22,181,0.3,0.15), (63,138,0.4,0.15), (91,110,0.5,0.15), (110,90,0.6,0.15), (124,77,0.7,0.15), (134,65,0.8,0.15), (0,134,0.4,0.225), (28,108,0.5,0.225), (60,87,0.6,0.225), (78,74,0.7,0.225), (92,63,0.8,0.225), (12,86,0.6,0.3), (35,72,0.7,0.3), (55,64,0.8,0.3), (4,69,0.7,0.375), (23,58,0.8,0.375), (340,270,0.2,-0.075), (308,192,0.3,-0.075), (285,145,0.4,-0.075), (272,115,0.5,-0.075), (262,96,0.6,-0.075), (256,80,0.7,-0.075), (252,68,0.8,-0.075), (399,193,0.3,-0.15), (360,148,0.4,-0.15), (334,117,0.5,-0.15), (313,98,0.6,-0.15), (301,82,0.7,-0.15), (292,70,0.8,-0.15), (410,147,0.4,-0.225), (393,121,0.5,-0.225), (368,101,0.6,-0.225), (346,83,0.7,-0.225), (330,72,0.8,-0.225), (407,100,0.6,-0.3), (390,85,0.7,-0.3), (369,73,0.8,-0.3), (410,87,0.7,-0.375), (400,73,0.8,-0.375)]

class MoveTowardsToken:
  """ Unregistered Token is in sight of Raspicam and is approached by Killerrobot. """

  def __init__(self, killerrobot) -> None:
    self._killerrobot = killerrobot
    self._raspicam_detector = RaspicamDetector()
    rospy.Subscriber('scan', LaserScan, self._process_scan)
    self._ranges = []
    self._range_min = math.inf
    self._unrecognized_tokens = []


  def _process_scan(self, data: LaserScan) -> None:
    self._ranges = data.ranges
    self._range_min = data.range_min


  def isApplicable(self) -> bool:
    """ Raspicam detects tokens that are not yet registered """
    self._unrecognized_tokens = self.filter_unrecognized_tokens()
    return len(self._unrecognized_tokens) > 0


  def execute(self) -> None:
    """
    linear velocity: the closer to the token, the slower
    angular velocity: angle of token or if it gets too close to the wall, drive along wall
    """
    rospy.logdebug('behavior: move_towards_token')
    clst_tkn = self._unrecognized_tokens[0]
    linear_velocity = 0.1 * clst_tkn[2] + 0.05
    angular_velocity = clst_tkn[3] * 0.35
    distance = {'front': filtered_min(self._ranges[:20] + self._ranges[-15:], self._range_min),
                'front_left': filtered_min(self._ranges[20:60], self._range_min),
                'left': filtered_min(self._ranges[60:105], self._range_min),
                'left_behind': filtered_min(self._ranges[105:135], self._range_min),
                'front_right': filtered_min(self._ranges[-60:-20], self._range_min),
                'right': filtered_min(self._ranges[-105:-60], self._range_min),
                'right_behind': filtered_min(self._ranges[-135:-105], self._range_min)}
    if distance['front_left'] < MAX_DETECTION_DIST and distance['front_left'] < distance['left_behind']:
      rospy.logwarn('correct left')
      angular_k = [-1, -0.4, 0.2]
      dist = [distance['front'], distance['front_left'], distance['left']]
      linear_velocity, angular_velocity = self.calc_wall_movement(angular_k, dist, linear_velocity)
    elif distance['front_right'] < MAX_DETECTION_DIST and distance['front_right'] < distance['right_behind']:
      rospy.logwarn('correct right')
      angular_k = [1, 0.4, -0.2]
      dist = [distance['front'], distance['front_right'], distance['right']]
      linear_velocity, angular_velocity = self.calc_wall_movement(angular_k, dist, linear_velocity)
    rospy.logdebug(f'lin {linear_velocity}, ang {angular_velocity}')
    self._killerrobot.move(linear_velocity, angular_velocity)

  def calc_wall_movement(self, angular_k, dist, linear_velocity):
    max_speed = self._killerrobot.max_speed
    linear_k = [1.5 * max_speed, 0, 0]
    sensitivity_dist = [1.5 * MAX_DETECTION_DIST, MAX_DETECTION_DIST, MAX_DETECTION_DIST]
    angular_velocity = 0
    for i in range(3):
      if dist[i] <= sensitivity_dist[i]:
        linear_velocity -= linear_k[i] * (1 - (dist[i] - MIN_DETECTION_DIST) / (sensitivity_dist[i] - MIN_DETECTION_DIST))
        angular_velocity += angular_k[i] * (1 - (dist[i] - MIN_DETECTION_DIST) / (sensitivity_dist[i] - MIN_DETECTION_DIST))
    return linear_velocity, angular_velocity


  def filter_unrecognized_tokens(self):
    """ Estimates position of recognized tokens of raspicam and compares to already registered. """
    unrecognized_tokens = []
    for detected_token in self._raspicam_detector.tokens:
      closest_point_in_grid = self.get_closest_grid_point(detected_token)
      rospy.logdebug(f'closest point: {closest_point_in_grid}')
      estimated_token_map_pose = self.estimate_map_pose_with_angle(closest_point_in_grid)
      rospy.logdebug(f'estimated map pose: x {estimated_token_map_pose.x}, y {estimated_token_map_pose.y}, angle {estimated_token_map_pose.angle}')
      if not any_registered_token_within_distance(estimated_token_map_pose, self._killerrobot.tokens, 0.2): # dist 1 = 1m
        unrecognized_tokens.append(detected_token)
        rospy.logdebug(f'unrecognized token x {estimated_token_map_pose.x}, y {estimated_token_map_pose.y}')
      else:
        rospy.logdebug(f'ignore token x {estimated_token_map_pose.x}, y {estimated_token_map_pose.y}')
    return unrecognized_tokens


  def get_closest_grid_point(self, token):
    """ Compare euclidean distance of given token to all grid points and return the closest """
    clst_point = (0,0,math.inf) # (x,y,distance)
    for (px,py,mx,my) in cam_point_grid_410x308:
      eucl_dist_pix = math.sqrt((px - token[0])**2 + (py - token[1])**2)
      if eucl_dist_pix < clst_point[2]:
        clst_point = (mx,my,eucl_dist_pix)
    return {'x': clst_point[0], 'y': clst_point[1], 'distance': clst_point[2]}

  
  def estimate_map_pose_with_angle(self, grid_point) -> PoseInMap:
    """ 
    Calculate the estimated map pose of the given grid point. 
    The grid point specifies how far away it is in map coordinates if the roboter's angle is 0
    """
    pose = self._killerrobot.pose
    new_x = pose.x + grid_point['x'] * math.cos(pose.angle) + grid_point['y'] * math.cos(math.pi / 2 + pose.angle)
    new_y = pose.y + grid_point['x'] * math.sin(pose.angle) + grid_point['y'] * math.sin(math.pi / 2 + pose.angle)
    return PoseInMap(x = new_x, y = new_y)
