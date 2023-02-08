import math
from current_pos.msg import PoseInMap


def filtered_min(ranges: list, min_value: float) -> float:
  """ returns minimum value in ranges that is not lower than min_value """
  values_greather_range_min = [i for i in ranges if i > min_value]
  if (len(values_greather_range_min) == 0):
      return math.inf
  return min(values_greather_range_min)


def any_registered_token_within_distance(estimated_token_map_pose: PoseInMap, registered_tokens, distance_threshold):
  """ Check if any already registered token is closer in euclidean distance to the estimated token than the given distance threshold """
  for registered_token in registered_tokens:
    eucl_dist = math.sqrt((registered_token[1] - estimated_token_map_pose.x)**2 + (registered_token[2] - estimated_token_map_pose.y)**2)
    if eucl_dist < distance_threshold:
      return True
  return False


def eucl_distance(x1, y1, x2, y2):
  return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def get_angle_difference(current_x, current_y, current_heading, target_x, target_y):
        '''
        current_heading is in radians
        angle difference is in radians
        '''
        # Calculate the direction vector
        direction_vector = [target_x - current_x, target_y - current_y]

        # Calculate the angle between the direction vector and the x-axis
        target_angle = math.atan2(direction_vector[1], direction_vector[0])

        # Calculate the difference between the current heading and the target angle
        angle_difference = target_angle - current_heading

        # Normalize the angle difference to be within the range of -pi and pi
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

        # Return the angle difference
        return angle_difference