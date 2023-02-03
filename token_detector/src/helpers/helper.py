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