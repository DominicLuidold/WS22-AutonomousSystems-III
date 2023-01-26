import math

def filtered_min(ranges: list, min_value: float) -> float:
  """ returns minimum value in ranges that is not lower than min_value """
  values_greather_range_min = [i for i in ranges if i > min_value]
  if (len(values_greather_range_min) == 0):
      return math.inf
  return min(values_greather_range_min)