class MoveTowardsToken:
  """
  Target is ahead and in sight
  """
  def isApplicable(self, tokens):
    return len(tokens) > 0

  def execute(self, killerrobot, tokens):
    clst_tkn = tokens[0]
    linear_velocity = 0.5 * clst_tkn[2] + 0.05
    angular_velocity = clst_tkn[3] * 0.3
    killerrobot.move(linear_velocity, angular_velocity)
    """
    closest_dist = min(turtle.scan.ranges)
    closest_index = turtle.scan.ranges.index(closest_dist)
    distance_percent = min((closest_dist - min_detection_dist) / (0.5 - min_detection_dist), 1)
    linear_velocity = max_speed * distance_percent
    angular_velocity = 0
    if closest_index < 180: # target is on left side -> turn left
      angular_velocity = closest_index / 20 * 0.5
    else:
      angular_velocity = (360 - closest_index) / 20 * -0.5
    print('towards linear {} angular {} min distance {} closest index {}'.format(linear_velocity, angular_velocity, closest_dist, closest_index))
    turtle.publish_move(linear_velocity, angular_velocity)
    """