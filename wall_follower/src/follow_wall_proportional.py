import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

max_speed = 0.13
min_detection_dist = 0.03
max_detection_dist = 0.3

def filtered_min(ranges: list, min_value: float) -> float:
  values_greather_range_min = [i for i in ranges if i > min_value]
  if (len(values_greather_range_min) == 0):
      return math.inf
  return min(values_greather_range_min)


class WallFollower:
  def __init__(self) -> None:
    rospy.init_node('wall_follower_proportional', anonymous=True)
    rospy.Subscriber('scan', LaserScan, self.process_scan)
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    self.behaviors = [TurnTowardsWall(), FollowWall(), FindWall()]
    self.scan = None

  def keep_movin(self):
    while not rospy.is_shutdown():
      for b in self.behaviors:
        if b.isApplicable(self.scan):
          b.execute(self)
          break
      rospy.Rate(10).sleep()

  def process_scan(self, data):
    self.scan = data

  def publish_move(self, linear, angular):
    move = Twist()
    move.linear.x = linear
    move.angular.z = angular
    self.pub.publish(move)


class TurnTowardsWall:
  def isApplicable(self, scan: LaserScan):
    """
    Applicable as soon as the wall bends away from the turtle (it moved past the corner)
    Stops and turns left.
    """
    if scan:
      ranges = scan.ranges
      range_min = scan.range_min
      # order of sensors: front, front_left, left
      dist = [filtered_min(ranges[:15] + ranges[-15:], range_min), filtered_min(ranges[30:60], range_min), filtered_min(ranges[75:105], range_min), filtered_min(ranges[120:150], range_min)]
      if dist[0] > max_detection_dist and dist[1] > max_detection_dist and dist[2] <= max_detection_dist:
        return True
    return False

  def execute(self, turtle: WallFollower):
    rospy.loginfo('turn')
    ranges = turtle.scan.ranges
    range_min = turtle.scan.range_min
    dist = [filtered_min(ranges[:15] + ranges[-15:], range_min), filtered_min(ranges[30:60], range_min), filtered_min(ranges[75:105], range_min), filtered_min(ranges[120:150], range_min)]
    angular_k = 1 # the further away, the sharper the turn
    closeness_percent = (1 - (dist[2] - min_detection_dist) / (max_detection_dist - min_detection_dist))
    linear_velocity = max_speed * closeness_percent # the further away, the slower
    angular_velocity = closeness_percent
    print('turn linear {} angular {} sensors {}'.format(linear_velocity, angular_velocity, dist))
    turtle.publish_move(linear_velocity, angular_velocity) # turn left towards lost wall


class FollowWall:
  """
  Behavior that follows a wall (keeps the wall on its left side)
  Takes into account the sensors in front, left front and on the left hand side
  and calculates the linear and angular velocity.
  """
  def isApplicable(self, scan: LaserScan):
    if scan and any(item < max_detection_dist for item in scan.ranges):
      return True
    return False

  def execute(self, turtle: WallFollower):
    rospy.loginfo('follow')
    ranges = turtle.scan.ranges
    range_min = turtle.scan.range_min
    # order of sensors: front, front_left, left
    dist = [filtered_min(ranges[:15] + ranges[-15:], range_min), filtered_min(ranges[30:60], range_min), filtered_min(ranges[75:105], range_min)]
    linear_k = [1.5 * max_speed, 0, 0]
    angular_k = [-1, -0.3, 0.2]
    sensitivity_dist = [1.5 * max_detection_dist, max_detection_dist, max_detection_dist]
    linear_velocity = max_speed
    angular_velocity = 0
    for i in range(3):
      if dist[i] <= sensitivity_dist[i]:
        linear_velocity -= linear_k[i] * (1 - (dist[i] - min_detection_dist) / (sensitivity_dist[i] - min_detection_dist))
        angular_velocity += angular_k[i] * (1 - (dist[i] - min_detection_dist) / (sensitivity_dist[i] - min_detection_dist))
    print('follow linear {} angular {} sensors {}'.format(linear_velocity, angular_velocity, dist))
    turtle.publish_move(linear_velocity, angular_velocity)


class FindWall:
  """
  Behavior that just moves straight ahead forever and is always applicable
  """
  def isApplicable(self, scan: LaserScan):
    return True

  def execute(self, turtle: WallFollower):
    turtle.publish_move(max_speed, 0)

if __name__ == '__main__':
  try:
    turtle = WallFollower()
    turtle.keep_movin()
    rospy.spin()
  except rospy.ROSInterruptException:
      pass