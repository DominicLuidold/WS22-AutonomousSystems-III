import sys
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from helpers.helper import filtered_min
from std_srvs.srv import Empty

ELLIPSE_SIZE_THRESHOLD = 0.5

class WallLocalizer:
    def __init__(self) -> None:
        rospy.wait_for_service('global_localization')
        self._global_localisation = rospy.ServiceProxy('global_localization', Empty)
        self._wall_follower = WallFollower()
        # get service for global localization

    def localize(self):
        self._global_localisation()
        while not rospy.is_shutdown() and self.calc_estimated_pose_ellipse() > ELLIPSE_SIZE_THRESHOLD:
            self._wall_follower.follow()
            rospy.Rate(10).sleep()

    def calc_estimated_pose_ellipse(self):
        return 1


MAX_SPEED = 0.13
MIN_DETECTION_DIST = 0.03
MAX_DETECTION_DIST = 0.3
MAX_TARGET_DISTANCE = 0.15

class WallFollower:
    """ Follow wall on the left side """

    def __init__(self) -> None:
        self._behaviors = [TurnTowardsWall(), FollowWall(), FindWall()]
        self._scan = None
        rospy.Subscriber('scan', LaserScan, self._process_scan)
        self._movement_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def _process_scan(self, data: LaserScan) -> None:
        self._scan = data


    def follow(self):
        if self._scan:
            ranges = self._scan.ranges
            range_min = self._scan.range_min
            dist = {'front': filtered_min(ranges[:15] + ranges[-15:], range_min), \
                          'front_left': filtered_min(ranges[30:60], range_min), \
                          'left': filtered_min(ranges[75:105], range_min)}
            for b in self._behaviors:
                if b.is_applicable(dist):
                    b.execute(dist, self._move)
                    break

    def _move(self, linear, angular):
        move = Twist()
        move.linear.x = linear
        move.angular.z = angular
        rospy.logdebug(f'pub {move.linear.x} {move.angular.z}')
        self._movement_publisher.publish(move)




class FindWall:
    """ Move slightly left ahead """

    def is_applicable(self, dist) -> bool:
        return True

    def execute(self, dist, move) -> None:
        move(0.1, 0.1)


class TurnTowardsWall:

    def is_applicable(self, dist) -> bool:
        """
        Applicable as soon as the wall bends away from the turtle (it moved past the corner)
        Stops and turns left.
        """
        return dist['front'] > MAX_DETECTION_DIST and dist['front_left'] > MAX_DETECTION_DIST and dist['left'] <= MAX_DETECTION_DIST

    def execute(self, dist, move) -> None:
        rospy.logdebug('behavior: turn towards wall')
        closeness_percent = max((1 - (dist['left'] - MIN_DETECTION_DIST) / (MAX_DETECTION_DIST - MIN_DETECTION_DIST)), 0.1)
        linear_velocity = MAX_SPEED * closeness_percent # the further away, the slower
        angular_velocity = closeness_percent
        rospy.logdebug('turn linear {} angular {} closeness {} sensors {}'.format(linear_velocity, angular_velocity, closeness_percent, dist))
        move(linear_velocity, angular_velocity) # turn left towards lost wall


class FollowWall:
    """
    Behavior that follows a wall (keeps the wall on its left side)
    Takes into account the sensors in front, left front and on the left hand side
    and calculates the linear and angular velocity.
    """

    def is_applicable(self, dist) -> bool:
        return any(distance < MAX_DETECTION_DIST for distance in dist.values())

    def execute(self, distance, move) -> None:
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
        move(linear_velocity, angular_velocity)



def main(args):
    rospy.init_node('amcl_localizer', anonymous=True)
    loc = WallLocalizer()
    loc.localize()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)