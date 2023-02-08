from token_inspector.srv import GimmeGoal, GimmeGoalResponse
from sensor_msgs.msg import LaserScan
import rospy
from helpers.helper import filtered_min, get_angle_difference
from geometry_msgs.msg import Twist
from current_pos.msg import PoseTF, PoseInMap
from enum import Enum

class WallFollowerRunner:
    def __init__(self) -> None:
        self._goal = None
        self._wall_follower = WallFollower()

    def is_applicable(self, goal:GimmeGoalResponse):
        return True

    def run(self, goal:GimmeGoalResponse):
        if goal != self._goal:
            self._wall_follower.pursue_new_goal(goal)
            self._goal = goal
        self._wall_follower.follow()



MAX_SPEED = 0.13
MIN_DETECTION_DIST = 0.03
MAX_DETECTION_DIST = 0.3
MAX_TARGET_DISTANCE = 0.15

class Direction(Enum):
    LEFT = 1
    RIGHT = 2

class WallFollower:

    def __init__(self) -> None:
        self._behaviors = [TurnTowardsGoal(self), TurnTowardsWall(self), FollowWall(self), TowardsGoal(self)]
        self._scan = None
        self._dist = None
        self.turned_towards_goal = False
        rospy.Subscriber('scan', LaserScan, self._process_scan)
        self.goal = None
        self._direction:Direction = None
        self._movement_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def _process_scan(self, data: LaserScan) -> None:
        self._scan = data

    def pursue_new_goal(self, goal: GimmeGoalResponse):
        self.goal = goal
        self.turned_towards_goal = False

    def follow(self):
        if self._scan and self.goal:
            ranges = self._scan.ranges
            range_min = self._scan.range_min
            dist = {'front': filtered_min(ranges[:20] + ranges[-15:], range_min), \
                    'front_left': filtered_min(ranges[20:60], range_min), \
                    'left': filtered_min(ranges[60:105], range_min)}
            for b in self._behaviors:
                if b.is_applicable(dist, self._direction):
                    b.execute(dist, self._direction, self._move)
                    break


    def _move(self, linear, angular):
        move = Twist()
        move.linear.x = linear
        move.angular.z = angular
        self._movement_publisher.publish(move)


class TurnTowardsGoal:
    """ Turn towards goal until goal is ahead (no matter if wall is inbetween) """

    def __init__(self, wall_follower:WallFollower) -> None:
        self._master = wall_follower
        self._pose: PoseInMap = None
        rospy.Subscriber('pose_tf', PoseTF, self._process_pose)


    def is_applicable(self, dist, direction: Direction) -> bool:
        return not self._master.turned_towards_goal and self._pose


    def execute(self, dist, direction: Direction, invoke_move):
        goal = self._master.goal
        angle = get_angle_difference(self._pose.x, self._pose.y, self._pose.angle, goal.x, goal.y)
        rospy.logerr(angle)
        if abs(angle) > 0.05:
            invoke_move(0, angle)
        else:
            invoke_move(0, 0)
            self._master.turned_towards_goal = True


    def _process_pose(self, posetf: PoseTF):
        self._pose = posetf.mapPose

        

class TurnTowardsWall:

    def __init__(self, wall_follower:WallFollower) -> None:
        self._master = wall_follower

    def is_applicable(self, dist, direction: Direction) -> bool:
        """
        Applicable as soon as the wall bends away from the turtle (it moved past the corner)
        Stops and turns left.
        """
        return self._master.turned_towards_goal and dist['front'] > MAX_DETECTION_DIST and dist['front_left'] > MAX_DETECTION_DIST and dist['left'] <= MAX_DETECTION_DIST

    def execute(self, dist, direction: Direction, invoke_move) -> None:
        #rospy.logdebug('behavior: turn towards wall')
        closeness_percent = max((1 - (dist['left'] - MIN_DETECTION_DIST) / (MAX_DETECTION_DIST - MIN_DETECTION_DIST)), 0.1)
        linear_velocity = MAX_SPEED * closeness_percent # the further away, the slower
        angular_velocity = closeness_percent
        rospy.logdebug('turn linear {} angular {} closeness {} sensors {}'.format(linear_velocity, angular_velocity, closeness_percent, dist))
        invoke_move(linear_velocity, angular_velocity) # turn left towards lost wall


class FollowWall:
    """
    Behavior that follows a wall (keeps the wall on its left side)
    Takes into account the sensors in front, left front and on the left hand side
    and calculates the linear and angular velocity.
    """
    def __init__(self, wall_follower:WallFollower) -> None:
        self._master = wall_follower

    def is_applicable(self, dist, direction: Direction) -> bool:
        return self._master.turned_towards_goal and any(distance < MAX_DETECTION_DIST for distance in dist.values())

    def execute(self, distance, direction: Direction, invoke_move) -> None:
        #rospy.logdebug('behavior: follow wall')
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
        #rospy.logdebug('follow linear {} angular {} sensors {}'.format(linear_velocity, angular_velocity, dist))
        invoke_move(linear_velocity, angular_velocity)


class TowardsGoal:
    """ Move straight towards goal"""
    def __init__(self, wall_follower:WallFollower) -> None:
        self._master = wall_follower

    def is_applicable(self, dist, direction: Direction) -> bool:
        return self._master.turned_towards_goal

    def execute(self, dist, direction: Direction, invoke_move) -> None:
        #rospy.logdebug('behavior: find wall')
        invoke_move(0.1, 0)