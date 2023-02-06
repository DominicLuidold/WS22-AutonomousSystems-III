#!/usr/bin/env python
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Launch arguments
DEBUG = rospy.get_param('debug')
SHOW_LIDAR_DATA = rospy.get_param('lidar_data')
HERTZ_RATE = rospy.get_param('hertz_rate')

# Lidar config
STOP_DISTANCE = 0.25
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class WallFollowerTwo:
    def __init__(self):
        # Init
        rospy.loginfo('WallFollower initialized!')
        self._rate = rospy.Rate(HERTZ_RATE) # A rate higher than 5 Hz is not working properly/too fast

        # Subscribed topics
        self._laser_sub = rospy.Subscriber('/scan', LaserScan, self.update_laser_data)

        # Published topics
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialize laser data until real data is read
        self._laser_data = {
            'left': 0,
            'front_left': 0,
            'front': 0,
            'front_right': 0,
            'right': 0,
            'back_left': 0
        }

        # Start wall-following
        while not rospy.is_shutdown():
            self.move()
            pass 

    def update_laser_data(self, msg: LaserScan) -> None:
        range_min = msg.range_min
        self._laser_data = {
            'back_right': min_greater_range_min(msg.ranges[210:239], range_min),
            'right': min_greater_range_min(msg.ranges[240:299], range_min),
            'front_right': min_greater_range_min(msg.ranges[300:329], range_min),
            'front': min_greater_range_min([min_greater_range_min(msg.ranges[330:359], range_min), min_greater_range_min(msg.ranges[0:29], range_min)], range_min),
            'front_left': min_greater_range_min(msg.ranges[30:59], range_min),
            'left': min_greater_range_min(msg.ranges[60:119], range_min),
            'back_left': min_greater_range_min(msg.ranges[120:149], range_min)
        }

        if DEBUG and SHOW_LIDAR_DATA:
            rospy.logdebug(self._laser_data)

    def move(self) -> None:
        if self._laser_data['front'] > SAFE_STOP_DISTANCE and self._laser_data['front_left'] > SAFE_STOP_DISTANCE and self._laser_data['front_right'] > SAFE_STOP_DISTANCE and self._laser_data['back_right'] > SAFE_STOP_DISTANCE:
            # Wall not in front
            twist = self.drive_to_wall_in_front()
        elif self._laser_data['front'] > SAFE_STOP_DISTANCE and self._laser_data['front_left'] > SAFE_STOP_DISTANCE and self._laser_data['front_right'] > SAFE_STOP_DISTANCE and self._laser_data['back_right'] < SAFE_STOP_DISTANCE:
            # Wall back right (90° right corners)
            twist = self.turn_right()
        elif self._laser_data['front'] < SAFE_STOP_DISTANCE and self._laser_data['front_left'] > SAFE_STOP_DISTANCE and self._laser_data['front_right'] > SAFE_STOP_DISTANCE:
            # Wall in front;
            twist = self.turn_left()
        elif self._laser_data['front'] > SAFE_STOP_DISTANCE and self._laser_data['front_left'] > SAFE_STOP_DISTANCE and self._laser_data['front_right'] < SAFE_STOP_DISTANCE:
            # wall to right
            twist = self.follow_wall()
        elif self._laser_data['front'] > SAFE_STOP_DISTANCE and self._laser_data['front_left'] < SAFE_STOP_DISTANCE and self._laser_data['front_right'] > SAFE_STOP_DISTANCE:
            # wall in front - left
            twist = self.find_wall()
        elif self._laser_data['front'] < SAFE_STOP_DISTANCE and self._laser_data['front_left'] > SAFE_STOP_DISTANCE and self._laser_data['front_right'] < SAFE_STOP_DISTANCE:
            # wall in front - right
            twist = self.turn_left()
        elif self._laser_data['front'] < SAFE_STOP_DISTANCE and self._laser_data['front_left'] < SAFE_STOP_DISTANCE and self._laser_data['front_right'] > SAFE_STOP_DISTANCE:
            # wall in front , front - left no right
            twist = self.turn_left()
        elif self._laser_data['front'] < SAFE_STOP_DISTANCE and self._laser_data['front_left'] < SAFE_STOP_DISTANCE and self._laser_data['front_right'] < SAFE_STOP_DISTANCE:
            # wall in front-left
            twist = self.turn_left()
        elif self._laser_data['front'] > SAFE_STOP_DISTANCE and self._laser_data['front_left'] < SAFE_STOP_DISTANCE and self._laser_data['front_right'] < SAFE_STOP_DISTANCE:
            # wall in front - left and front right
            twist = self.find_wall()
        else:
            twist = Twist()
            rospy.logwarn('Unknown robot status/location')

        self._cmd_vel_pub.publish(twist)
        self._rate.sleep()

    def drive_to_wall_in_front(self) -> Twist:
        rospy.loginfo('Driving to wall in front..')
        twist = Twist()
        twist.linear.x = 0.2

        return twist

    def find_wall(self) -> Twist:
        rospy.loginfo('Finding wall..')
        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = -0.25

        return twist

    def turn_left(self) -> Twist:
        rospy.loginfo('Turning left..')
        twist = Twist()
        twist.angular.z = 0.3

        return twist

    def turn_right(self) -> Twist:
        rospy.loginfo('Turning right..')
        twist = Twist()
        twist.angular.z = -0.8

        return twist

    def follow_wall(self) -> Twist:
        rospy.loginfo('Following wall..')
        twist = Twist()
        twist.linear.x = 0.5

        return twist

def min_greater_range_min(ranges: list, min_value: float) -> float:
    values_greather_range_min = [i for i in ranges if i > min_value]
    if (len(values_greather_range_min) == 0):
        return math.inf

    return min(values_greather_range_min)

def main() -> None:
    # Init
    log_level = rospy.DEBUG if DEBUG else rospy.INFO
    rospy.init_node('wall_follower', log_level=log_level)

    # Start
    try:
        WallFollowerTwo()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down!')

if __name__ == '__main__':
    main()
