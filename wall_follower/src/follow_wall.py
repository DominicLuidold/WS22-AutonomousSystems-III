#!/usr/bin/env python
import rospy
import numpy as np
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
            'back_left': 0,
            'left': 0,
            'front_left': 0,
            'front': 0,
            'front_right': 0,
            'right': 0,
            'back_right': 0
        }

        # Start wall-following
        while not rospy.is_shutdown():
            self.move()
            pass 

    def update_laser_data(self, msg: LaserScan) -> None:
        self._laser_data = {
            'back_left':  min(msg.ranges[120:149]),
            'left':  min(msg.ranges[75:104]),
            'front_left': min(msg.ranges[30:59]),
            'front': min(min(msg.ranges[0:14]), min(msg.ranges[345:359])),
            'front_right':  min(msg.ranges[300:329]),
            'right':  min(msg.ranges[255:284]),
            'back_right':  min(msg.ranges[210:239])
        }

        if DEBUG and SHOW_LIDAR_DATA:
            rospy.logdebug(self._laser_data)

    def move(self) -> None:
        back_left = self._laser_data['back_left']
        left = self._laser_data['left']
        front_left = self._laser_data['front_left']
        front = self._laser_data['front']
        front_right = self._laser_data['front_right']
        right = self._laser_data['right']
        back_right = self._laser_data['back_right']

        # if any left is smallest: turn left
        # if front is smallest or front is < threshold: turn left
        # if rigt is smallest: drive straight
            # if front_right is smaller than back_right: turn left a bit
            # if back_right is smaller than front_right: turn right a bit
        # if back_right is smallest: turn right a lot

        twist = Twist()       
        self._cmd_vel_pub.publish(twist)
        self._rate.sleep()

    def findWall(self) -> Twist:
        rospy.loginfo("Finding wall..")
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0

        return twist

    def turnLeft(self) -> Twist:
        rospy.loginfo("Turning left..")
        twist = Twist()
        twist.angular.z = 0.4

        return twist

    def followWall(self) -> Twist:
        rospy.loginfo("Following wall..")
        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = -0.2

        return twist

def main() -> None:
    # Init
    log_level = rospy.DEBUG if DEBUG else rospy.INFO
    rospy.init_node('wall_follower', log_level=log_level)

    # Start
    try:
        WallFollowerTwo()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('Shutting down!')

if __name__ == '__main__':
    main()
