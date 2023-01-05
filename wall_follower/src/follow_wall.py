#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Launch arguments
DEBUG = rospy.get_param('debug')
SHOW_LIDAR_DATA = rospy.get_param('lidar_data')

# Lidar config
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class WallFollowerTwo:
    def __init__(self):
        # Init
        rospy.loginfo('WallFollower initialized!')
        self._rate = rospy.Rate(5) # A rate higher than 5 Hz is not working properly/too fast

        # Subscribed topics
        self._laser_sub = rospy.Subscriber('/scan', LaserScan, self.updateLaserData)

        # Published topics
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialize laser data until real data is read
        self._laser_data = {
            'fl': 0,
            'l': 0,
            'f': 0,
            'r': 0,
            'fr': 0
        }

        # Start wall-following
        while not rospy.is_shutdown():
            self.step()
            pass 

    def updateLaserData(self, msg) -> None:
        self._laser_data = {
            'fl': min(min(msg.ranges[30:60]), 10),
            'l': min(min(msg.ranges[61:120]), 10),
            'f': min(min(msg.ranges[330:359]), min(msg.ranges[0:30]), 10),
            'r': min(min(msg.ranges[260:299]), 10),
            'fr': min(min(msg.ranges[300:329]), 10)
        }

        if DEBUG and SHOW_LIDAR_DATA:
            rospy.logdebug(self._laser_data)

    def step(self) -> None:
        self.sensePlan()

        if self.state == 0:
            rospy.logdebug("Searching for wall..")
            twist = self.findWall()
        elif self.state == 1:
            rospy.logdebug("Turning left..")
            twist = self.turnLeft()
        elif self.state == 2:
            rospy.logdebug("Following wall..")
            twist = self.followWall()
        elif self.state == 3:
            rospy.logdebug("Turning right..")
            twist = self.turnRight()
        else:
            rospy.logwarn('Unknown state!')

        self._cmd_vel_pub.publish(twist)
        self._rate.sleep()

    def sensePlan(self) -> int:
        fl = self._laser_data['fl']
        l = self._laser_data['l']
        f = self._laser_data['f']
        fr = self._laser_data['fr']
        r = self._laser_data['r']

        if f < SAFE_STOP_DISTANCE:
            # Wall in front - turn left
            self.state = 1
        elif r < SAFE_STOP_DISTANCE:
            # Wall to the right - follow wall
            self.state = 2
        elif f > SAFE_STOP_DISTANCE and fr > SAFE_STOP_DISTANCE and r > SAFE_STOP_DISTANCE:
            # No wall found
            self.state = 0
        else:
           rospy.logerr("Roboter is in unkown state")

    def findWall(self) -> Twist:
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0
        return twist

    def turnLeft(self) -> Twist:
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0.2
        return twist

    def turnRight(self) -> Twist:
        twist = Twist()
        twist.angular.z = -0.2
        return twist

    def followWall(self) -> Twist:
        twist = Twist()
        twist.linear.x = 0.2
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
