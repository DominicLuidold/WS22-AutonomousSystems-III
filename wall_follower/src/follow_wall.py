#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollowerTwo:
    def __init__(self):
        # Init
        rospy.loginfo('WallFollower initialized!')

        self._rate = rospy.Rate(10)
        self._laser_data = None

        # Subscribed topics
        self._laser_sub = rospy.Subscriber('/scan', LaserScan, self.updateLaserData)

        # Published topics
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Start wall-following
        # self.step()

    def updateLaserData(self, msg) -> None:
        self._laser_data = {
            'fr': min(min(msg.ranges[30:60]), 10),
            'r': min(min(msg.ranges[61:120]), 10),
            'f': min(min(msg.ranges[330:359]), min(msg.ranges[0:30]), 10),
            'l': min(min(msg.ranges[260:299]), 10),
            'fl': min(min(msg.ranges[300:329]), 10)
        }
        print(self._laser_data)

    def step(self) -> None:
        while not rospy.is_shutdown():
            twist = self.sensePlan()
            self._cmd_vel_pub.publish(twist)

    def sensePlan(self) -> Twist:
        if not self._laser_data:
            rospy.logerr('No laser data found!')
            return Twist()

        rospy.logdebug('blubb')
        data = np.array(self._laser_data)
        k = 5
        closestK = np.argpartition(data, k)
        print(closestK[:k])
        # print(data[closestK[:k]])

        d = 0.5  # Distance to keep away from wall
        twist = Twist()

        if data[0] < d:
            # Frontal obstacle detected -> avoidance
            # print("Evade front - Turn Left")
            # twist.angular.z = 0.2
            print('Something in FRONT')
        elif 260 < closestK[0] < 280:
            # The closest measurement is to the right side
            print("Follow Wall")
            twist.linear.x = 0.2
            # Correct steering
            steer = max(min(closestK[0] - 270, 1), -1)
            if data[closestK[0]] < (d / 1.5):
                # Override steer if entered distance threshold
                steer = 1
            twist.angular.z = 0.005 * steer
        elif not (120 < closestK[0] < 240) and data[closestK[0]] < d:
            # Obstacle ahead, need to turn
            # Fast rotate based on sensor provided angle
            if 270 < closestK[0] or closestK[0] < 90:
                print("Turn Left")
                twist.angular.z = 0.2
            else:
                print("Turn Right")
                twist.angular.z = -0.05
        else:
            # Dive to wall or clear back of robot if closest walls are behind
            if not (120 < closestK[0] < 240):
                print("Lost wall")
            else:
                print("Forward to clear back of robot")
            twist.linear.x = 0.05
        return twist


def main() -> None:
    # Init
    rospy.init_node('wall_follower')

    # Start
    try:
        WallFollowerTwo()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down!')

if __name__ == '__main__':
    main()
