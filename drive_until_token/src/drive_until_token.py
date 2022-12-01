#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from pixy_msgs.msg import PixyData

class DriveUntilToken:
    def __init__(self):
        # Init
        rospy.loginfo("Initialized!")
        
        # Behaviors
        self._behaviors = [DriveAndSearch(), FoundTag()]
        self._currBehavior = 0

        # Publish
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._rate = rospy.Rate(10)

        # Start
        self.process()

    def process(self):
        (linear, angular, changeBehavior) = self._behaviors[self._currBehavior].process()
        rospy.loginfo(self._behaviors[self._currBehavior])
        
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        self._currBehavior += changeBehavior
        self._cmd_pub.publish(twist)

class DriveAndSearch:
    def __init__(self):
        # Init 
        rospy.loginfo("Drive and Search")

        # Subscribe
        self._cmd_sub = rospy.Subscriber('/my_pixy/block_data', PixyData)

    def process(self):
        if self.found_tag():
            return (0.2, 0, 1)
        else:
            return (0, 0.2, 0)

    def found_tag(self) -> bool:
        rospy.loginfo(self._cmd_sub)

        return False

class FoundTag:
    def __init__(self):
        pass

    def process(self):
        rospy.loginfo("found tag")
        rospy.loginfo("loud noise")

        return (1, 1, 0)

def main():
    # Init
    rospy.init_node('turtlebot3_drive_until_token')

    # Start
    try:
        DriveUntilToken()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
