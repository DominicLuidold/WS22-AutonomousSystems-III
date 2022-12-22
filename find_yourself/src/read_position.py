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
        self._cmd_sub = rospy.Subscriber('/my_pixy/block_data', PixyData, self.process)



    def process(self, data):
        (linear, angular, changeBehavior) = self._behaviors[self._currBehavior].process(data)
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

    def process(self, data):
        if self.found_tag(data):
            rospy.loginfo("stop")
            return (0, 0, 1)
        else:
            rospy.loginfo("run")
            return (0.02, 0, 0)

    def found_tag(self, data) -> bool:
        rospy.loginfo("This: " + str(data))
        if data.header.stamp.secs == 0:
            return False
        else:
            return True

class FoundTag:
    def __init__(self):
        pass

    def process(self, data):
        rospy.loginfo("found tag")
        rospy.loginfo("loud noise")

        return (0, 0.2, 0)

def main():
    # Init
    rospy.init_node('turtlebot3_find_position')

    # Start
    try:
        DriveUntilToken()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
