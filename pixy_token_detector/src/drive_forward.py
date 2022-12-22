#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from pixy_msgs.msg import PixyData

class DriveStraightAhead:
    def __init__(self):

        # Init
        rospy.loginfo("Initialized!")

        # Publish
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
        self.drive_forward()        


    def drive_forward(self):
        while not rospy.is_shutdown():

            (linear, angular) = 0.02,0
            #(linear, angular) = 0.0,0
            #rospy.loginfo("Driving forward: " + str(linear) + ", " + str(angular))
            
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            rospy.Rate(10).sleep()

            
            self._cmd_pub.publish(twist)


def main():
    # Init
    rospy.init_node('drive_forward')

    driver = DriveStraightAhead()
    # Start
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
