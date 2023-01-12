#! /usr/bin/env python
#Does not yet work
#Idea: /odom gives estimated position relative to start
#Use: when wallfollower drives around the walls, he stops at his estimated starting position
import rospy
from nav_msgs.msg import Odometry

DEBUG = rospy.get_param('debug')

class OdometrySub:
    def __init__(self) -> None:
        rospy.loginfo("OdometrySub initialized!")
        self._odom_sub = rospy.Subscriber('/odom', Odometry, callback)

def callback(self, msg):
    rospy.loginfo(msg.pose.pose)



def main() -> None:
    # Init
    log_level = rospy.DEBUG if DEBUG else rospy.INFO
    rospy.init_node('check_odometry', log_level=log_level)

    # Start
    try:
        OdometrySub()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('Shutting down!')

if __name__ == '__main__':
    main()