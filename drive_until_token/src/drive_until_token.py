import rospy
from geometry_msgs.msg import Twist
from pixy_msgs.msg import PixyData

class DriveUntilToken:
    def __init__(self) -> None:
        rospy.init_node('turtlebot3_drive_until_token', anonymous=False)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.loginfo("Initialized")
        self.behaviors = [DriveAndSearch(), FoundTag()]
        self.currBehavior = 0
        self.process()

    def process(self):
        (linear, angular, changeBehavior) = self.behaviors[self.currBehavior]
        tw = Twist()
        tw.linear.x = linear
        tw.angular.z = angular
        self.currBehavior += changeBehavior
        self.pub.publish(tw)

class DriveAndSearch:
    def __init__(self) -> None:
        self.process()

    def process(self):
        rospy.loginfo("drive and search")
        if self.found_tag() :
            return 0, 0, 1
        else:
            return 1, 0, 0

    def found_tag(self):
        sub = rospy.Subscriber('/my_pixy/block_data', data_class=PixyData)
        rospy.loginfo(sub)
        return False

class FoundTag:
    def __init__(self) -> None:
        self.process()

    def process(self):
        rospy.loginfo("found tag")
        rospy.loginfo("loud noise")

if __name__ == '__main__':
    try:
        DriveUntilToken()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass