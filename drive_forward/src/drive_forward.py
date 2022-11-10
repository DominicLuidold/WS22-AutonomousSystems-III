import rospy
from geometry_msgs.msg import Twist

class DriveForward():
    def __init__(self):

        rospy.init_node("DriveForward", anonymous=False)

        rospy.loginfo("To stop Turtlebot CTRL + C")

        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        r = rospy.Rate(10)

        move_cmd = Twist()

        move_cmd.linear.x = 1

        move_cmd.angular.z = 0

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)

            r.sleep()

    def shutdown(self):
        rospy.loginfo("Stop turtlebot")

        self.cmd_vel.publish(Twist())

        rospy.sleep(1)

if __name__ == "__main__":
    try:
        DriveForward()
    except:
        rospy.loginfo("Drive Forward node terminated!!!!!!!!!!!!")