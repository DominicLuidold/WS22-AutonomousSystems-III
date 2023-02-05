#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goal():
    # initialize node
    rospy.init_node('send_goal')

    # create a client to send the goal to move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # wait for the action server to come up
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()

    # create the goal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = 0.136
    goal.target_pose.pose.position.y = -0.535
    goal.target_pose.pose.orientation.w = 1.0

    # send the goal
    rospy.loginfo("Sending goal...")
    client.send_goal(goal)

    # wait for the goal to complete
    client.wait_for_result()
    rospy.loginfo("Goal reached!")
    rospy.spin()

if __name__ == '__main__':
    try:
        send_goal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Goal sending interrupted.")

