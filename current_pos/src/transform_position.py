#! /usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from current_pos.msg import PoseInMap, PoseTF

DEBUG = rospy.get_param('debug')

class PoseConversions:

    def __init__(self):
        log_level = rospy.DEBUG if DEBUG else rospy.INFO
        rospy.init_node('robot2map_conversion', anonymous=True, log_level=log_level)
        rospy.loginfo("PoseConversions: Startup")
        self._mapFrame = "map"
        self._robotFrame = "base_footprint"
        self._mapInfo = MapMetaData()
        self._mapInfo = rospy.wait_for_message("map", OccupancyGrid).info
        self._tfListener = tf.TransformListener()
        self._pubPoseInMap = rospy.Publisher("pose_tf", PoseTF, queue_size=10)
        self._subRobotPose = rospy.Subscriber('odom', Odometry, self.sub_next_pose)
        self._rate = rospy.Rate(2)
        rospy.spin()


    def sub_next_pose(self, odom):
        """
        Callback of pose subscriber.
        Converts the current position of the robot in the map.
        """
        try:
            atTimeStamp = self._tfListener.getLatestCommonTime(self._mapFrame, self._robotFrame)
            pos, quad = self._tfListener.lookupTransform(self._mapFrame, self._robotFrame, atTimeStamp)
        
            #resolution = self._mapInfo.resolution
            mapPose = PoseInMap()
            mapPose.x = pos[0] #(pos[0] - self._mapInfo.origin.position.x) / resolution
            mapPose.y = pos[1] #(pos[1] - self._mapInfo.origin.position.y) / resolution
            mapPose.angle = tf.transformations.euler_from_quaternion(quad)[2]

            msg = PoseTF()
            msg.header.seq = 0
            msg.header.stamp = rospy.Time.now()
            msg.originalPose = odom.pose.pose
            msg.mapPose = mapPose
            self._pubPoseInMap.publish(msg)
            rospy.logdebug("PoseConversions: Next Pose -> {}".format(mapPose))
        except tf.Exception:
            rospy.logwarn("PoseConversions: Transform between /odom and /map is not ready")


def main():
    try:
        node = PoseConversions()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()