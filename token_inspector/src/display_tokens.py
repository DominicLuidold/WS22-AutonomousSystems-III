import rospy
import json
from os import path
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

class DisplayTokens:
    def __init__(self):
        self._filename = '/home/dominic/catkin_ws/positions.json'

        self._pubPoseOfTokens = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

        rospy.init_node('display_tokens',log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo('Display_tokens node initialized!')
        self._tokendicts = []

        if path.isfile(self._filename):
            with open(self._filename) as fp:
                self._tokendicts = json.load(fp)

        self._tokenmarkers = self.generate_markers_from_targetdicts()
        self.run()

    def generate_markers_from_targetdicts(self):
        tokenmarkers = []
        for token in self._tokendicts:
            pose = Pose()
            point = Point(token['map_position']['x'], token['map_position']['y'], 1)
            pose.position = point
            marker = Marker()
            marker.header.frame_id = "/my_frame%s"%token["name"]
            marker.pose = pose
            marker.scale = 0.1
            marker.header.stamp = rospy.Time.now()
            marker.id = token['name']
            marker.ns = "tokens"
            color = ColorRGBA()
            if token['found']: #green
                color.r = 124
                color.g = 252
                color.b = 0
            else: #orange
                color.r = 1.0
                color.g = 0.0
                color.b = 1.0
            color.a = 1.0
            marker.color = color
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            #marker.lifetime = rospy.Duration(1000)
            tokenmarkers.append(marker)
        return tokenmarkers

    def run(self):
        #while not rospy.is_shutdown():
            markers = MarkerArray()
            markers.markers = self._tokenmarkers
            rospy.loginfo('Sent tokens')

            self._pubPoseOfTokens.publish(markers)

            






def main():
    try:
        node = DisplayTokens()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()