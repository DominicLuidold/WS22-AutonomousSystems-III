import rospy
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.srv import GetPlan, Path
from current_pos.msg import PoseTF

TOLERANCE_TO_TARGET = 0.05

class Pathfinder:
    def __init__(self):
        rospy.init_node('pathfinder', log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo('Pathfinder initialized!')


        rospy.wait_for_service('make_plan')
        self._makePlan = rospy.ServiceProxy('make_plan', GetPlan)

    def get_path_to_target(self, target: Pose):
        start = self.get_current_position()
        start_stamped = PoseStamped()
        start_stamped.pose = self.convert_posetf_to_pose(start)
        start_stamped.header.frame_id = 'map'

        target_stamped = PoseStamped()
        target_stamped.pose = target
        target_stamped.header.frame_id = 'map'

        response = self._makePlan.call(start_stamped, target_stamped, TOLERANCE_TO_TARGET)
        rospy.loginfo('Received path')
        return response


    def convert_posetf_to_pose(self, posetf: PoseTF):
        pose = Pose()
        pose.x = posetf.mapPose.x
        pose.y = posetf.mapPose.y
        pose.z = 0.0
        pose.orientation = posetf.originalPose.orientation
        return pose


    def get_current_position(self):
        #use current_pos transform_position for receiving your position
        return rospy.wait_for_message('pose_tf', PoseTF)

        

def main():
    try:
        node = Pathfinder()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
