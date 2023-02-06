import rospy
import queue
import math
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.srv import GetPlan, GetPlanResponse
from nav_msgs.msg import Path
from current_pos.msg import PoseTF
from token_inspector.srv import GimmePathLength

TOLERANCE_TO_TARGET = 0.05

class Pathfinder:
    def __init__(self):
        rospy.init_node('pathfinder', log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo('Pathfinder initialized!')

        self._providePathLenghtService = rospy.Service('provide_path_length_service', GimmePathLength, self.handle_path_length)
        rospy.wait_for_service('/move_base/make_plan')
        self._makePlan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        rospy.loginfo('Initialized service')

        self.test()

    def get_path_to_target(self, target: Pose):
        rospy.loginfo('Initializing PoseStampeds')
        start = self.get_current_position()
        start_stamped = PoseStamped()
        start_stamped.pose = self.convert_posetf_to_pose(start)
        start_stamped.header.frame_id = 'map'

        target_stamped = PoseStamped()
        target_stamped.pose = target
        target_stamped.header.frame_id = 'map'
        rospy.loginfo('Get path')

        response = self._makePlan(start_stamped, target_stamped, TOLERANCE_TO_TARGET)
        rospy.loginfo('Received path')
        return response


    def convert_posetf_to_pose(self, posetf: PoseTF):
        pose = Pose()
        pose.position.x = posetf.mapPose.x
        pose.position.y = posetf.mapPose.y
        pose.position.z = 0.0
        pose.orientation = posetf.originalPose.orientation
        return pose


    def get_current_position(self):
        #use current_pos transform_position for receiving your position
        return rospy.wait_for_message('pose_tf', PoseTF)

    def handle_path_length(self, req):
        rospy.loginfo('Received request from %s: %s|%s'%req.id_token%req.x%req.y)

        target = Pose()
        target.position.x = req.x
        target.position.y = req.y
        target.orientation.w = 1.0
        resp = self.get_path_to_target(target)
        rospy.loginfo(resp)

        distance = self.calculate_path_length(resp.plan)
        rospy.loginfo('Distance to target %s is: %s'%req.id_token%distance)
        return distance
        

    def calculate_path_length(self, path: Path):
        length = 0.0        
        posequeue = queue.Queue()
        for posestamped in path.poses:
            posequeue.put(posestamped.pose)

        if posequeue.empty():
            return 0.0
        
        prev_pose = posequeue.get()
        
        while not posequeue.empty():
            curr_pose = posequeue.get()
            
            length += self.euklidean_distance(prev_pose, curr_pose)
            prev_pose = curr_pose

        return length

    def euklidean_distance(self, start: Pose, target: Pose):
        x1 = target.position.x
        y1 = target.position.y
        x2 = start.position.x
        y2 = start.position.y
        return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))







    def test(self):
        rospy.loginfo('Test started')
        target = Pose()
        target.position.x = 0.734
        target.position.y = -0.535
        target.orientation.w = 1.0
        resp = self.get_path_to_target(target)
        rospy.loginfo(resp)

        distance = self.calculate_path_length(resp.plan)

        rospy.loginfo('Distance to target is: %s'%distance)

def main():
    try:
        node = Pathfinder()
        #node.test()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
