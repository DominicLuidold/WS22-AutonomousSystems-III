import rospy
import queue
import tf
import numpy as np
import math
from token_inspector.srv import GimmeGoal
from geometry_msgs.msg import Point, Twist
from current_pos.msg import PoseTF, PoseInMap

MAX_TOKEN_DISTANCE = 0.05
FORWARD_SPEED = 0.22
MAX_ANGLE_DIFFERENCE_DEGREES = 0.5
MAX_ANGULAR_VELOCITY = 0.1

class Runner:
    def __init__(self):
        rospy.init_node('runner_client', log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo('Init runner_client node')
        self._isFinished = False
        self._hasGoal = False
        rospy.wait_for_service('give_goals_service')
        self._gimmeGoals = rospy.ServiceProxy('give_goals_service', GimmeGoal)
        self._currentToken = -1
        self._reachedGoal = False
        self._path = [] #[Point]

        #init frames for transformations
        self._mapFrame = 'map'
        self._robotFrame = 'base_footprint'

        #init listener publisher and subscriber
        self._tfListener = tf.TransformListener()

        self._movementPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        while not rospy.is_shutdown() or self._isFinished:
            #Get goal
            self._currentToken, x, y = self.get_goal()
            self._reachedGoal = False
            rospy.loginfo('Drive to point: %s | %s'%x%y)
            self._path = self.get_path(x,y)

            pointsqueue = queue.Queue(self._path)
            while not self._reachedGoal:
                currentPoint = pointsqueue.get()
                self.drive_towards_target(currentPoint)
                if pointsqueue.empty: #and robot is at right position
                    self._reachedGoal = True

    def drive_towards_target(self, target: Point): #target is of type Point
        self.turn_towards_target(target)
        self.drive_straight_to_target(target)

    def is_robot_on_target(self, target: Point):
        try:
            atTimeStamp = self._tfListener.getLatestCommonTime(self._mapFrame, self._robotFrame)
            pos, quad = self._tfListener.lookupTransform(self._mapFrame, self._robotFrame, atTimeStamp)
            currDistanceFromTaget = self.euklidean_distance(pos, target)

            if currDistanceFromTaget < MAX_TOKEN_DISTANCE:
                return True
        except tf.Exception:
            rospy.logwarn("PoseConversions: Transform between /odom and /map is not ready")
            return False

    def euklidean_distance(self, currPos, target: Point): #currpos = [x,y,z]
        x1 = target.x
        y1 = target.y
        x2 = currPos[0]
        y2 = currPos[1]
        return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

    def turn_towards_target(self, target: Point):
        current_pos = PoseTF()
        current_pos = self.get_current_position()
        current_pos_in_map = PoseInMap()
        current_pos_in_map = current_pos.mapPose
        
        angle_difference = self.get_angle_difference(current_pos_in_map.x, current_pos_in_map.y, current_pos_in_map.angle, target.x, target.y)
        max_angle_diff_radians = self.convert_to_radians(MAX_ANGLE_DIFFERENCE_DEGREES)
        while abs(angle_difference) > max_angle_diff_radians:
            #Calculate angular velocity
            constant_k_p = 1.0 #for smooth running
            angular_velocity = constant_k_p * angle_difference
            if abs(angular_velocity) > MAX_ANGULAR_VELOCITY:
                angular_velocity = MAX_ANGULAR_VELOCITY

            cmd_vel = Twist()
            cmd_vel.angular.z = angular_velocity
            self._movementPublisher.publish(cmd_vel)

            current_pos = self.get_current_position()
            current_pos_in_map = current_pos.mapPose
            
            angle_difference = self.get_angle_difference(current_pos_in_map.x, current_pos_in_map.y, current_pos_in_map.angle, target.x, target.y)
        return
    
    def convert_to_radians(self, degree):
        return degree * (math.pi / 180)
    
    def get_current_position(self):
        #use current_pos transform_position for receiving your position
        return rospy.wait_for_message('pose_tf', PoseTF)

        

    def drive_straight_to_target(self, target):
        while not self.is_robot_on_target(target):
            cmd_vel = Twist()
            cmd_vel.linear.x = FORWARD_SPEED 
            self._movementPublisher.publish(cmd_vel)

    def get_goal(self):
        rospy.loginfo('Get Goal')
        try:
            resp = self._gimme_goals(-4711 if self._currentToken == -1 else -1)
            return resp.id, resp.x, resp.y
        except rospy.ServiceException as e:
            print('Service call failed: %s'%e)

    def get_path(self):
        rospy.loginfo('Get Path')

    def get_angle_difference(current_x, current_y, current_heading, target_x, target_y):
        '''
        current_heading is in radians
        angle difference is in radians
        '''
        # Calculate the direction vector
        direction_vector = [target_x - current_x, target_y - current_y]

        # Calculate the angle between the direction vector and the x-axis
        target_angle = math.atan2(direction_vector[1], direction_vector[0])

        # Calculate the difference between the current heading and the target angle
        angle_difference = target_angle - current_heading

        # Normalize the angle difference to be within the range of -pi and pi
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))

        # Return the angle difference
        return angle_difference



def main():
    try:
        node = Runner()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()