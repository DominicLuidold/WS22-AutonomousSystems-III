import rospy
import queue
import tf
import math
from token_inspector.srv import GimmeGoal
from geometry_msgs.msg import Point, Twist

MAX_TOKEN_DISTANCE = 0.05
FORWARD_SPEED = 0.22

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

    def turn_towards_target(self, target):
        
        return

    def drive_straight_to_target(self, target):
        while not self.is_robot_on_target(target):
            move = Twist()
            move.linear.x = FORWARD_SPEED 
            self._movementPublisher.publish(move)

    def get_goal(self):
        rospy.loginfo('Get Goal')
        try:
            resp = self._gimme_goals(-4711 if self._currentToken == -1 else -1)
            return resp.id, resp.x, resp.y
        except rospy.ServiceException as e:
            print('Service call failed: %s'%e)

    def get_path(self):
        rospy.loginfo('Get Path')





def main():
    try:
        node = Runner()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()