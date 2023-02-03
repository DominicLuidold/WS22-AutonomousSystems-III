import rospy
from token_inspector.srv import GimmeGoal
import queue

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

    def drive_towards_target(self, target): #target is of type Point
        while not self.is_robot_on_target(target):
            self.turn_towards_target(target)
            self.drive_straight_to_target(target)

    def is_robot_on_target(self, target):
        return False

    def turn_towards_target(self, target):
        return

    def drive_straight_to_target(self, target):
        return

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