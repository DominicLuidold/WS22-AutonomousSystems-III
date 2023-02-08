import rospy
from runners.custom_runner import CustomRunner
from runners.move_base_runner import MoveBaseRunner
from runners.raspicam_runner import RaspicamRunner
from runners.wall_follower_runner import WallFollowerRunner
from localizers.wall_localizer import WallLocalizer
from token_inspector.srv import GimmeGoal, GimmeGoalResponse

DEBUG = rospy.get_param('debug')
log_level = rospy.DEBUG if DEBUG else rospy.INFO

class TokenInspector:
    """
    Node that localizes itself in labyrinth, and visits goals within one after another
    """

    def __init__(self) -> None:
        rospy.init_node('inspector', log_level=log_level, anonymous=True)
        rospy.loginfo('Init Inspector')
        self._localizer = WallLocalizer()
        rospy.loginfo('token_inspector: wait for give goals service')
        rospy.wait_for_service('give_goals_service')
        self._gimmeGoals = rospy.ServiceProxy('give_goals_service', GimmeGoal)
        rospy.logerr('token_inspector: give goals service up')
        self._movebaserunner = MoveBaseRunner(self.goal_reached)
        self._runners = [RaspicamRunner(), self._movebaserunner, CustomRunner(), WallFollowerRunner()]
        rospy.loginfo('Init Inspector Done')
        rospy.on_shutdown(self._stop)

    def keep_movin(self):
        while not rospy.is_shutdown():
            #self._localizer.localize() # do this until localization is complete
            self._next_goal:GimmeGoalResponse = self._request_goal()
            while self._next_goal:
                for r in self._runners:
                    if r.is_applicable(self._next_goal):
                        r.run(self._next_goal)
                        break
                rospy.Rate(10).sleep()
            rospy.loginfo('Inspector Finished')
            self._stop()
            break


    def _request_goal(self, found_token:bool=False) -> GimmeGoalResponse:
        """ 
        Requests a new goal via the GimmeGoals service.
        Initally no token is found, for all the other calls a found_token is assumed
        """
        try:
            goal:GimmeGoalResponse = self._gimmeGoals(-1 if found_token else -4711)
            if goal.id == -1: return None # -1 is code for: no more goals left
            rospy.logerr(f'inspector: received goal {goal.id}')
            return goal
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s'%e)
        return None


    def goal_reached(self):
        """ Called from the runners as soon they reach a goal """
        self._next_goal:GimmeGoalResponse = self._request_goal(found_token=True)

    def _stop(self):
        self._movebaserunner.stop()

def main():
    try:
        ti = TokenInspector()
        ti.keep_movin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()