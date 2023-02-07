import rospy
from runners.custom_runner import CustomRunner
from runners.move_base_runner import MoveBaseRunner
from runners.raspicam_runner import RaspicamRunner
from runners.wall_follower_runner import WallFollowerRunner
from localizers.wall_localizer import WallLocalizer
from token_inspector.srv import GimmeGoal, GimmeGoalResponse

class TokenInspector:
    """
    Node that localizes itself in labyrinth, and visits goals within one after another
    """

    def __init__(self) -> None:
        rospy.init_node('inspector', log_level=rospy.DEBUG, anonymous=True)
        rospy.logerr('Init Inspector')
        #self._localizer = WallLocalizer()
        rospy.logerr('token_inspector: wait for give goals service')
        rospy.wait_for_service('give_goals_service')
        self._gimmeGoals = rospy.ServiceProxy('give_goals_service', GimmeGoal)
        rospy.logerr('token_inspector: give goals service up')
        self._runners = [RaspicamRunner(), MoveBaseRunner(self.goal_reached), CustomRunner(), WallFollowerRunner()]
        rospy.loginfo('Init Inspector Done')

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
            rospy.loginfo('Finished')


    def _request_goal(self, found_token:bool=False) -> GimmeGoalResponse:
        """ 
        Requests a new goal via the GimmeGoals service.
        Initally no token is found, for all the other calls a found_token is assumed
        """
        try:
            goal:GimmeGoalResponse = self._gimmeGoals(-1 if found_token else -4711)
            if goal.id == -1: return None # -1 is code for: no more goals left
            return goal
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s'%e)
        return None


    def goal_reached(self):
        """ Called from the runners as soon they reach a goal """
        self._next_goal:GimmeGoalResponse = self._request_goal(found_token=True)



def main():
    try:
        ti = TokenInspector()
        ti.keep_movin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()