import rospy
from collections.abc import Callable
from token_inspector.srv import GimmeGoal, GimmeGoalResponse
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback

class MoveBaseRunner:
    def __init__(self, goal_reached:Callable) -> None:
        self.goal_reached:Callable = goal_reached
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # wait for the action server to come up
        rospy.logdebug("Waiting for move_base action server...")
        self._move_base_client.wait_for_server()
        self._active = False
        

    def is_applicable(self, goal:GimmeGoalResponse):
        #state = self._move_base_client.get_state()
        #rospy.logerr(f'move base state {state}')
        #return state not in [GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.]
        return True

    def run(self, target:GimmeGoalResponse):
        if not self._active:
            goal:MoveBaseGoal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = target.x
            goal.target_pose.pose.position.y = target.y
            goal.target_pose.pose.orientation.w = 1.0
            self._move_base_client.send_goal(goal=goal, active_cb=self.active_cb, feedback_cb=self.feedback_cb, done_cb=self.goal_reached_cb)
        state = self._move_base_client.get_state()
        rospy.logerr(f'move base state {state}')

    def active_cb(self):
        rospy.logerr('active')

    def feedback_cb(self, feedback: MoveBaseFeedback):
        rospy.logerr('feedback with types %s' % type(feedback))
        feedback.base_position.pose.

    def goal_reached_cb(self, status, result):
        rospy.logerr('done with types %s %s' % type(status), type(result))
        rospy.logerr('done with types %s %s' % status, result)

    def stop(self):
        self._move_base_client.cancel_all_goals()
        rospy.logdebug(f'move_base_runner cancel all goals')