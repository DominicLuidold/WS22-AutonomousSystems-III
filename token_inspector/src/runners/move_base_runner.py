import rospy
from collections.abc import Callable
from token_inspector.srv import GimmeGoal, GimmeGoalResponse
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
import time
from helpers.helper import eucl_distance

MAX_TOKEN_DISTANCE = 0.05

class MoveBaseRunner:
    def __init__(self, goal_reached:Callable) -> None:
        self._invoke_inspector_goal_reached:Callable = goal_reached
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # wait for the action server to come up
        rospy.logdebug("Waiting for move_base action server...")
        self._move_base_client.wait_for_server()
        rospy.logerr('status' + str(self._move_base_client.get_state()))
        self._active = False
        self._failure_time = 0 # very looong ago
        self._state: GoalStatus = 0
        self._goal: GimmeGoalResponse = None
        

    def is_applicable(self, goal:GimmeGoalResponse):
        return self._active or time.time() - self._failure_time > 10

    def run(self, target:GimmeGoalResponse):
        if not self._active:
            self._goal = target
            goal:MoveBaseGoal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = target.x
            goal.target_pose.pose.position.y = target.y
            goal.target_pose.pose.orientation.w = 1.0
            self._move_base_client.send_goal(goal=goal, active_cb=self._active_cb, feedback_cb=self._feedback_cb, done_cb=self._done_cb)

    def _active_cb(self):
        self._active = True
        rospy.loginfo('movebase active')

    def _feedback_cb(self, feedback: MoveBaseFeedback):
        new_state = self._move_base_client.get_state()
        if new_state != self._state:
            rospy.logdebug(f'move base new state {new_state}')
            self._state = new_state
        pos= feedback.base_position.pose.position
        if eucl_distance(pos.x, pos.y, self._goal.x, self._goal.y) <= MAX_TOKEN_DISTANCE:
            rospy.logerr(f'movebaserunner: Goal reached by feedback distance')
            self._goal_reached()

    def _done_cb(self, state: GoalStatus, result:MoveBaseResult):
        rospy.logerr(f'move base done with status {state}')
        if result:
            rospy.logerr(f'move base done with result {result}')
            rospy.logerr('feedback with types %s' % type(result))
        if state == GoalStatus.SUCCEEDED:
            self._goal_reached()
        elif state == GoalStatus.ABORTED:
            rospy.logerr('MoveBaseRunner: Aborted!')
            self.stop()
            self._failure_time = time.time()

    def _goal_reached(self):
        rospy.logerr(f'MoveBaseRunner: Token {self._goal.id} reached!')
        self._invoke_inspector_goal_reached()
        self._active = False

    def stop(self):
        self._move_base_client.cancel_all_goals()
        self._goal = None
        rospy.logdebug(f'move_base_runner cancel all goals')