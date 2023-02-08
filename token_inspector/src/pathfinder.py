import rospy
import queue
import math
import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.srv import GetPlan, GetPlanResponse
from nav_msgs.msg import Path, OccupancyGrid
from current_pos.msg import PoseTF
from token_inspector.srv import GimmePathLength, GimmePath
from abc import ABCMeta, abstractmethod
from heapq import heappush, heappop, heapify
from collections.abc import Sequence

TOLERANCE_TO_TARGET = 0.05
GET_EVERY_NTH = 10

def add(a, b):
    return a[0] + b[0], a[1] + b[1]

N = (0, 1)
S = (0, -1)
E = (1, 0)
W = (-1, 0)
NW = add(N, W)
NE = add(N, E)
SW = add(S, W)
SE = add(S, E)
cardinals = [N, NE, E, SE, S, SW, W, NW]

class Pathfinder:
    def __init__(self):
        rospy.init_node('pathfinder', log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo('Pathfinder initialized!')

        self._providePathLenghtService = rospy.Service('provide_path_length_service', GimmePathLength, self.handle_path_length_euklid)
        #self._providePathLenghtService = rospy.Service('provide_path_length_service', GimmePathLength, self.handle_path_length)
        self._providePathService = rospy.Service('provide_path_service', GimmePath, self.handle_path)
        rospy.wait_for_service('/move_base/make_plan')
        self._makePlan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.neighbourThreshold = 20
        rospy.loginfo('Initialized service')
        self.path_pub = rospy.Publisher('astar_path', Path, queue_size=10)
        #self.test()

    def get_path_to_target(self, target: Pose):
        rospy.logdebug('Initializing PoseStampeds')
        start:PoseTF = self.get_current_position()
        #start_stamped = PoseStamped()
        #start_stamped.pose = self.convert_posetf_to_pose(start)
        #start_stamped.header.frame_id = 'map'

        #target_stamped = PoseStamped()
        #target_stamped.pose = target
        #target_stamped.header.frame_id = 'map'
        rospy.loginfo('Get path')

        start_tuple = (start.mapPose.x, start.mapPose.y)
        target_tuple = (target.position.x, target.position.y)

        #response = self._makePlan(start_stamped, target_stamped, TOLERANCE_TO_TARGET)
        response = self.get_a_star_path(start_tuple, target_tuple, TOLERANCE_TO_TARGET)
        rospy.loginfo('Received path')
        return response.plan

    def get_a_star_path(self, start, target, tolerance: int) -> Path:
        costmapMsg = rospy.wait_for_message("/move_base/global_costmap/costmap", OccupancyGrid)
        costmap = np.array(costmapMsg.data, dtype=np.int8).reshape(costmapMsg.info.height, costmapMsg.info.width)
        rospy.logerr('a_star: costmap received')
        np.savetxt('/killerrobot/path.txt', costmap)
        path = list(find_path(
            start=start,
            goal=target,
            neighbors_fnct=lambda cell: self.neighborsForCell(costmap, cell),
            reversePath=False,
            heuristic_cost_estimate_fnct=self.euclideanDistanceBetweenCells,
            distance_between_fnct= lambda a, b: self.costBetween(a, b, costmap, costmapMsg.info),
            is_goal_reached_fnct=lambda a, b: abs(a[0] - b[0]) < tolerance and abs(a[1] - b[1]) < tolerance))
        rospy.logwarn(path)
        return path


    def neighborsForCell(self, cm, cell):
        l = []
        for direction in cardinals:
            target = add(cell, direction)
            try:
                if cm[target[0]][target[1]] < self.neighbourThreshold:
                    l.append(target)
            except IndexError:
                pass
                # rospy.logwarn("PlanPath: Costmap index access outside of bounds: {}".format(target))
        return l

    @staticmethod
    def euclideanDistanceBetweenCells(a, b):
        x = np.power(b[0] - a[0], 2)
        y = np.power(b[1] - a[1], 2)
        return np.sqrt(np.sum([x, y]))

    def costBetween(self, a, b, costmap, info):
        cost = 1 + max(0, self.extractCostAt(costmap, info, b[0], b[1]))
        return cost

    @staticmethod
    def extractCostAt(costmap, info, x, y):
        if -1 < x < info.width and -1 < y < info.height:
            # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
            # first index is the row, second index the column
            return costmap[y][x]
        else:
            IndexError(
                "Coordinates out of gridmap, x: {}, y: {} must be in between: [0, {}], [0, {}]".format(
                    x, y, info.height, info.width))
            return None


    def convert_posetf_to_pose(self, posetf: PoseTF):
        pose = Pose()
        pose.position.x = posetf.mapPose.x
        pose.position.y = posetf.mapPose.y
        pose.position.z = 0.0
        pose.orientation = posetf.originalPose.orientation
        return pose


    def get_current_position(self):
        #use current_pos transform_position for receiving your position
        pose = rospy.wait_for_message('pose_tf', PoseTF)
        return pose

    def handle_path_length(self, req):
        rospy.logwarn('Received request for pathlength of %s: %s|%s'%(str(req.id_token),str(req.x),str(req.y)))
        rospy.logwarn(req)
        target = Pose()
        target.position.x = req.x
        target.position.y = req.y
        target.orientation.w = 1.0
        resp = self.get_path_to_target(target)
        #rospy.loginfo(resp)

        distance = self.calculate_path_length(resp)
        rospy.loginfo('Pathfinder: Distance to target %i is: %f'%(req.id_token,distance))
        return distance

    def handle_path_length_euklid(self, req):
        rospy.logwarn('Received request for pathlength of %s: %s|%s'%(str(req.id_token),str(req.x),str(req.y)))
        rospy.logwarn(req)
        target = Pose()
        target.position.x = req.x
        target.position.y = req.y
        target.orientation.w = 1.0

        #resp = self.get_path_to_target(target)
        #rospy.loginfo(resp)

        distance = self.calculate_path_length_euklid(target)
        rospy.loginfo('Pathfinder: Distance to target %i is: %f'%(req.id_token,distance))
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

    def calculate_path_length_euklid(self, target: Pose):
        length = 0.0        
        startTF:PoseTF = self.get_current_position()
        start = Pose()
        start.position.x = startTF.mapPose.x
        start.position.y = startTF.mapPose.y

        length = self.euklidean_distance(start, target)

        return length

    def euklidean_distance(self, start: Pose, target: Pose):
        x1 = target.position.x
        y1 = target.position.y
        x2 = start.position.x
        y2 = start.position.y
        return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

    def handle_path(self,req):
        rospy.loginfo('Received request for path of %s: %s|%s'%(str(req.id_token),str(req.x),str(req.y)))
        target = Pose()
        target.position.x = req.x
        target.position.y = req.y
        target.orientation.w = 1.0
        resp:Path = self.get_path_to_target(target)
        # TODO: adapt path to target for nicer driving here
        counter = 0
        poses = []
        for posestamped in resp.poses:
            if counter >= GET_EVERY_NTH:
                poses.append(posestamped)
                counter = 0
            counter += 1
        resp.poses = poses
        rospy.logwarn('Pathfinder initialized poses')

        return resp

    def test(self):
        rospy.loginfo('Test started')
        target = Pose()
        target.position.x = 0.734
        target.position.y = -0.535
        target.orientation.w = 1.0
        resp = self.get_path_to_target(target)
        #rospy.loginfo(resp)

        distance = self.calculate_path_length(resp.plan)

        rospy.loginfo('Pathfinder: distance to target is: %s'%distance)

Infinite = float('inf')
class AStar:
    __metaclass__ = ABCMeta
    __slots__ = ()

    class SearchNode:
        __slots__ = ('data', 'gscore', 'fscore',
                     'closed', 'came_from', 'out_openset')

        def __init__(self, data, gscore=Infinite, fscore=Infinite):
            self.data = data
            self.gscore = gscore
            self.fscore = fscore
            self.closed = False
            self.out_openset = True
            self.came_from = None

        def __lt__(self, b):
            return self.fscore < b.fscore

    class SearchNodeDict(dict):

        def __missing__(self, k):
            v = AStar.SearchNode(k)
            self.__setitem__(k, v)
            return v

    @abstractmethod
    def heuristic_cost_estimate(self, current, goal):
        """Computes the estimated (rough) distance between a node and the goal, this method must be implemented in a subclass. The second parameter is always the goal."""
        raise NotImplementedError

    @abstractmethod
    def distance_between(self, n1, n2):
        """Gives the real distance between two adjacent nodes n1 and n2 (i.e n2 belongs to the list of n1's neighbors).
           n2 is guaranteed to belong to the list returned by the call to neighbors(n1).
           This method must be implemented in a subclass."""
        raise NotImplementedError

    @abstractmethod
    def neighbors(self, node):
        """For a given node, returns (or yields) the list of its neighbors. this method must be implemented in a subclass"""
        raise NotImplementedError

    def is_goal_reached(self, current, goal):
        """ returns true when we can consider that 'current' is the goal"""
        return current == goal

    def reconstruct_path(self, last, reversePath=False):
        def _gen():
            current = last
            while current:
                yield current.data
                current = current.came_from

        if reversePath:
            return _gen()
        else:
            return reversed(list(_gen()))

    def astar(self, start, goal, reversePath=False):
        rospy.logwarn('starting astar')
        if self.is_goal_reached(start, goal):
            return [start]
        searchNodes = AStar.SearchNodeDict()
        startNode = searchNodes[start] = AStar.SearchNode(
            start, gscore=.0, fscore=self.heuristic_cost_estimate(start, goal))
        openSet = []
        heappush(openSet, startNode)
        while openSet:
            current = heappop(openSet)
            if self.is_goal_reached(current.data, goal):
                return self.reconstruct_path(current, reversePath)
            current.out_openset = True
            current.closed = True
            for neighbor in map(lambda n: searchNodes[n], self.neighbors(current.data)):
                if neighbor.closed:
                    continue
                tentative_gscore = current.gscore + \
                                   self.distance_between(current.data, neighbor.data)
                if tentative_gscore >= neighbor.gscore:
                    continue
                neighbor.came_from = current
                neighbor.gscore = tentative_gscore
                neighbor.fscore = tentative_gscore + \
                                  self.heuristic_cost_estimate(neighbor.data, goal)
                if neighbor.out_openset:
                    neighbor.out_openset = False
                    heappush(openSet, neighbor)
                else:
                    # re-add the node in order to re-sort the heap
                    openSet.remove(neighbor)
                    heappush(openSet, neighbor)
        return None


def find_path(
        start,
        goal,
        neighbors_fnct,
        reversePath=False,
        heuristic_cost_estimate_fnct=lambda a, b: Infinite,
        distance_between_fnct=lambda a, b: 1.0,
        is_goal_reached_fnct=lambda a, b: a == b):
    """ execute A-Star algorithm to find path """

    class FindPath(AStar):

        def heuristic_cost_estimate(self, current, goal):
            return heuristic_cost_estimate_fnct(current, goal)

        def distance_between(self, n1, n2):
            return distance_between_fnct(n1, n2)

        def neighbors(self, node):
            return neighbors_fnct(node)

        def is_goal_reached(self, current, goal):
            return is_goal_reached_fnct(current, goal)

    return FindPath().astar(start, goal, reversePath)


def main():
    try:
        node = Pathfinder()
        #node.test()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
