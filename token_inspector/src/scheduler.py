import rospy
import json
from os import path
from operator import itemgetter
from token_inspector.srv import GimmeGoal, GimmePathLength, GimmeGoalResponse


FOUND_TAG = -1
INITIAL = -4711
FILENAME = '/killerrobot/token_positions.json'
IS_COLLABORATIVE = False

class Scheduler:
    def __init__(self):
        rospy.init_node('scheduler_server', log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo('Init Scheduler Server')
        self._giveGoalsService = rospy.Service('give_goals_service', GimmeGoal, self.handle_goal_scheduling)
        
        rospy.wait_for_service('provide_path_length_service')
        rospy.loginfo('Found provide_path_length_service')
        self._gimmePathLength = rospy.ServiceProxy('provide_path_length_service', GimmePathLength)
        self._currentTag = -1
        self._tokenpositions = {}

        self.load_file()

        rospy.spin()
    
    def load_file(self):
        if path.isfile(FILENAME):
            with open(FILENAME) as fp:
                tokenpositions = json.load(fp)
        if len(tokenpositions) <= 0:
            rospy.logerr('The positions file is empty!')
        for pos in tokenpositions:
            self._tokenpositions[pos['name']] = {'name': pos['name'], 'found': pos['found'], 'x': pos['map_position']['x'], 'y': pos['map_position']['y']}

    def handle_goal_scheduling(self, req):
        rospy.loginfo('Received request: %s'%req.id_found)

        if req.id_found == FOUND_TAG:
            #write found in tag name
            rospy.loginfo('Write True to Found')
            self._tokenpositions[self._currentTag]['found'] = True

        rospy.loginfo('Look for shortest A* path')
        token_distances = [] #(token, distance)

        for id, token in self._tokenpositions.items():
            if not token['found']:
                rospy.loginfo('Ask for distance to token: %s'%token['name'])
                token_distances.append(self.get_path_length(token))

                #TODO: Add communication with A*, tokendistances.append(token, distance)

        #Order token_distances by distances
        token_distances = sorted(token_distances, key=itemgetter(1))

        goal_token = None
        #Check if there are still unseen tokens
        if len(token_distances) > 0:
            if IS_COLLABORATIVE:
                rospy.loginfo('Ask GlobalScheduler if tag is ok')
                for token in token_distances:
                    rospy.loginfo('Ask for tag: %s'%token[0])
                    #TODO: Add global Scheduler
            else:
                rospy.loginfo('Get first from ordered token_distances')
                goal_token = token_distances[0]
            
            resp = GimmeGoalResponse(goal_token[0], self._tokenpositions[goal_token[0]]['x'], self._tokenpositions[goal_token[0]]['y'])
            return resp
        else:
            rospy.loginfo('All Tokens have been seen')
            return GimmeGoalResponse(-1, 0.0, 0.0)

    def get_path_length(self, token):
        rospy.loginfo('Get pathlength of token %s'%token['name'])
        try:
            resp = self._gimmePathLength(token['name'], token['x'], token['y'])
            rospy.loginfo('Path length to token %s is: %s'%(token['name'],resp.path_length))
            return token['name'], resp.path_length
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s'%e)

    def test_path_length(self):
        rospy.loginfo('Scheduler: started test for path length')
        tokenid = 0
        x = 0.136
        y = -0.535
        token = {'name': tokenid, 'x': x, 'y': y}

        distance = self.get_path_length(token)

        rospy.loginfo('Scheduler: distance to target is: %s'%str(distance))


def main():
    try:
        node = Scheduler()
        #node.test_path_length()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()