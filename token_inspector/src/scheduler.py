import rospy
import json
from os import path
from operator import itemgetter
from token_inspector.srv import GimmeGoals, GimmeGoalsResponse


FINISHED = -1
INITIAL = -4711
FILENAME = '/killerrobot/token_positions.json'
IS_COLLABORATIVE = False

class Scheduler:
    def __init__(self):
        rospy.init_node('scheduler_server', log_level=rospy.DEBUG, anonymous=True)
        rospy.loginfo('Init Scheduler Server')
        self._giveGoalsService = ('give_goals_service', GimmeGoals, self.handle_goal_scheduling)
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
            self._tokenpositions[pos['name']] = {'found': pos['found'], 'x': pos['map_position']['x'], 'y': pos['map_position']['y']}

    def handle_goal_scheduling(self, req):
        rospy.loginfo('Received request: %s'%req.id_found)

        if req.id_found == FINISHED:
            #write found in tag name
            rospy.loginfo('Write True to Found')
            self._tokenpositions[self._currentTag]['found'] = True

        rospy.loginfo('Look for shortest A* path')
        token_distances = [] #(token, distance)
        for token in self._tokenpositions.keys():
            if not self._tokenpositions[token]['found']:
                rospy.loginfo('Ask for distance to token: %s'%token)

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
            
            resp = GimmeGoalsResponse(self._tokenpositions[goal_token], self._tokenpositions[goal_token]['x'], self._tokenpositions[goal_token]['y'])
            return resp
        else:
            rospy.loginfo('All Tokens have been seen')
            return GimmeGoalsResponse(-1, 0.0, 0.0)


def main():
    try:
        node = Scheduler()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()