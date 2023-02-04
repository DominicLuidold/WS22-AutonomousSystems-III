import rospy

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, MapMetaData

class AStar:
    def __init__(self):
        rospy.init_node('a_star', log_level=rospy.DEBUG, anonymous=True)
        self._mapInfo = MapMetaData()
        self._mapInfo = rospy.wait_for_message('map', OccupancyGrid).info

    #get costmap

    #
        
    def a_star_search(self, start, goal):
        #check if you're already there
        if start == goal:
            return [Point(start[0], start[1], 0)]

        

        return [Point]

def main():
    try:
        node = AStar()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
