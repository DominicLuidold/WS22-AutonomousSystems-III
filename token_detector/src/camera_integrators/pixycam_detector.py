import rospy
import math
from pixy_msgs.msg import PixyData

class PixycamDetector:

    def __init__(self, rospy_rate: int):
        self.is_detecting_token = False
        self.consecutive_non_detections = math.inf # Counter for letting intermediate blocks that don't recognize the tag fly through
        self.__rate = rospy.Rate(rospy_rate)
        rospy.Subscriber('/my_pixy/block_data', PixyData, self.process)

    def process(self, data):
        if self.found_tag(data):
            self.is_detecting_token = True
            self.consecutive_non_detections = 0
        else:
            self.is_detecting_token = False
            self.consecutive_non_detections += 1
    
    
    def found_tag(self, data) -> bool:
        return data.header.stamp.secs != 0