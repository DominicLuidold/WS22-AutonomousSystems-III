#!/usr/bin/env python
import rospy
import math
from pixy_msgs.msg import PixyData

class PixycamDetector:
    def __init__(self) -> None:
        self.is_detecting_token = False
        self.consecutive_non_detections = math.inf # Counter for letting intermediate blocks that don't recognize the tag fly through
        rospy.Subscriber('/my_pixy/block_data', PixyData, self.process)

    def process(self, data: PixyData) -> None:
        if self.found_tag(data):
            self.is_detecting_token = True
            self.consecutive_non_detections = 0
        else:
            self.is_detecting_token = False
            self.consecutive_non_detections += 1
    
    
    def found_tag(self, data: PixyData) -> bool:
        return data.header.stamp.secs != 0
