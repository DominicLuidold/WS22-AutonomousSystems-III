#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from pixy_msgs.msg import PixyData

class TokenRecognizer:
    def __init__(self):
        # Init
        rospy.loginfo("Initialized!")
        
        #Counter for letting intermediate blocks that don't recognize the tag fly through
        self._counter = 0
        # Publish
        self._rate = rospy.Rate(100)
        self._cmd_sub = rospy.Subscriber('/my_pixy/block_data', PixyData, self.process)



    def process(self, data):
        #rospy.loginfo("Search token")
        
        if self.found_tag(data):
            print("Found tag")
            self._counter = 0
        else:
            self._counter += 1
            #Here the counter is checked
            if self._counter > 5:
                print("Nothing to see here")

        self._rate.sleep()
    
    
    def found_tag(self, data) -> bool:
        #rospy.loginfo("This: " + str(data))
        if data.header.stamp.secs == 0:
            return False
        else:
            return True   


def main():
    # Init
    rospy.init_node('pixy_token_detector')

    # Start
    try:
        TokenRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
