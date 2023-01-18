import rospy
from camera_integrators.pixycam_detector import PixycamDetector

class StepOntoToken:
  """ Use pixycam to position above token """

  def __init__(self):
    self.__pixycam_detector = PixycamDetector()
    self.position = (0,0)

  def isApplicable(self, tokens):
    print(self.__pixycam_detector.is_detecting_token, self.__pixycam_detector.consecutive_non_detections)
    return self.__pixycam_detector.is_detecting_token or \
           self.__pixycam_detector.consecutive_non_detections < 5
             # there are always non-detections inbetween even if it should detect a token (only god knows why)

  def execute(self, killerrobot, tokens):
    """ position above token """
    rospy.loginfo('behavior: step onto token')
    killerrobot.move(killerrobot.max_speed / 7, 0)