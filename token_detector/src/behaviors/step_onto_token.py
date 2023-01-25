import rospy
from camera_integrators.pixycam_detector import PixycamDetector

class StepOntoToken:
  """ Use pixycam to position above token """

  def __init__(self, killerrobot):
    self.__pixycam_detector = PixycamDetector()
    self.position = (0,0)
    self.__killerrobot = killerrobot

  def isApplicable(self):
    return self.__pixycam_detector.is_detecting_token or \
           self.__pixycam_detector.consecutive_non_detections < 5
             # there are always non-detections inbetween even if it should detect a token (only god knows why)

  def execute(self):
    """ position above token """
    rospy.loginfo('behavior: step onto token')
    self.__killerrobot.move(self.__killerrobot.max_speed / 7, 0)
    self.__killerrobot._isovertoken = True