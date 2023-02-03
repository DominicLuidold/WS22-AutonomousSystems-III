#!/usr/bin/env python
import rospy
from camera_integrators.pixycam_detector import PixycamDetector

class StepOntoToken:
  """ Use pixycam to position above token """

  def __init__(self, killerrobot):
    self._pixycam_detector = PixycamDetector()
    self.position = (0,0)
    self._killerrobot = killerrobot

  def isApplicable(self) -> bool:
    return self._pixycam_detector.is_detecting_token or \
           self._pixycam_detector.consecutive_non_detections < 5
             # there are always non-detections inbetween even if it should detect a token (only god knows why)

  def execute(self) -> None:
    """ position above token """
    rospy.logwarn('behavior: step onto token')
    self._killerrobot.move(self._killerrobot.max_speed / 7, 0)
    self._killerrobot.isovertoken = True
