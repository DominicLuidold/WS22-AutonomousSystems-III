#!/usr/bin/env python
import json
import rospy
from os import path
from current_pos.msg import PoseTF

class CaptureToken:
  """ Register the token with coordinates """
  def __init__(self, killerroboter) -> None:
    self._killerroboter = killerroboter
    self._tagno = 0
    self._filename = "/killerrobot/token_positions.json"

  def isApplicable(self) -> bool:
    """ if StepOntoToken just finished && token has not been registered yet """
    if self._killerroboter.isovertoken:
      self._killerroboter.isovertoken = False
      return True
    return False

  def execute(self):
    """ register coordinates and move on """
    pos = rospy.wait_for_message("pose_tf", PoseTF)
    listobj = []

    #get position
    dictionary = {
      "name": self._tagno,
      "found": False,
      "map_position": {
        "x": pos.mapPose.x,
        "y": pos.mapPose.y,
        "angle": pos.mapPose.angle
      }
    }

    if path.isfile(self._filename):
      with open(self._filename) as fp:
        listobj = json.load(fp)

    listobj.append(dictionary)

    with open(self._filename,"w") as outfile:
      json.dump(listobj, outfile, indent=3)

    self._tagno += 1
