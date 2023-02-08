#!/usr/bin/env python
import json
import rospy
import os
from current_pos.msg import PoseTF
from helpers.helper import any_registered_token_within_distance
from sound_play.msg import SoundRequest

class CaptureToken:
  """ Register the token with coordinates """
  def __init__(self, killerrobot) -> None:
    self._killerrobot = killerrobot
    self._tagno = 0
    self._filename = "/killerrobot/token_positions.json"
    self._playsound = rospy.Publisher('robotsound', SoundRequest, queue_size=10)
    if os.path.exists(self._filename):
      os.remove(self._filename)

  def isApplicable(self) -> bool:
    """ if StepOntoToken just finished && token has not been registered yet """
    if self._killerrobot.isovertoken:
      self._killerrobot.isovertoken = False
      #play sound
      sound = SoundRequest()
      sound.sound = sound.PLAY_START
      sound.command = sound.PLAY_ONCE
      sound.volume = 1.0
      self._playsound.publish(sound)
      return True
    return False

  def execute(self):
    """ register coordinates and move on """
    rospy.logdebug('behavior: capture_token')
    pos = rospy.wait_for_message("pose_tf", PoseTF)
    pose = self._killerrobot.pose
    if any_registered_token_within_distance(pose, self._killerrobot.tokens, 0.2):
      rospy.logwarn(f'token at {pose.x}, {pose.y} not registered because it is has already been captured')
      return
    rospy.loginfo(f'capturing token at {pose.x}, {pose.y} with id {self._tagno}')
    self._killerrobot.register_token(self._tagno, pos.mapPose.x, pos.mapPose.y, pos.mapPose.angle)

    listobj = []
    dictionary = {
      "name": self._tagno,
      "found": False,
      "map_position": {
        "x": pos.mapPose.x,
        "y": pos.mapPose.y,
        "angle": pos.mapPose.angle
      }
    }
    # write to file
    if os.path.isfile(self._filename):
      with open(self._filename) as fp:
        listobj = json.load(fp)
    listobj.append(dictionary)
    with open(self._filename,"w") as outfile:
      json.dump(listobj, outfile, indent=3)

    self._tagno += 1


