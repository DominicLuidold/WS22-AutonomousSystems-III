class FindWall:
  """ Move towards wall where turtlebot left it to capture a token """

  def isApplicable(self, tokens):
    """ always """
    return True

  def execute(self, killerrobot, token_detector):
    """ move towards point in wall where follower was exited """
    pass