class FindWall:
  """ Move towards wall where turtlebot left it to capture a token """
  
  def __init__(self, killerrobot) -> None:
    self.__killerrobot = killerrobot

  def isApplicable(self):
    """ always """
    return True

  def execute(self):
    """ move towards point in wall where follower was exited """
    pass