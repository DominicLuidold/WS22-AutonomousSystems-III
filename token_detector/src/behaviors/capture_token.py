class CaptureToken:
  """ Register the token with coordinates """

  def __init__(self, killerroboter) -> None:
    self.__killerroboter = killerroboter

  def isApplicable(self):
    """ if StepOntoToken just finished && token has not been registered yet """
    return False

  def execute(self):
    """ register coordinates and move on """