class CaptureToken:
  """ Register the token with coordinates """

  def isApplicable(self):
    """ if StepOntoToken just finished && token has not been registered yet """
    return False

  def execute(self):
    """ register coordinates and move on """