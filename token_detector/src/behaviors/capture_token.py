class CaptureToken:
  """ Register the token with coordinates """

  def isApplicable(self, tokens):
    """ if StepOntoToken just finished && token has not been registered yet """
    return False

  def execute(self, token_detector):
    """ register coordinates and move on """