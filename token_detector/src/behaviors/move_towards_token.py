from camera_integrators.raspicam_detector import RaspicamDetector

class MoveTowardsToken:
  """ Target is ahead and in sight """

  def __init__(self, killerrobot):
    self.__killerrobot = killerrobot
    self.__raspicam_detector = RaspicamDetector()

  def isApplicable(self):
    return len(self.__raspicam_detector.tokens) > 0

  def execute(self):
    clst_tkn = self.__raspicam_detector.tokens[0]
    linear_velocity = 0.3 * clst_tkn[2] + 0.05
    angular_velocity = clst_tkn[3] * 0.3
    self.__killerrobot.move(linear_velocity, angular_velocity)