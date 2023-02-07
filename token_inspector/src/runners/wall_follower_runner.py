from token_inspector.srv import GimmeGoal, GimmeGoalResponse

class WallFollowerRunner:
    def __init__(self) -> None:
        pass

    def is_applicable(self, goal:GimmeGoalResponse):
        return False

    def run(self, goal:GimmeGoalResponse):
        pass