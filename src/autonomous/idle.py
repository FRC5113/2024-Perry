from magicbot import AutonomousStateMachine, timed_state

from components.drivetrain import Drivetrain


class Idle(AutonomousStateMachine):
    MODE_NAME = "Idle"
    DEFAULT = True

    drivetrain: Drivetrain

    @timed_state(duration=15, first=True)
    def idle(self):
        self.drivetrain.arcade_drive(0, 0)
