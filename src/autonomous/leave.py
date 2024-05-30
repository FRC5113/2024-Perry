from magicbot import AutonomousStateMachine, state, timed_state

from components.drivetrain import Drivetrain


class Leave(AutonomousStateMachine):
    MODE_NAME = "Leave"

    drivetrain: Drivetrain

    @timed_state(duration=3, first=True, next_state="stopped")
    def leaving(self):
        self.drivetrain.arcade_drive(-0.5, 0)

    @state
    def stopped(self):
        self.drivetrain.arcade_drive(0, 0)
