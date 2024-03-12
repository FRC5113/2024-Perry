from magicbot import AutonomousStateMachine, state, timed_state

from components.shooter_control import ShooterControl
from components.drivetrain import Drivetrain


class Shoot(AutonomousStateMachine):
    MODE_NAME = "Shoot"

    shooter_control: ShooterControl
    drivetrain: Drivetrain

    @timed_state(duration=2, first=True, next_state="leaving")
    def shooting(self):
        self.shooter_control.engage(initial_state="loading")

    @timed_state(duration=2, next_state="stopped")
    def leaving(self):
        self.drivetrain.arcade_drive(-0.5, 0)

    @state
    def stopped(self):
        self.drivetrain.arcade_drive(0, 0)
