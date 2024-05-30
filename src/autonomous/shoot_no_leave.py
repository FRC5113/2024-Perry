from magicbot import AutonomousStateMachine, state, timed_state

from components.shooter_control import ShooterControl
from components.drivetrain import Drivetrain


class Shootnoleave(AutonomousStateMachine):
    MODE_NAME = "Shoot_No_Leave"

    shooter_control: ShooterControl
    drivetrain: Drivetrain

    @timed_state(duration=2, first=True, next_state="stopped")
    def shooting(self):
        self.shooter_control.engage(initial_state="loading")
    @state
    def stopped(self):
        self.drivetrain.arcade_drive(0, 0)
