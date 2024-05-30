from magicbot import AutonomousStateMachine, state, timed_state

from components.drivetrain import Drivetrain
from components.shooter_control import ShooterControl


class Shoot_DelayLeave(AutonomousStateMachine):
    MODE_NAME = "shoot_delay_leave"

    drivetrain: Drivetrain
    shooter_control: ShooterControl

    @timed_state(duration=2, first=True, next_state="staying")
    def shoot(self):
        self.shooter_control.engage(initial_state="loading")

    @timed_state(duration=7, next_state="going")
    def staying(self):
        self.drivetrain.arcade_drive(0, 0)

    @timed_state(duration=3, next_state="stopping")
    def going(self):
        self.drivetrain.arcade_drive(-0.5, 0)

    @state
    def stopping(self):
        self.drivetrain.arcade_drive(0, 0)
