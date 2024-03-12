from magicbot import AutonomousStateMachine, state, timed_state

from components.drivetrain import Drivetrain


class DelayLeave(AutonomousStateMachine):
    MODE_NAME = "delay_leave"

    drivetrain: Drivetrain

    @timed_state(duration=10, first=True, next_state="going")
    def staying(self):
        self.drivetrain.arcade_drive(0, 0)

    @timed_state(duration=2, next_state="stopping")
    def going(self):
        self.drivetrain.arcade_drive(-0.5, 0)

    @state
    def stopping(self):
        self.drivetrain.arcade_drive(0, 0)
