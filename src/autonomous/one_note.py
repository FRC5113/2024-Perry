from magicbot import AutonomousStateMachine, state, timed_state

from components.drivetrain import Drivetrain
from components.drive_control import DriveControl
from components.shooter_control import ShooterControl
from components.vision import Vision


class OneNote(AutonomousStateMachine):
    MODE_NAME = "One Note"

    drive_control: DriveControl
    shooter_control: ShooterControl
    drivetrain: Drivetrain
    vision: Vision

    @timed_state(duration=3, first=True, next_state="finding_tag")
    def driving_forward(self):
        self.drivetrain.arcade_drive(0.5, 0)

    @state
    def finding_tag(self):
        self.drivetrain.arcade_drive(0, 0)
        if self.vision.hasTargets():
            self.next_state("aligning")

    @state
    def aligning(self):
        self.drive_control.engage(initial_state="aligning")
        if not self.vision.hasTargets():
            self.next_state("finding_tag")
            return
        if self.drive_control.current_state == "free":
            self.next_state("shooting")

    @timed_state(duration=2, next_state="stopped")
    def shooting(self):
        self.drivetrain.arcade_drive(0, 0)
        self.shooter_control.engage(initial_state="loading")

    @state
    def stopped(self):
        self.drivetrain.arcade_drive(0, 0)
