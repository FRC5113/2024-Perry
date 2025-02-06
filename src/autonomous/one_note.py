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

    @state(first=True)
    def finding_tag(self, state_tm):
        self.drivetrain.arcade_drive(0, 0)
        if self.vision.hasTargets() and state_tm > 0.5:
            self.next_state("aligning")

    @state
    def aligning(self, state_tm):
        self.drive_control.engage()
        self.drive_control.request_align()
        if not self.vision.hasTargets():
            self.next_state("finding_tag")
            return
        if self.drive_control.current_state == "free" and state_tm > 1:
            self.next_state("shooting")

    @timed_state(duration=2.5, next_state="leaving")
    def shooting(self):
        self.drivetrain.arcade_drive(0, 0)
        self.shooter_control.engage(initial_state="loading")

    @timed_state(duration=3, next_state="stopped")
    def leaving(self):
        self.drivetrain.arcade_drive(-0.5, 0)

    @state
    def stopped(self):
        self.drivetrain.arcade_drive(0, 0)
