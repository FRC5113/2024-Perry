from magicbot import AutonomousStateMachine, state, timed_state

from components.drivetrain import Drivetrain
from components.drive_control import DriveControl
from components.shooter_control import ShooterControl
from components.vision import Vision


class OneNoteJog(AutonomousStateMachine):
    MODE_NAME = "One Note (Jog)"

    drive_control: DriveControl
    shooter_control: ShooterControl
    drivetrain: Drivetrain
    vision: Vision

    @timed_state(duration=2, first=True, next_state="finding_tag")
    def driving_forward(self):
        self.drivetrain.arcade_drive(-0.5, 0)

    @state
    def finding_tag(self, state_tm):
        self.drivetrain.arcade_drive(0.05, 0)
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

    @timed_state(duration=2.5, next_state="stopped")
    def shooting(self):
        self.drivetrain.arcade_drive(0, 0)
        self.shooter_control.engage(initial_state="loading")

    @state
    def stopped(self):
        self.drivetrain.arcade_drive(0, 0)
