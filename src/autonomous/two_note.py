import math
import numpy

from magicbot import AutonomousStateMachine, state, timed_state

from components.drivetrain import Drivetrain
from components.drive_control import DriveControl
from components.shooter_control import ShooterControl
from components.vision import Vision
import util


class TwoNote(AutonomousStateMachine):
    MODE_NAME = "Two Note"

    drive_control: DriveControl
    shooter_control: ShooterControl
    drivetrain: Drivetrain
    vision: Vision

    @state(first=True)
    def finding_tag1(self, state_tm):
        self.drivetrain.arcade_drive(0, 0)
        if self.vision.hasTargets() and state_tm > 0.5:
            self.next_state("aligning1")

    @state
    def aligning1(self, state_tm):
        self.drive_control.engage()
        self.drive_control.request_align()
        if not self.vision.hasTargets():
            self.next_state("finding_tag1")
            return
        if self.drive_control.current_state == "free" and state_tm > 1:
            self.next_state("shooting1")

    @timed_state(duration=2.5, next_state="aligning_to_note")
    def shooting1(self):
        self.drivetrain.arcade_drive(0, 0)
        self.shooter_control.engage(initial_state="loading")

    @state
    def aligning_to_note(self, state_tm):
        if not self.vision.hasTargets():
            return
        rtc = [self.vision.getX(), self.vision.getY()]
        rtf = list(util.rotate_vector(rtc[0],rtc[1]))
        tn = [2.845,1.448]
        rn = [tn[0] + rtf[0], tn[1] + rtf[1]]
        theta = math.atan2(rn[1], rn[0])
        self.drive_control.set_angle(theta)
        self.drive_control.engage(initial_state="turning_to_angle")
        if self.drive_control.current_state == "free" and state_tm > 1:
            self.next_state("stopped")
        

    @state
    def stopped(self):
        self.drivetrain.arcade_drive(0, 0)
