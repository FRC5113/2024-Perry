import math
import numpy as np

import wpimath.controller
import magicbot
from magicbot.state_machine import state, timed_state
from magicbot import tunable, will_reset_to
import navx

from components.drivetrain import Drivetrain
from components.vision import Vision
import util


class DriveControl(magicbot.StateMachine):
    # other components
    drivetrain: Drivetrain
    vision: Vision

    # variables to be injected
    gyro: navx.AHRS

    turn_to_angle_kP = tunable(0.025)
    turn_to_angle_kI = tunable(0)
    turn_to_angle_kD = tunable(0.003)
    turn_to_angle_tP = tunable(5)
    turn_to_angle_tV = tunable(0.1)

    drive_from_tag_kP = tunable(2)
    drive_from_tag_tP = tunable(0.1)
    drive_from_tag_setpoint = tunable(0.3)

    align_trigger = will_reset_to(False)

    def setup(self):
        # setup() required because tunables need to be fetched
        self.turn_to_angle_controller = wpimath.controller.PIDController(
            self.turn_to_angle_kP, self.turn_to_angle_kI, self.turn_to_angle_kD
        )
        self.turn_to_angle_controller.setTolerance(
            self.turn_to_angle_tP, self.turn_to_angle_tV
        )
        self.turn_to_angle_controller.enableContinuousInput(0, 360)

    def request_align(self):
        self.align_trigger = True

    def set_angle(self, angle: float):
        """Changes the `turn_to_angle` PID controller's setpoint"""
        self.turn_to_angle_controller.setSetpoint(angle)

    def arcade_drive(self, forward: float, turn: float):
        """Call this instead of `drivetrain.arcade_drive()` because
        this will only drive if the drivetrain is free to do so
        """
        if self.current_state == "free":
            self.drivetrain.arcade_drive(forward, turn)

    def turn_to_tag(self):
        """Changes the `turn_to_angle` setpoint to one such that the robot
        would face an AprilTag
        """
        if not self.vision.hasTargets():
            return
        ct = np.array([self.vision.getX(), self.vision.getY(), self.vision.getZ()])
        latency = self.vision.getLatency()
        turn_rate = self.gyro.getRate()
        theta = self.vision.get_adjusted_heading(ct) - latency * turn_rate
        if self.turn_to_angle_controller.atSetpoint():
            theta = 0
        if self.vision.hasTargets():
            self.set_angle(self.gyro.getAngle() + theta)

    @state(first=True)
    def free(self):
        """First state -- does nothing to avoid accidents if `engage()`
        is called without an initial state.
        """
        
        if self.align_trigger and self.vision.hasTargets():
            self.next_state("aligning")

    @timed_state(duration=5.0, next_state="spacing")
    def aligning(self):
        """State in which robot uses a PID controller to turn to a certain
        angle using sensor data from the gyroscope.
        """
        if not self.vision.hasTargets():
            self.next_state("free")
            return
        
        self.turn_to_angle_controller.setPID(
            self.turn_to_angle_kP, self.turn_to_angle_kI, self.turn_to_angle_kD
        )
        self.turn_to_angle_controller.setTolerance(
            self.turn_to_angle_tP, self.turn_to_angle_tV
        )

        self.turn_to_tag()

        measurement = self.gyro.getAngle()
        output = self.turn_to_angle_controller.calculate(measurement)
        """Here (and elsewhere) the output is negated because a positive turn
        value in `arcade_drive()` corresponds with a decrease in angle.
        This could also be fixed with negative PID values, but this is not
        recommended.
        """
        self.drivetrain.arcade_drive(0, util.clamp(-output, -0.3, 0.3))
        if self.turn_to_angle_controller.atSetpoint():
            self.next_state("spacing")

    @state
    def spacing(self):
        """State in which robot drives forward or backward so that it is
        a set distance away from a detected Apriltag
        """
        if not self.vision.hasTargets():
            self.next_state("free")
            return
        measurement = self.vision.getX()
        error = self.drive_from_tag_setpoint - measurement
        output = error * self.drive_from_tag_kP
        self.drivetrain.arcade_drive(util.clamp(output, -0.3, 0.3), 0)
        if abs(error) < self.drive_from_tag_tP:
            self.next_state("free")