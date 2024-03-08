#!/usr/bin/env python3
import numpy as np

import wpilib
from wpilib import DutyCycleEncoder, DigitalInput
from wpimath import controller
from navx import AHRS
from phoenix5 import WPI_TalonSRX
from rev import CANSparkMax, CANSparkLowLevel
from photonlibpy.photonCamera import PhotonCamera
from magicbot import MagicRobot, tunable, feedback

from components.climber import Climber
from components.drivetrain import Drivetrain
from components.drive_control import DriveControl
from components.intake import Intake
from components.intake_control import IntakeControl
from components.shooter import Shooter
from components.shooter_control import ShooterControl
from components.vision import Vision, SmartCamera
import oi
import util


class MyRobot(MagicRobot):
    #
    # Define components here (high level first, low level last)
    #
    drive_control: DriveControl
    intake_control: IntakeControl
    shooter_control: ShooterControl

    climber: Climber
    drivetrain: Drivetrain
    intake: Intake
    shooter: Shooter
    vision: Vision

    def createObjects(self):
        """Initialize variables to be injected:"""
        BRUSHLESS = CANSparkLowLevel.MotorType.kBrushless

        self.gyro = AHRS.create_spi()

        self.climber_left_motor = WPI_TalonSRX(61)
        self.climber_right_motor = WPI_TalonSRX(62)

        self.drivetrain_front_left_motor = CANSparkMax(5, BRUSHLESS)
        self.drivetrain_front_right_motor = CANSparkMax(50, BRUSHLESS)
        self.drivetrain_back_left_motor = CANSparkMax(51, BRUSHLESS)
        self.drivetrain_back_right_motor = CANSparkMax(52, BRUSHLESS)

        self.intake_joint_left_motor = CANSparkMax(2, BRUSHLESS)
        self.intake_joint_right_motor = CANSparkMax(3, BRUSHLESS)
        self.intake_left_encoder = DutyCycleEncoder(DigitalInput(0))
        self.intake_right_encoder = DutyCycleEncoder(DigitalInput(1))
        self.intake_left_encoder_offset = 0.886  # 0.90
        self.intake_right_encoder_offset = 0.359  # 0.38
        self.intake_belt_motor = util.WPI_TalonFX(6)

        self.shooter_belt_motor = CANSparkMax(55, BRUSHLESS)
        self.shooter_feed_left_motor = WPI_TalonSRX(25)
        self.shooter_feed_right_motor = WPI_TalonSRX(41)
        self.shooter_shooter_left_motor = CANSparkMax(53, BRUSHLESS)
        self.shooter_shooter_right_motor = CANSparkMax(54, BRUSHLESS)

        # CHANGE THE SOUGHT_IDS BEFORE COMP!!!!!!!!!!
        self.vision_left_camera = SmartCamera(
            "Global_Shutter_Camera", np.array([0.3556, 0.2159]), sought_ids=[8]
        )
        self.vision_right_camera = SmartCamera("", np.array([0, 0]), sought_ids=[8])

        self.oi = oi.Double_Xbox_OI()

        self.drive_curve = util.cubic_curve(
            scalar=0.5, deadband=0.1, max_mag=1, offset=0.2, absolute_offset=False
        )
        self.turn_curve = util.cubic_curve(
            scalar=0.5, deadband=0.1, max_mag=1, offset=0.2, absolute_offset=False
        )

    def teleopPeriodic(self):
        self.intake.update_position()
        self.intake_control.update_shooter_state(self.shooter_control.current_state)
        self.shooter_control.update_intake_state(self.intake_control.current_state)

        with self.consumeExceptions():
            self.drive_control.arcade_drive(
                self.drive_curve(self.oi.drive_forward()),
                self.turn_curve(self.oi.drive_turn()),
            )

            if self.oi.contract_left_climber():
                self.climber.contract_left()
            if self.oi.contract_right_climber():
                self.climber.contract_right()
            if self.oi.extend_left_climber():
                self.climber.extend_left()
            if self.oi.extend_right_climber():
                self.climber.extend_right()

            self.drive_control.engage()
            if self.oi.align():
                self.drive_control.request_align()
            if self.oi.cancel_align():
                self.drive_control.engage(initial_state="free", force=True)

            self.intake_control.engage()
            self.shooter_control.engage()
            if self.intake.get_position() is None:
                self.intake_control.engage(initial_state="disabled", force=True)
            if self.oi.soft_shoot() and self.drive_control.get_manually_aligned():
                self.shooter_control.request_shoot()
            if self.oi.hard_shoot():
                self.shooter_control.request_shoot()
            if self.oi.intake():
                self.intake_control.request_intake()
                self.shooter_control.request_intake()
            if self.oi.eject():
                self.intake_control.request_eject()
                self.intake_control.request_down()
                self.shooter_control.request_eject()
            if self.shooter_control.current_state == "idle":
                if not self.oi.intake_down():  # self.oi.intake_up()
                    self.intake_control.request_up()
                if self.oi.intake_down():
                    self.intake_control.request_down()

    @feedback
    def get_angle(self) -> float:
        return self.gyro.getAngle()


if __name__ == "__main__":
    wpilib.run(MyRobot)
