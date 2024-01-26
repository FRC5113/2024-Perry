#!/usr/bin/env python3
import wpilib
from wpilib import DoubleSolenoid, PneumaticsModuleType
from rev import CANSparkMax, CANSparkLowLevel
from magicbot import MagicRobot

from components.drivetrain import Drivetrain, OctoMode
from components.indexer import Indexer
from components.intake import Intake
from components.shooter import Shooter
import util


class MyRobot(MagicRobot):
    #
    # Define components here (high level first, low level last)
    #

    drivetrain: Drivetrain
    indexer: Indexer
    intake: Intake
    shooter: Shooter

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""
        self.drivetrain_front_left_motor = util.WPI_TalonFX(0)
        self.drivetrain_front_right_motor = util.WPI_TalonFX(0)
        self.drivetrain_back_left_motor = util.WPI_TalonFX(0)
        self.drivetrain_back_right_motor = util.WPI_TalonFX(0)

        # update with actual ids
        self.drivetrain_solenoid = DoubleSolenoid(
            PneumaticsModuleType.REVPH, 0, 0
        )

        self.indexer_feed_left_motor = util.EmptyController()
        self.indexer_feed_right_motor = util.EmptyController()
        self.indexer_belt_motor = util.EmptyController()
        self.intake_joint_left_motor = util.EmptyController()
        self.intake_joint_right_motor = util.EmptyController()
        self.shooter_left_motor = util.EmptyController()
        self.shooter_right_motor = util.EmptyController()

        self.xbox = wpilib.XboxController(0)

        self.drive_curve = util.cubic_curve(scalar=1, deadband=0.1, max_mag=1)
        self.intake_curve = util.linear_curve(scalar=1, deadband=0.1, max_mag=1)

    def teleopInit(self):
        """Called right before teleop control loop starts"""
        self.drivetrain.drive.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        with self.consumeExceptions():
            # may want to consider a more sound way of changing modes
            if self.xbox.getXButton():
                self.drivetrain.set_mode(OctoMode.MECANUM_DRIVE)
            if self.xbox.getYButton():
                self.drivetrain.set_mode(OctoMode.DIFFERENTIAL_DRIVE)

            if self.drivetrain.get_mode() == OctoMode.MECANUM_DRIVE:
                self.drivetrain.cartesian_drive(
                    self.drive_curve(self.xbox.getLeftY()),
                    self.drive_curve(self.xbox.getLeftX()),
                    self.drive_curve(-self.xbox.getRightX()),
                )
            if self.drivetrain.get_mode() == OctoMode.DIFFERENTIAL_DRIVE:
                self.drivetrain.arcade_drive(
                    self.drive_curve(self.xbox.getLeftY()),
                    self.drive_curve(-self.xbox.getRightX()),
                )

            self.shooter.spin(self.shooter_curve(-self.xbox.getRightTriggerAxis()))


if __name__ == "__main__":
    wpilib.run(MyRobot)
