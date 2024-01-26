#!/usr/bin/env python3
import wpilib
from wpilib import DoubleSolenoid, PneumaticsModuleType
from phoenix5 import WPI_TalonSRX
from rev import CANSparkMax, CANSparkLowLevel
from magicbot import MagicRobot

from components.drivetrain import Drivetrain, OctoMode
from components.shooter import Shooter
import util


class MyRobot(MagicRobot):
    #
    # Define components here
    #

    drivetrain: Drivetrain
    shooter: Shooter

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""
        self.drivetrain_front_left_motor = WPI_TalonSRX(41)
        self.drivetrain_front_right_motor = WPI_TalonSRX(44)
        self.drivetrain_back_left_motor = WPI_TalonSRX(42)
        self.drivetrain_back_right_motor = WPI_TalonSRX(43)

        # update with actual ids
        self.drivetrain_front_left_solenoid = DoubleSolenoid(
            PneumaticsModuleType.REVPH, 0, 0
        )
        self.drivetrain_front_right_solenoid = DoubleSolenoid(
            PneumaticsModuleType.REVPH, 0, 0
        )
        self.drivetrain_back_left_solenoid = DoubleSolenoid(
            PneumaticsModuleType.REVPH, 0, 0
        )
        self.drivetrain_back_right_solenoid = DoubleSolenoid(
            PneumaticsModuleType.REVPH, 0, 0
        )

        self.shooter_left_motor = CANSparkMax(5, CANSparkLowLevel.MotorType.kBrushless)
        self.shooter_right_motor = CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless)

        self.xbox = wpilib.XboxController(0)

        self.drive_curve = util.cubic_curve(scalar=3, deadband=0.1, max_mag=1)
        self.shooter_curve = util.linear_curve(deadband=0.1)

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
