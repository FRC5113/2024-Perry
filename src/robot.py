#!/usr/bin/env python3
import wpilib
from phoenix5 import WPI_TalonSRX
from rev import CANSparkMax, CANSparkLowLevel
from magicbot import MagicRobot

from components.drivetrain import Drivetrain
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

        self.shooter_left_motor = CANSparkMax(
            5, CANSparkLowLevel.MotorType.kBrushless
        )
        self.shooter_right_motor = CANSparkMax(
            2, CANSparkLowLevel.MotorType.kBrushless
        )

        self.xbox = wpilib.XboxController(0)

        self.curve = util.cubic_curve(scalar=3, deadband=0.1, max_mag=1)

    def teleopInit(self):
        """Called right before teleop control loop starts"""
        self.drivetrain.drive.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        try:
            self.drivetrain.arcade_drive(
                self.curve(self.xbox.getLeftY()), -self.curve(self.xbox.getLeftX())
            )
            self.shooter.spin(self.xbox.getRightY())
        except:
            self.onException()


if __name__ == "__main__":
    wpilib.run(MyRobot)
