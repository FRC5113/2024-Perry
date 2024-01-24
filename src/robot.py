#!/usr/bin/env python3
import wpilib
from rev import CANSparkMax, CANSparkLowLevel
from magicbot import MagicRobot

from components.drivetrain import Drivetrain
import util


class MyRobot(MagicRobot):
    #
    # Define components here
    #

    drivetrain: Drivetrain

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""
        self.drivetrain_front_left_motor = CANSparkMax(
            41, CANSparkLowLevel.MotorType.kBrushless
        )
        self.drivetrain_front_right_motor = CANSparkMax(
            44, CANSparkLowLevel.MotorType.kBrushless
        )
        self.drivetrain_back_left_motor = CANSparkMax(
            42, CANSparkLowLevel.MotorType.kBrushless
        )
        self.drivetrain_back_right_motor = CANSparkMax(
            43, CANSparkLowLevel.MotorType.kBrushless
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
        except:
            self.onException()


if __name__ == "__main__":
    wpilib.run(MyRobot)
