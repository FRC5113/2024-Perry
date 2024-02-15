#!/usr/bin/env python3
import wpilib
from phoenix5 import WPI_TalonSRX
from rev import CANSparkMax, CANSparkLowLevel
from magicbot import MagicRobot

from components.drivetrain import Drivetrain
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
        # eventually correct all ids and remove EmptyControllers
        BRUSHLESS = CANSparkLowLevel.MotorType.kBrushless
        self.drivetrain_front_left_motor = CANSparkMax(5, BRUSHLESS)
        self.drivetrain_front_right_motor = CANSparkMax(50, BRUSHLESS)
        self.drivetrain_back_left_motor = CANSparkMax(51, BRUSHLESS)
        self.drivetrain_back_right_motor = CANSparkMax(52, BRUSHLESS)
        self.indexer_feed_left_motor = WPI_TalonSRX(0)
        self.indexer_feed_right_motor = WPI_TalonSRX(0)
        self.indexer_belt_motor = util.EmptyController()  # replace w/ talon fx
        self.intake_joint_left_motor = CANSparkMax(2, BRUSHLESS)
        self.intake_joint_right_motor = CANSparkMax(3, BRUSHLESS)
        self.shooter_left_motor = CANSparkMax(53, BRUSHLESS)
        self.shooter_right_motor = CANSparkMax(54, BRUSHLESS)

        self.xbox = wpilib.XboxController(0)

        self.drive_curve = util.cubic_curve(scalar=1, deadband=0.1, max_mag=1)
        self.intake_curve = util.linear_curve(scalar=0.2, deadband=0.1, max_mag=1)

    def teleopInit(self):
        """Called right before teleop control loop starts"""
        self.drivetrain.drive.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        with self.consumeExceptions():
            self.drivetrain.arcade_drive(
                self.drive_curve(self.xbox.getLeftY()),
                -self.drive_curve(self.xbox.getLeftX()),
            )
            if self.xbox.getAButton():
                self.shooter.enable()
            else:
                self.shooter.disable()
            """VERY UNSAFE: MAKE SURE TO IMPLEMENT LIMIT SWITCHES
            AND/OR ENCODERS BEFORE TESTING
            """
            self.intake.set_joint_speed = self.intake_curve(self.xbox.getRightY())


if __name__ == "__main__":
    wpilib.run(MyRobot)
