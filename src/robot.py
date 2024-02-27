#!/usr/bin/env python3
import wpilib
from wpilib import DutyCycleEncoder, DigitalInput
from phoenix5 import WPI_TalonSRX
from rev import CANSparkMax, CANSparkLowLevel
from magicbot import MagicRobot, tunable, feedback

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
        BRUSHLESS = CANSparkLowLevel.MotorType.kBrushless
        self.drivetrain_front_left_motor = CANSparkMax(5, BRUSHLESS)
        self.drivetrain_front_right_motor = CANSparkMax(50, BRUSHLESS)
        self.drivetrain_back_left_motor = CANSparkMax(51, BRUSHLESS)
        self.drivetrain_back_right_motor = CANSparkMax(52, BRUSHLESS)
        self.indexer_feed_left_motor = WPI_TalonSRX(25)
        self.indexer_feed_right_motor = WPI_TalonSRX(41)
        self.indexer_belt_motor = util.WPI_TalonFX(6)
        self.intake_left_motor = CANSparkMax(2, BRUSHLESS)
        self.intake_right_motor = CANSparkMax(3, BRUSHLESS)
        self.intake_left_encoder = DutyCycleEncoder(DigitalInput(0))
        self.intake_right_encoder = DutyCycleEncoder(DigitalInput(1))
        self.intake_left_encoder_offset = 0.08
        self.intake_right_encoder_offset = 0.80
        self.intake_encoder_error_tolerance = 0.1
        self.intake_lower_limit = 0.0
        self.intake_upper_limit = 0.39
        self.shooter_left_motor = CANSparkMax(53, BRUSHLESS)
        self.shooter_right_motor = CANSparkMax(54, BRUSHLESS)

        self.xbox = wpilib.XboxController(0)

        self.drive_curve = util.cubic_curve(scalar=0.5, deadband=0.1, max_mag=1)
        self.intake_curve = util.linear_curve(scalar=0.05
                                              , deadband=0.1, max_mag=1)

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
            if self.xbox.getBButton():
                self.indexer.enable_feed()
            else:
                self.indexer.disable_feed()

            if self.xbox.getXButton():
                self.indexer.set_belt_speed(-0.1)
                self.indexer.enable_belt()

            if self.xbox.getYButton():
                self.indexer.set_belt_speed(0.1)
                self.indexer.enable_belt()
            """VERY UNSAFE: MAKE SURE TO IMPLEMENT LIMIT SWITCHES
            AND/OR ENCODERS IF TESTING WITH FULL INTAKE ATTACHED
            """
            self.intake.set_speed(self.intake_curve(self.xbox.getRightY()))
            if self.xbox.getLeftBumper():
                self.intake.down()
            if self.xbox.getRightBumper():
                self.intake.up()

    def testPeriodic(self):
        self.drivetrain.test_right()

    @feedback
    def get_intake_pos(self):
        position = self.intake.get_position()
        if position is not None:
            return position
        return 0
    
    @feedback
    def get_intake_left_pos(self):
        return self.intake.get_left_position()
    
    @feedback
    def get_intake_right_pos(self):
        return self.intake.get_right_position()
    
    @feedback
    def get_upper_limit(self):
        return self.intake.is_past_upper_limit()
    
    @feedback
    def get_lower_limit(self):
        return self.intake.is_past_lower_limit()
    



if __name__ == "__main__":
    wpilib.run(MyRobot)
