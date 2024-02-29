#!/usr/bin/env python3
import wpilib
from wpilib import DutyCycleEncoder, DigitalInput
from wpimath import controller
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
        """Initialize variables to be injected:"""
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
        self.intake_left_encoder_offset = 0.90
        self.intake_right_encoder_offset = 0.38
        self.intake_encoder_error_tolerance = 0.1
        self.intake_lower_limit = 0.0
        self.intake_upper_limit = 0.39
        self.intake_horizontal_offset = 0.29 #fix
        self.intake_feedforward = controller.ArmFeedforward(0.25, 0.42, 0.78, 0.01)

        self.shooter_left_motor = CANSparkMax(53, BRUSHLESS)
        self.shooter_right_motor = CANSparkMax(54, BRUSHLESS)

        """Initialize input objects"""
        self.xbox = wpilib.XboxController(0)
        self.joystick = wpilib.Joystick(1)
        self.drive_curve = util.cubic_curve(scalar=0.5, deadband=0.1, max_mag=1)
        self.intake_curve = util.linear_curve(scalar=0.05, deadband=0.1, max_mag=1)
        self.belt_speed = 0.33

    def teleopInit(self):
        """Called right before teleop control loop starts"""
        self.drivetrain.drive.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        # with self.consumeExceptions():
        #     self.drivetrain.arcade_drive(
        #         self.drive_curve(self.xbox.getLeftY()),
        #         -self.drive_curve(self.xbox.getLeftX()),
        #     )
        #     if self.xbox.getAButton():
        #         self.shooter.enable()
        #     else:
        #         self.shooter.disable()
        #     if self.xbox.getBButton():
        #         self.indexer.enable_feed()
        #     else:
        #         self.indexer.disable_feed()

        #     if self.xbox.getXButton():
        #         self.indexer.set_belt_speed(-self.belt_speed)
        #         self.indexer.enable_belt()

        #     if self.xbox.getYButton():
        #         self.indexer.set_belt_speed(self.belt_speed)
        #         self.indexer.enable_belt()
        #     """VERY UNSAFE: MAKE SURE TO IMPLEMENT LIMIT SWITCHES
        #     AND/OR ENCODERS IF TESTING WITH FULL INTAKE ATTACHED
        #     """
        #     self.intake.set_motor_speed(self.intake_curve(self.xbox.getRightY()))
        #     if self.xbox.getLeftBumper():
        #         self.intake.down()
        #     if self.xbox.getRightBumper():
        #         self.intake.up()
        with self.consumeExceptions():
            self.drivetrain.arcade_drive(
                self.drive_curve(self.joystick.getY()),
                -self.drive_curve(self.joystick.getX()),
            )
            if self.joystick.getRawButton(7):
                self.shooter.enable()
            else:
                self.shooter.disable()
            if self.joystick.getRawButton(8):
                self.indexer.enable_feed()
            else:
                self.indexer.disable_feed()

            if self.joystick.getRawButton(9):
                self.indexer.set_belt_speed(-self.belt_speed)
                self.indexer.enable_belt()

            if self.joystick.getRawButton(10):
                self.indexer.set_belt_speed(self.belt_speed)
                self.indexer.enable_belt()
            """VERY UNSAFE: MAKE SURE TO IMPLEMENT LIMIT SWITCHES
            AND/OR ENCODERS IF TESTING WITH FULL INTAKE ATTACHED
            """
            # self.intake.set_motor_speed(self.intake_curve(self.xbox.getRightY()))
            if self.joystick.getRawButton(3):
                # self.intake.set_motor_speed(self.intake_curve(1))
                self.intake.set_speed(0.2)
            if self.joystick.getRawButton(4):
                # self.intake.set_motor_speed(self.intake_curve(-1))
                self.intake.set_speed(-0.2)
            # if self.joystick.getRawButton(11):
            #     self.intake.down()
            # if self.joystick.getRawButton(12):
            #     self.intake.up()

    def testInit(self):
        self.intake.set_voltage(0.01)

    def testPeriodic(self):
        """This is will find the approximate kS value for a feedforward
        intake controller. It will apply an increasing voltage to the
        joint until it moves up.
        """
        voltage = self.intake.get_voltage()
        if voltage != 0:
            self.intake.set_voltage(voltage + 0.01)
        if abs(self.intake.get_speed()) > 0.02:
            self.intake.set_voltage(0)
            print("!!! ", end="")
        self.intake.execute()
        print(voltage)


if __name__ == "__main__":
    wpilib.run(MyRobot)
