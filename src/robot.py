#!/usr/bin/env python3
import wpilib
from wpilib import DutyCycleEncoder, DigitalInput
from wpimath import controller
from phoenix5 import WPI_TalonSRX
from rev import CANSparkMax, CANSparkLowLevel
from magicbot import MagicRobot, tunable, feedback

from components.drivetrain import Drivetrain
from components.intake import Intake
from components.shooter import Shooter
from oi import OI
import util


class MyRobot(MagicRobot):
    #
    # Define components here (high level first, low level last)
    #

    drivetrain: Drivetrain
    intake: Intake
    shooter: Shooter

    def createObjects(self):
        """Initialize variables to be injected:"""
        BRUSHLESS = CANSparkLowLevel.MotorType.kBrushless

        self.drivetrain_front_left_motor = CANSparkMax(5, BRUSHLESS)
        self.drivetrain_front_right_motor = CANSparkMax(50, BRUSHLESS)
        self.drivetrain_back_left_motor = CANSparkMax(51, BRUSHLESS)
        self.drivetrain_back_right_motor = CANSparkMax(52, BRUSHLESS)

        self.intake_joint_left_motor = CANSparkMax(2, BRUSHLESS)
        self.intake_joint_right_motor = CANSparkMax(3, BRUSHLESS)
        self.intake_left_encoder = DutyCycleEncoder(DigitalInput(0))
        self.intake_right_encoder = DutyCycleEncoder(DigitalInput(1))
        self.intake_left_encoder_offset = 0.886 # 0.90
        self.intake_right_encoder_offset = 0.359 # 0.38
        self.intake_encoder_error_tolerance = 0.1
        self.intake_lower_limit = 0.0
        self.intake_upper_limit = 0.39
        self.intake_horizontal_offset = 0.27 #fix
        self.intake_feedforward = controller.ArmFeedforward(0.24, 0.42, 0.78, 0.01)
        self.intake_belt_motor = util.WPI_TalonFX(6)

        self.shooter_belt_motor = CANSparkMax(55, BRUSHLESS)
        self.shooter_feed_left_motor = WPI_TalonSRX(25)
        self.shooter_feed_right_motor = WPI_TalonSRX(41)
        self.shooter_shooter_left_motor = CANSparkMax(53, BRUSHLESS)
        self.shooter_shooter_right_motor = CANSparkMax(54, BRUSHLESS)

        self.oi = OI()

        self.drive_curve = util.cubic_curve(scalar=1, deadband=0.1, max_mag=1)
        self.turn_curve = util.cubic_curve(scalar=0.75, deadband=0.1, max_mag=1)
        # self.intake_curve = util.linear_curve(scalar=0.05, deadband=0.1, max_mag=1)

    def teleopInit(self):
        self.drivetrain.drive.setSafetyEnabled(True)

    def teleopPeriodic(self):
        with self.consumeExceptions():
            self.drivetrain.arcade_drive(
                self.drive_curve(self.oi.drive_forward()),
                self.turn_curve(self.oi.drive_turn()),
            )
            if self.oi.shoot():
                self.shooter.intake()
                self.shooter.feed()
                self.shooter.shoot()
            # if self.oi.feed():
            if self.oi.do_intake():
                self.shooter.feed()
                self.intake.intake()
                self.shooter.intake()
            # if self.oi.eject():
            #     self.intake.eject()
            #     self.shooter.eject()
            if self.oi.intake_down():
                # self.intake.set_joint_setpoint(self.intake_upper_limit)
                self.intake.set_joint_voltage(1)
                # self.intake.set_index_speed(0.25)
            if self.oi.intake_up():
                self.intake.set_joint_voltage(-0.5)
                # self.intake.set_joint_setpoint(self.intake_lower_limit)
                # self.intake.set_index_speed(-0.25)
            

    def testInit(self):
        self.intake.set_joint_voltage(0.001)

    def testPeriodic(self):
        """This is will find the approximate kS value for a feedforward
        intake controller. It will apply an increasing voltage to the
        joint until it moves up.
        """
        delta = 0.001
        voltage = self.intake.get_joint_voltage()
        if voltage != 0:
            self.intake.set_joint_voltage(voltage + delta)
        if abs(self.intake.get_speed()) > 0.003 and voltage > delta * 10:
            self.intake.set_joint_voltage(0)
            print("!!! ", end="")
        self.intake.execute()
        print(voltage - 5 * delta)

    
if __name__ == "__main__":
    wpilib.run(MyRobot)
