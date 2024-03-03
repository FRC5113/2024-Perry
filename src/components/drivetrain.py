import wpilib
from wpilib.interfaces import MotorController
from wpilib.drive import DifferentialDrive
from magicbot import will_reset_to
from rev import CANSparkBase

import util


class Drivetrain:
    # annotate motor and configuration instances
    front_left_motor: MotorController
    front_right_motor: MotorController
    back_left_motor: MotorController
    back_right_motor: MotorController

    # values will reset to 0 after every time control loop runs
    forward = will_reset_to(0)
    turn = will_reset_to(0)

    def setup(self):
        self.front_left_motor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        self.front_right_motor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        self.back_left_motor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        self.back_right_motor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        self.left_motor_controller_group = wpilib.MotorControllerGroup(
            self.front_left_motor, self.back_left_motor
        )
        self.right_motor_controller_group = wpilib.MotorControllerGroup(
            self.front_right_motor, self.back_right_motor
        )
        self.right_motor_controller_group.setInverted(True)
        self.drive = DifferentialDrive(
            self.left_motor_controller_group, self.right_motor_controller_group
        )

    def on_enable(self):
        self.drive.setSafetyEnabled(True)

    def arcade_drive(self, forward: float, turn: float):
        if not (-1.0 <= forward <= 1.0):
            raise Exception(f"Improper value for forward entered: {forward}")
        if not (-1.0 <= turn <= 1.0):
            raise Exception(f"Improper value for turn entered: {turn}")
        # print(self.front_left_motor.get(), self.back_left_motor.get())
        # print("R", self.front_right_motor.get(), self.back_right_motor.get())
        self.forward = forward
        self.turn = turn

    def execute(self):
        self.drive.arcadeDrive(self.forward, self.turn)

    def test_right(self):
        self.right_motor_controller_group.set(0.4)
        # self.drive.arcadeDrive(0.4,0.0)
        # self.back_right_motor.set(0.2)
        pass
