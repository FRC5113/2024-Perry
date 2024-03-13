import wpilib
from wpilib.interfaces import MotorController
from wpilib.drive import DifferentialDrive
from magicbot import will_reset_to
from rev import CANSparkBase, CANSparkMax

import util


class Drivetrain:
    # annotate motor and configuration instances
    front_left_motor: CANSparkMax
    front_right_motor: CANSparkMax
    back_left_motor: CANSparkMax
    back_right_motor: CANSparkMax

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
        # print(forward, turn)
        self.forward = forward
        self.turn = turn

    def set_coast(self):
        self.front_left_motor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        self.front_right_motor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        self.back_left_motor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        self.back_right_motor.setIdleMode(CANSparkBase.IdleMode.kCoast)

    def set_brake(self):
        self.front_left_motor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        self.front_right_motor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        self.back_left_motor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        self.back_right_motor.setIdleMode(CANSparkBase.IdleMode.kBrake)

    def execute(self):
        self.drive.arcadeDrive(self.forward, self.turn)
