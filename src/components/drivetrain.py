from enum import Enum

import wpilib
from wpilib import DoubleSolenoid
from wpilib.interfaces import MotorController
from wpilib.drive import DifferentialDrive, MecanumDrive
from magicbot import will_reset_to


class OctoMode(Enum):
    DISABLED = 0
    DIFFERENTIAL_DRIVE = 1
    MECANUM_DRIVE = 2


class Drivetrain:
    # annotate motor and configuration instances
    front_left_motor: MotorController
    front_right_motor: MotorController
    back_left_motor: MotorController
    back_right_motor: MotorController

    front_left_solenoid: DoubleSolenoid
    front_right_solenoid: DoubleSolenoid
    back_left_solenoid: DoubleSolenoid
    back_right_solenoid: DoubleSolenoid

    mode: OctoMode = OctoMode.MECANUM_DRIVE

    # used to add a delay when switching states
    buffer: int = 0
    delay_seconds: float = 1

    # values will reset to 0 after every time control loop runs
    x_speed: float = will_reset_to(0)
    y_speed: float = will_reset_to(0)
    z_rotation: float = will_reset_to(0)

    def setup(self):
        self.left_motor_controller_group = wpilib.MotorControllerGroup(
            self.front_left_motor, self.back_left_motor
        )
        self.right_motor_controller_group = wpilib.MotorControllerGroup(
            self.front_right_motor, self.back_right_motor
        )
        self.right_motor_controller_group.setInverted(True)
        self.differential_drive = DifferentialDrive(
            self.left_motor_controller_group, self.right_motor_controller_group
        )
        """ might need to invert right motors -- 
        not sure if inverting the group does this"""
        self.mecanum_drive = MecanumDrive(
            self.front_left_motor,
            self.back_left_motor,
            self.front_right_motor,
            self.back_right_motor,
        )

    def get_mode(self) -> OctoMode:
        return self.mode

    def get_expected_solenoid_value(self) -> DoubleSolenoid.Value:
        # might be backwards
        if self.get_mode() == OctoMode.DIFFERENTIAL_DRIVE:
            return DoubleSolenoid.Value.kForward
        if self.get_mode() == OctoMode.MECANUM_DRIVE:
            return DoubleSolenoid.Value.kReverse
        return DoubleSolenoid.Value.kOff

    def is_in_correct_mode(self) -> bool:
        expected = self.get_expected_solenoid_value()
        if (
            self.front_left_solenoid.get() != expected
            or self.front_right_solenoid.get() != expected
            or self.back_left_solenoid.get() != expected
            or self.back_right_solenoid.get() != expected
        ):
            # assumes MagicRobot runs at 50Hz (likely incorrect)
            self.buffer = int(self.delay_seconds * 50)
            return False
        return True

    def set_mode(self, mode: OctoMode):
        self.mode = mode

    def arcade_drive(self, x_speed: float, z_rotation: float):
        assert -1.0 < x_speed < 1.0, f"Improper x_speed: {x_speed}"
        assert -1.0 < z_rotation < 1.0, f"Improper z_rotation: {z_rotation}"
        self.x_speed = x_speed
        self.y_speed = 0
        self.z_rotation = z_rotation

    def cartesian_drive(self, x_speed: float, y_speed: float, z_rotation: float):
        assert -1.0 < x_speed < 1.0, f"Improper x_speed: {x_speed}"
        assert -1.0 < y_speed < 1.0, f"Improper y_speed: {x_speed}"
        assert -1.0 < z_rotation < 1.0, f"Improper z_rotation: {z_rotation}"
        assert (
            self.get_mode == OctoMode.MECANUM_DRIVE
        ), "Cannot use cartesian_drive in tank mode"
        self.x_speed = x_speed
        self.y_speed = y_speed
        self.z_rotation = z_rotation

    def execute(self):
        if not self.is_in_correct_mode():
            expected = self.get_expected_solenoid_value()
            self.front_left_solenoid.set(expected)
            self.front_right_solenoid.set(expected)
            self.back_left_solenoid.set(expected)
            self.back_right_solenoid.set(expected)
        if self.buffer > 0:
            self.buffer -= 1
            return

        if self.get_mode() == OctoMode.MECANUM_DRIVE:
            self.mecanum_drive.driveCartesian(
                self.x_speed, self.y_speed, self.z_rotation
            )
        if self.get_mode() == OctoMode.DIFFERENTIAL_DRIVE:
            self.differential_drive.arcadeDrive(self.x_speed, self.z_rotation)
