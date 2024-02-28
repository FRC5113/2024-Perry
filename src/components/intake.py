from wpilib import MotorControllerGroup, DutyCycleEncoder
from wpilib.interfaces import MotorController
from magicbot import tunable, will_reset_to
from wpimath import controller

import util


class Intake:
    """Component that controls the two motors that power the lifting
    and lowering of the intake module

    Injected Variables:
    left_motor -- MotorController of left side
    right_motor -- MotorController of right side
    left_encoder -- DutyCycleEncoder of left side
    right_encoder -- DutyCycleEncoder of right side
    left_encoder_offset -- Subtracted from left encoder measurements
    right_encoder_offset -- Subtracted from right encoder measurements
    encoder_error_tolerance -- Maximum amount left and right encoder
        values are allowed to differ
    lower_limit -- intake not allowed to drop below this (corresponds to down)
    upper_limit -- intake not allowed to raise above this (corresponds to up)
    """

    left_motor: MotorController
    right_motor: MotorController
    left_encoder: DutyCycleEncoder
    right_encoder: DutyCycleEncoder
    left_encoder_offset: float
    right_encoder_offset: float
    encoder_error_tolerance: float
    lower_limit: float
    upper_limit: float

    rotate_kP = tunable(0.5)
    speed_kI = tunable(0.03)

    speed = will_reset_to(0)
    position = 0
    last_position = 0
    error_flag = False

    max_pid_mag = tunable(0.2)

    def setup(self):
        self.right_motor.setInverted(True)
        self.motor_group = MotorControllerGroup(self.left_motor, self.right_motor)

        self.rotatePID = controller.PIDController(self.rotate_kP, 0, 0)
        self.rotatePID.setTolerance(0.05)
        self.rotatePID.enableContinuousInput(0,1)

        """This controller should only need an integral term because
        the error corresponds to a change in velocity, so velocity
        corresponds to the integral of error.
        """
        self.speedPID = controller.PIDController(0, self.speed_kI, 0)

    def up(self):
        if self.get_position() is None:
            print("PIDing FAILED - misaligned")
            return
        if self.rotatePID.getSetpoint() != 0.05:
            self.rotatePID.setSetpoint(0.05)
        pidOutput = self.rotatePID.calculate(self.get_position())
        print(pidOutput, self.rotatePID.getSetpoint(), self.get_position())
        self.set_speed(util.clamp(pidOutput, -self.max_pid_mag, self.max_pid_mag))        


    def down(self):
        if self.get_position() is None:
            print("PIDing FAILED - misaligned")
            return
        if self.rotatePID.getSetpoint() != 0.35:
            self.rotatePID.setSetpoint(0.35)
        pidOutput = self.rotatePID.calculate(self.get_position())
        print("DOWN", pidOutput, self.rotatePID.getSetpoint(), self.get_position())
        self.set_speed(util.clamp(pidOutput, -self.max_pid_mag, self.max_pid_mag))   

    def get_left_position(self) -> float:
        return (self.left_encoder.getAbsolutePosition() - self.left_encoder_offset) % 1
    
    def get_right_position(self) -> float:
        return (self.right_encoder.getAbsolutePosition() - self.right_encoder_offset) % 1

    def get_position(self) -> float | None:
        """Returns the average position between the two encoders.
        If the difference between the two is greater than the given
        threshold, this function will return `None`, signifying an
        error. Call this function every loop to update the position.
        Position zero should be when the intake is in "up" position.
        """
        a = self.get_left_position()
        b = self.get_right_position()
        position = None
        if abs(a - b) <= self.encoder_error_tolerance:
            position = (a + b) / 2
        if (
            abs(a - b + 1) <= self.encoder_error_tolerance
            or abs(a - b - 1) <= self.encoder_error_tolerance
        ):
            position = (a + b) / 2 - 0.5
            if position < 0:
                position += 1
        return position
    
    def get_speed(self) -> float | None:
        """Returns speed of intake in rotations/second"""
        if self.position is None:
            return None
        speed = self.position - self.last_position
        if speed > 0.5:
            return -(1 - speed) * 50
        if speed < -0.5:
            return (1 + speed) * 50
        return speed * 50

    
    def is_past_lower_limit(self) -> bool:
        if self.position is None:
            return True
        midpoint = (util.cyclic_average(self.lower_limit, self.upper_limit) + 0.5) % 1
        return util.cyclic_contains(self.position, self.lower_limit, midpoint)
    
    def is_past_upper_limit(self) -> bool:
        if self.position is None:
            return True
        midpoint = (util.cyclic_average(self.lower_limit, self.upper_limit) + 0.5) % 1
        return util.cyclic_contains(self.position, self.upper_limit, midpoint)

    def set_speed(self, speed: float):
        assert -1.0 < speed < 1.0, f"Improper intake joint speed: {speed}"
        self.speed = speed

    def set_speed_pid(self, speed: float):
        if speed == 0:
            self.set_speed(0)
            return
        measurement = self.get_speed()
        if measurement is None:
            return
        self.speedPID.setSetpoint(speed)
        output = self.speedPID.calculate(measurement)
        print(output)
        self.set_speed(output)

    def execute(self):
        self.last_position = self.position
        self.position = self.get_position()
        self.rotatePID.setP(self.rotate_kP)
        self.speedPID.setI(self.speed_kI)
        if self.position is not None:
            if self.speed > 0 and self.is_past_lower_limit():
                self.motor_group.set(0)
                return
            if self.speed < 0 and self.is_past_upper_limit():
                self.motor_group.set(0)
                return
            self.motor_group.set(self.speed)
        else:
            self.motor_group.set(0)
            print("ERROR: INTAKE ENCODERS MISALIGNED")
