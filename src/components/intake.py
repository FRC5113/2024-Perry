from wpilib import MotorControllerGroup, DutyCycleEncoder
from wpilib.interfaces import MotorController
from magicbot import tunable, will_reset_to, feedback
from wpimath import controller, units, filter

import util


class Intake:
    """Component that controls the two motors that power the lifting
    and lowering of the intake module

    Injected Variables:
    joint_left_motor -- MotorController of left side
    joint_right_motor -- MotorController of right side
    left_encoder -- DutyCycleEncoder of left side
    right_encoder -- DutyCycleEncoder of right side
    left_encoder_offset -- Subtracted from left encoder measurements
    right_encoder_offset -- Subtracted from right encoder measurements
    encoder_error_tolerance -- Maximum amount left and right encoder
        values are allowed to differ
    lower_limit -- intake not allowed to drop below this (corresponds to down)
    upper_limit -- intake not allowed to raise above this (corresponds to up)
    horizontal_offset -- offset between what is considered zero
    rotations and the rotations it would take for the intake to be
    horizontal
    feedforward -- feedforward controller that assists in finding the 
    correct voltage for a desired velocity and acceleration
    belt_motor -- MotorController of belt
    intake_indexer_motor -- Indexer neo550 
    """

    joint_left_motor: MotorController
    joint_right_motor: MotorController
    left_encoder: DutyCycleEncoder
    right_encoder: DutyCycleEncoder
    left_encoder_offset: float
    right_encoder_offset: float
    encoder_error_tolerance: float
    lower_limit: float
    upper_limit: float
    horizontal_offset: float
    feedforward: controller.ArmFeedforward

    belt_motor: MotorController

    indexer_motor: MotorController

    rotate_kP = tunable(0.5)
    joint_speed = will_reset_to(0)
    joint_voltage = 0 # make resettable after testing
    position = 0
    last_position = 0
    speed_filter = filter.MedianFilter(10)
    max_pid_mag = tunable(0.2)

    belt_intaking = will_reset_to(False)
    belt_ejecting = will_reset_to(False)
    belt_speed = 0.3

    index_speed = will_reset_to(0)

    def setup(self):
        self.joint_right_motor.setInverted(True)
        self.joint_motor_group = MotorControllerGroup(self.joint_left_motor, self.joint_right_motor)

        self.rotatePID = controller.PIDController(self.rotate_kP, 0, 0)
        self.rotatePID.setTolerance(0.05)
        self.rotatePID.enableContinuousInput(0, 1)

    # move these
    # def up(self):
    #     if self.position is None:
    #         print("PIDing FAILED - misaligned")
    #         return
    #     if self.rotatePID.getSetpoint() != 0.05:
    #         self.rotatePID.setSetpoint(0.05)
    #     pidOutput = self.rotatePID.calculate(self.get_position())
    #     print(pidOutput, self.rotatePID.getSetpoint(), self.get_position())
    #     self.set_motor_speed(util.clamp(pidOutput, -self.max_pid_mag, self.max_pid_mag))

    # def down(self):
    #     if self.position is None:
    #         print("PIDing FAILED - misaligned")
    #         return
    #     if self.rotatePID.getSetpoint() != 0.35:
    #         self.rotatePID.setSetpoint(0.35)
    #     pidOutput = self.rotatePID.calculate(self.get_position())
    #     print("DOWN", pidOutput, self.rotatePID.getSetpoint(), self.get_position())
    #     self.set_motor_speed(util.clamp(pidOutput, -self.max_pid_mag, self.max_pid_mag))
        
    def set_index_speed(self, speed):
        self.index_speed = speed

    # informational methods
    def get_radians(self) -> units.radians | None:
        """Returns the position of the intake such that 0 radians is 
        when the center of mass is directly in front of the axel and
        pi/2 radians is when the canter of mass is directly above the
        axel
        """
        if self.position is None:
            return None
        return units.rotationsToRadians(-(self.position - self.horizontal_offset))
    
    def convert_to_rotations(self, radians: units.radians) -> units.turns:
        """Converts radians to rotations with respect to the intake
        offsets
        """
        return -units.radiansToRotations(radians) + self.horizontal_offset

    @feedback
    def get_left_position(self) -> float:
        return (self.left_encoder.getAbsolutePosition() - self.left_encoder_offset) % 1

    @feedback
    def get_right_position(self) -> float:
        return (
            self.right_encoder.getAbsolutePosition() - self.right_encoder_offset
        ) % 1

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
        self.position = position
        return position
    
    @feedback
    def get_nt_position(self) -> float:
        self.get_position()
        if self.position is None:
            return 0
        return self.position

    def get_speed(self) -> float | None:
        """Returns speed of intake in rotations/second (via encoders)"""
        self.get_position()
        if self.position is None or self.last_position is None:
            return None
        speed = self.position - self.last_position
        if speed > 0.5:
            return self.speed_filter.calculate(-(1 - speed) * 50)
        if speed < -0.5:
            return self.speed_filter.calculate((1 + speed) * 50)
        return self.speed_filter.calculate(speed * 50)
    
    @feedback
    def get_nt_speed(self) -> float:
        if self.position is None:
            return 0
        return self.get_speed()
    
    def get_joint_voltage(self) -> float:
        return self.joint_voltage

    def is_past_lower_limit(self) -> bool:
        self.get_position()
        if self.position is None:
            return True
        midpoint = (util.cyclic_average(self.lower_limit, self.upper_limit) + 0.5) % 1
        return util.cyclic_contains(self.position, self.lower_limit, midpoint)

    def is_past_upper_limit(self) -> bool:
        self.get_position()
        if self.position is None:
            return True
        midpoint = (util.cyclic_average(self.lower_limit, self.upper_limit) + 0.5) % 1
        return util.cyclic_contains(self.position, self.upper_limit, midpoint)

    # control methods
    def set_joint_motor_speed(self, speed: float):
        assert -1.0 < speed < 1.0, f"Improper intake joint speed: {speed}"
        self.joint_speed = speed
    
    def set_joint_voltage(self, voltage: float) -> float:
        self.joint_voltage = voltage
        return voltage

    def set_joint_speed(self, velocity: units.turns_per_second, acceleration: units.turns_per_second_squared = 0) -> float:
        """Sets the motor voltage to match the supplied angular speed
        (rotations/second) and angular acceleration (rotations/second^2)
        using a feedforward controller. Returns the set voltage.
        """
        if self.position is None:
            return 0
        return self.set_joint_voltage(self.feedforward.calculate(self.get_radians(), velocity, acceleration))

    def intake(self):
        self.belt_intaking = True
        self.belt_ejecting = False

    def eject(self):
        self.belt_ejecting = True
        self.belt_intaking = False

    def execute(self):
        self.indexer_motor.set(self.index_speed)

        self.last_position = self.position
        self.rotatePID.setP(self.rotate_kP)
        if self.belt_intaking and self.belt_ejecting:
            self.belt_intaking = False

        if self.belt_intaking:
            self.belt_motor.set(self.belt_speed)
        elif self.belt_ejecting:
            self.belt_motor.set(-self.belt_speed)
        else:
            self.belt_motor.set(0)

        if self.position is not None:
            if (self.joint_speed > 0 or self.joint_voltage > 0) and self.is_past_lower_limit():
                self.joint_motor_group.set(0)
                return
            if (self.joint_speed < 0 or self.joint_voltage < 0) and self.is_past_upper_limit():
                self.joint_motor_group.set(0)
                return
            if self.joint_voltage != 0:
                self.joint_motor_group.setVoltage(self.joint_voltage)
            else:
                self.joint_motor_group.set(self.joint_speed)
        else:
            self.joint_motor_group.set(0)
            print("ERROR: INTAKE ENCODERS MISALIGNED")

    @feedback
    def get_upper_limit(self):
        return self.is_past_upper_limit()

    @feedback
    def get_lower_limit(self):
        return self.is_past_lower_limit()
