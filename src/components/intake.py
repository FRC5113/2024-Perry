from wpilib import MotorControllerGroup, DutyCycleEncoder
from wpilib.interfaces import MotorController
from magicbot import tunable, will_reset_to, feedback
from wpimath import controller, units, filter, trajectory

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
    belt_motor -- MotorController of belt
    intake_indexer_motor -- Indexer neo550
    """

    joint_left_motor: MotorController
    joint_right_motor: MotorController
    left_encoder: DutyCycleEncoder
    right_encoder: DutyCycleEncoder
    left_encoder_offset: float
    right_encoder_offset: float
    belt_motor: util.WPI_TalonFX

    lower_limit = 0.0
    upper_limit = 0.41
    horizontal_offset = 0.27
    joint_kP = tunable(6)
    joint_tP = tunable(0.02)
    joint_max_velocity = tunable(0.3)
    joint_max_acceleration = tunable(0.7)
    # replace this with a ProfiledPIDController
    joint_PID = controller.ProfiledPIDController(
        0, 0, 0, trajectory.TrapezoidProfile.Constraints(0, 0)
    )
    joint_voltage = will_reset_to(0)
    feedforward = controller.ArmFeedforward(0.24, 0.42, 0.78, 0.01)
    position = 0

    last_position = 0
    speed_filter = filter.MedianFilter(10)
    motor_speed_filter = filter.MedianFilter(5)
    max_pid_mag = tunable(0.2)
    belt_intaking = will_reset_to(False)
    belt_ejecting = will_reset_to(False)
    belt_speed = tunable(-0.5)
    encoder_error_tolerance = 0.1
    note_detection_threshold = tunable(20)
    disabled = False


    def setup(self):
        self.joint_right_motor.setInverted(True)
        self.joint_motor_group = MotorControllerGroup(
            self.joint_left_motor, self.joint_right_motor
        )

        self.joint_PID.setP(self.joint_kP)
        self.joint_PID.setConstraints(
            trajectory.TrapezoidProfile.Constraints(
                self.joint_max_velocity, self.joint_max_acceleration
            )
        )
        self.joint_PID.setGoal(self.lower_limit)
        self.joint_PID.setTolerance(self.joint_tP)
        self.joint_PID.enableContinuousInput(0, 1)

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
        """Get position of left encoder adjusted so that 0 is up"""
        return (self.left_encoder.getAbsolutePosition() - self.left_encoder_offset) % 1

    @feedback
    def get_right_position(self) -> float:
        """Get position of right encoder adjusted so that 0 is up"""
        return (
            self.right_encoder.getAbsolutePosition() - self.right_encoder_offset
        ) % 1

    def update_position(self) -> float | None:
        """Returns the average position between the two encoders.
        If the difference between the two is greater than the given
        threshold, this function will return `None`, signifying an
        error. Call this function every loop to update the position.
        Position zero should be when the intake is in "up" position.
        """
        a = self.get_left_position()
        b = self.get_right_position()
        position = None

        if not self.left_encoder.isConnected() and not self.right_encoder.isConnected():
            self.position = position
            return position
        if not self.left_encoder.isConnected():
            position = b
            self.position = position
            return position
        if not self.right_encoder.isConnected():
            position = a
            self.position = position
            return position
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

    def get_position(self) -> float | None:
        return self.position

    @feedback
    def get_nt_position(self) -> float:
        """Only used for sending data to NetworkTables"""
        if self.position is None:
            return 0
        return self.position

    def get_speed(self) -> float | None:
        """Returns speed of intake in rotations/second (via encoders)"""
        if self.position is None or self.last_position is None:
            return None
        speed = self.position - self.last_position
        if speed > 0.5:
            return -(1 - speed) * 50
        if speed < -0.5:
            return (1 + speed) * 50
        return speed * 50

    def get_filtered_speed(self) -> float | None:
        """Returns speed smoothed with a median filter. Must be called
        repeatedly for filter to be useful
        """
        speed = self.get_speed()
        if speed is None:
            return None
        return self.speed_filter.calculate(speed)

    @feedback
    def get_nt_speed(self) -> float:
        """Only used for sending data to NetworkTables"""
        filtered_speed = self.get_filtered_speed()
        if self.position is None or not filtered_speed:
            return 0
        return filtered_speed

    @feedback
    def get_filtered_motor_speed(self) -> float:
        """Call continuously"""
        return self.motor_speed_filter.calculate(self.belt_motor.get_velocity().value)

    @feedback
    def get_joint_voltage(self) -> float:
        return self.joint_voltage

    def is_past_lower_limit(self) -> bool:
        """Returns `True` if the position exceeds the lower limit or if
        the encoders are misaligned. Note: this assumes that the
        position is closer to the lower limit than the upper limit.
        """
        if self.position is None:
            return True
        midpoint = (util.cyclic_average(self.lower_limit, self.upper_limit) + 0.5) % 1
        return util.cyclic_contains(self.position, self.lower_limit, midpoint)

    def is_past_upper_limit(self) -> bool:
        """Returns `True` if the position exceeds the lower limit or if
        the encoders are misaligned. Note: this assumes that the
        position is closer to the upper limit than the lower limit.
        """
        if self.position is None:
            return True
        midpoint = (util.cyclic_average(self.lower_limit, self.upper_limit) + 0.5) % 1
        return util.cyclic_contains(self.position, self.upper_limit, midpoint)

    @feedback
    def get_joint_setpoint(self) -> float:
        """Returns setpoint of the PID controller for the joint"""
        return self.joint_PID.getGoal().position

    def is_at_setpoint(self) -> bool:
        return self.joint_PID.atGoal()

    def has_note(self) -> bool:
        """Returns `True` if a note is detected in the intake. This is
        accomplished by looking at the velocity of the belt motor, as it
        should drop when a note enters the intake. Note that this may be
        inaccurate when the motor starts running."""
        return abs(self.get_filtered_motor_speed()) < self.note_detection_threshold

    # control methods
    def set_joint_voltage(self, voltage: float) -> float:
        self.joint_voltage = voltage
        return voltage

    def set_joint_speed(
        self,
        velocity: units.turns_per_second,
        acceleration: units.turns_per_second_squared = 0,
    ) -> float:
        """Sets the motor voltage to match the supplied angular speed
        (rotations/second) and angular acceleration (rotations/second^2)
        using a feedforward controller. Returns the set voltage. Must be
        continually called.
        """
        rad = self.get_radians()
        if rad is None:
            return 0
        return self.set_joint_voltage(
            self.feedforward.calculate(
                rad,
                units.rotationsToRadians(velocity),
                units.rotationsToRadians(acceleration),
            )
        )

    def set_joint_setpoint(self, setpoint: units.turns):
        self.joint_PID.setGoal(setpoint)

    def move_to_setpoint(self):
        """Uses the PID controller to find a speed for the joint based
        on the current position and the setpoint, then uses the
        feedforward controller to convert that speed to a voltage.
        """
        if self.position is None:
            return
        output = -self.joint_PID.calculate(self.position)
        """Because the input speed is the position error times kP
        (when kI=kD=0), the input acceleration should in theory be
        the velocity error times kP. (Might be completely wrong)
        """
        acceleration = self.joint_PID.getSetpoint().velocity
        if self.joint_PID.atGoal():
            output = 0
            acceleration = 0
        # print(
        #     f"y: {self.position}, r: {self.get_joint_setpoint()}, e: {self.joint_PID.getPositionError()}, u: {output}, u': {acceleration}"
        # )
        self.set_joint_speed(output, acceleration)  # clamp?

    def intake(self):
        self.belt_intaking = True
        self.belt_ejecting = False

    def eject(self):
        self.belt_ejecting = True
        self.belt_intaking = False

    def disable(self):
        self.disabled = True

    def override_disable(self):
        self.disabled = False

    def execute(self):

        self.last_position = self.position
        self.joint_PID.setP(self.joint_kP)
        self.joint_PID.setConstraints(
            trajectory.TrapezoidProfile.Constraints(
                self.joint_max_velocity, self.joint_max_acceleration
            )
        )
        if self.belt_intaking and self.belt_ejecting:
            self.belt_intaking = False

        if self.disabled:
            return
        if self.belt_intaking:
            self.belt_motor.set(self.belt_speed)
        elif self.belt_ejecting:
            self.belt_motor.set(-self.belt_speed)
        else:
            self.belt_motor.set(0)

        if self.position is not None:
            if (self.joint_voltage > 0 and self.is_past_lower_limit()) or (
                self.joint_voltage < 0 and self.is_past_upper_limit()
            ):
                self.joint_motor_group.set(0)
            else:
                self.joint_motor_group.setVoltage(self.joint_voltage)
        else:
            self.joint_motor_group.set(0)
            print("ERROR: INTAKE ENCODERS MISALIGNED")

    # extra feedback
    @feedback
    def get_upper_limit(self):
        return self.is_past_upper_limit()

    @feedback
    def get_lower_limit(self):
        return self.is_past_lower_limit()

    @feedback
    def get_pid_p_error(self):
        return self.joint_PID.getPositionError()

    @feedback
    def get_pid_v_error(self):
        return self.joint_PID.getVelocityError()

    @feedback
    def get_pid_at_setpoint(self):
        return self.is_at_setpoint()

    @feedback
    def get_motor_speed(self):
        return abs(self.belt_motor.get_velocity().value)

    @feedback
    def get_has_note(self):
        return self.has_note()
    
    @feedback
    def get_status_intake_belt(self):
        return not self.disabled and (self.belt_intaking or self.belt_ejecting)
