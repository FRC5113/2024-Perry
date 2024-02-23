from wpilib import MotorControllerGroup, DutyCycleEncoder
from wpilib.interfaces import MotorController
from magicbot import tunable, will_reset_to


class Intake:
    """Component that controls the two motors that power the lifting
    and lowering of the intake module
    """

    left_motor: MotorController
    right_motor: MotorController
    left_encoder: DutyCycleEncoder
    right_encoder: DutyCycleEncoder
    left_encoder_offset: float
    right_encoder_offset: float
    encoder_error_threshold: float
    lower_limit: float
    upper_limit: float

    speed = will_reset_to(0)
    position = 0
    error_flag = False
    suppress_errors = False

    def setup(self):
        self.right_motor.setInverted(True)
        self.motor_group = MotorControllerGroup(self.left_motor, self.right_motor)

    def get_position(self) -> float | None:
        """Returns the average position between the two encoders.
        If the difference between the two is greater than the given
        threshold, this function will return `None`, signifying an
        error. Call this function every loop to update the position.
        """
        a = self.left_encoder.getAbsolutePosition() - self.left_encoder_offset
        # the position is negated because the right encoder is reversed
        b = 1 - self.right_encoder.getAbsolutePosition() - self.right_encoder_offset
        position = None
        if abs(a - b) <= self.encoder_error_threshold:
            position = (a + b) / 2
        if (
            abs(a - b + 1) <= self.encoder_error_threshold
            or abs(a - b + 1) < self.encoder_error_threshold
        ):
            position = (a + b) / 2 - 0.5
            if position < 0:
                position += 1
        return position

    def set_speed(self, speed: float):
        assert -1.0 < speed < 1.0, f"Improper intake joint speed: {speed}"
        self.speed = speed

    def execute(self):
        self.position = self.get_position()
        if self.position is not None or self.suppress_errors:
            """this assumes that an increase in position corresponds
            with a positive speed, which is not at all guaranteed
            """
            if self.speed < 0 and self.position <= self.lower_limit:
                self.motor_group.set(0)
                return
            if self.speed > 0 and self.position >= self.upper_limit:
                self.motor_group.set(0)
                return
            self.motor_group.set(self.speed)
        else:
            self.motor_group.set(0)
            print("ERROR: INTAKE ENCODERS MISALIGNED")
