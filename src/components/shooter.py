from wpilib import MotorControllerGroup
from wpilib.interfaces import MotorController
from wpimath.filter import MedianFilter
from magicbot import tunable, will_reset_to, feedback
from rev import CANSparkMax


class Shooter:
    """Component that controls the two motors that shoot the notes"""

    belt_motor: CANSparkMax
    feed_left_motor: MotorController
    feed_right_motor: MotorController
    shooter_left_motor: MotorController
    shooter_right_motor: MotorController

    belt_intaking = will_reset_to(False)
    belt_ejecting = will_reset_to(False)
    belt_speed = tunable(0.3)
    feeding_in = will_reset_to(False)
    feeding_out = will_reset_to(False)
    feed_speed = tunable(-0.3)
    shooter_enabled = will_reset_to(False)
    shooter_speed = tunable(1)
    note_detection_threshold = tunable(3000)
    motor_speed_filter = MedianFilter(5)

    def setup(self):
        self.feed_right_motor.setInverted(True)
        self.feed_motor_group = MotorControllerGroup(
            self.feed_left_motor, self.feed_right_motor
        )
        self.shooter_right_motor.setInverted(True)
        self.shooter_motor_group = MotorControllerGroup(
            self.shooter_left_motor, self.shooter_right_motor
        )
        self.belt_encoder = self.belt_motor.getEncoder()

    @feedback
    def get_filtered_motor_speed(self):
        return self.motor_speed_filter.calculate(self.belt_encoder.getVelocity())

    def has_note(self) -> bool:
        """Returns `True` if a note is detected in the belt. This is
        accomplished by looking at the velocity of the belt motor, as it
        should drop when a note enters the belt. Note that this may be
        inaccurate when the motor starts running.
        """
        return abs(self.get_filtered_motor_speed()) < self.note_detection_threshold

    def intake(self):
        self.belt_intaking = True

    def eject(self):
        self.belt_ejecting = True

    def shoot(self):
        self.shooter_enabled = True

    def feed_in(self):
        self.feeding_in = True

    def feed_out(self):
        self.feeding_out = True

    def execute(self):
        if self.belt_intaking and self.belt_ejecting:
            self.belt_intaking = False
        if self.feeding_in and self.feeding_out:
            self.feeding_in = False

        if self.belt_intaking:
            self.belt_motor.set(self.belt_speed)
        elif self.belt_ejecting:
            self.belt_motor.set(-self.belt_speed)
        else:
            self.belt_motor.set(0)

        if self.feeding_in:
            self.feed_motor_group.set(self.feed_speed)
        elif self.feeding_out:
            self.feed_motor_group.set(-self.feed_speed * 0.3)
        else:
            self.feed_motor_group.set(0)

        if self.shooter_enabled:
            self.shooter_motor_group.set(self.shooter_speed)
        else:
            self.shooter_motor_group.set(0)

    @feedback
    def get_motor_speed(self):
        return abs(self.belt_encoder.getVelocity())

    @feedback
    def get_has_note(self):
        return self.has_note()
