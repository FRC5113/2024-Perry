from wpilib import MotorControllerGroup
from magicbot import will_reset_to, tunable, feedback
from rev import CANSparkBase, CANSparkMax


class Climber:
    left_motor: CANSparkMax
    right_motor: CANSparkMax
    left_lower_limit: float
    left_upper_limit: float
    right_lower_limit: float
    right_upper_limit: float

    contracting_right = will_reset_to(False)
    contracting_left = will_reset_to(False)
    extending_left = will_reset_to(False)
    extending_right = will_reset_to(False)
    ignoring_limits = will_reset_to(False)
    speed = tunable(1)

    def setup(self):
        self.left_motor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        self.right_motor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        self.left_encoder = self.left_motor.getEncoder()
        self.right_encoder = self.right_motor.getEncoder()

    @feedback
    def get_left_position(self):
        return self.left_encoder.getPosition()

    @feedback
    def get_right_position(self):
        return self.right_encoder.getPosition()

    def contract_left(self):
        self.contracting_left = True

    def contract_right(self):
        self.contracting_right = True

    def contract(self):
        self.contracting_left = True
        self.contracting_right = True

    def extend_left(self):
        self.extending_left = True

    def extend_right(self):
        self.extending_right = True

    def extend(self):
        self.extending_right = True
        self.extending_left = True

    def reset_encoders(self):
        self.left_encoder.setPosition(0)
        self.right_encoder.setPosition(0)

    def ignore_limits(self):
        self.ignoring_limits = True

    def execute(self):
        if self.contracting_left and self.extending_left:
            self.extending_left = False

        if self.contracting_right and self.extending_right:
            self.extending_right = False

        if self.extending_left and (
            self.get_left_position() > self.left_lower_limit or self.ignoring_limits
        ):
            self.left_motor.set(-self.speed)
        elif self.contracting_left and (
            self.get_left_position() < self.left_upper_limit or self.ignoring_limits
        ):
            self.left_motor.set(self.speed)
        else:
            self.left_motor.set(0)

        if self.contracting_right and (
            self.get_right_position() > self.right_lower_limit or self.ignoring_limits
        ):
            self.right_motor.set(-self.speed)
        elif self.extending_right and (
            self.get_right_position() < self.right_upper_limit or self.ignoring_limits
        ):
            self.right_motor.set(self.speed)
        else:
            self.right_motor.set(0)
