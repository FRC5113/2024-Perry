from wpilib import MotorControllerGroup
from magicbot import will_reset_to, tunable
from rev import CANSparkBase, CANSparkMax

class Climber:
    left_motor: CANSparkMax
    right_motor: CANSparkMax

    contracting = will_reset_to(False)
    extending = will_reset_to(False)
    speed = tunable(0.6)

    def setup(self):
        self.left_motor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        self.right_motor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        self.right_motor.setInverted(True)
        self.motor_group = MotorControllerGroup(self.left_motor, self.right_motor)

    def contract(self):
        self.contracting = True

    def extend(self):
        self.extending = True

    def execute(self):
        if self.contracting and self.extending:
            self.contracting = False

        if self.contracting:
            self.motor_group.set(-self.speed)
        elif self.extending:
            self.motor_group.set(self.speed)
        else:
            self.motor_group.set(0)
