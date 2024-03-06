from wpilib import MotorControllerGroup
from wpilib.interfaces import MotorController
from magicbot import will_reset_to, tunable


class Climber:
    left_motor: MotorController
    right_motor: MotorController

    contracting = will_reset_to(False)
    extending = will_reset_to(False)
    speed = tunable(0.3)

    def setup(self):
        self.motor_group = MotorControllerGroup(self.left_motor, self.right_motor)

    def contract(self):
        self.contracting = True

    def extend(self):
        self.extending = True

    def execute(self):
        if self.contracting and self.extending:
            self.extending = False
        if self.contracting:
            self.motor_group.set(-0.3)
        elif self.extending:
            self.motor_group.set(0.3)
        else:
            self.motor_group.set(0)