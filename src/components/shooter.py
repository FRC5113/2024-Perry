from wpilib import MotorControllerGroup
from wpilib.interfaces import MotorController
from magicbot import tunable


class Shooter:
    """Component that controls the two motors that shoot the notes"""

    left_motor: MotorController
    right_motor: MotorController

    enabled = False
    shoot_speed = tunable(1)

    def setup(self):
        self.right_motor.setInverted(True)
        self.motor_group = MotorControllerGroup(self.left_motor, self.right_motor)

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def execute(self):
        if self.enabled:
            self.motor_group.set(self.shoot_speed)
        else:
            self.motor_group.set(0)
