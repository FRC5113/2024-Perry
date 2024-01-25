import wpilib
from wpilib.interfaces import MotorController
from magicbot import will_reset_to


class Shooter:
    left_motor: MotorController
    right_motor: MotorController

    speed = will_reset_to(0)

    def setup(self):
        self.right_motor.setInverted(True)

    def spin(self, speed: float):
        if not (-1.0 <= speed <= 1.0):
            raise Exception(f"Improper value for speed entered: {speed}")
        self.speed = speed

    def execute(self):
        self.left_motor.set(self.speed)
        self.right_motor.set(self.speed)
