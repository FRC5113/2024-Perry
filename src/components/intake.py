from wpilib import MotorControllerGroup
from wpilib.interfaces import MotorController
from magicbot import tunable, will_reset_to


class Intake:
    """Component that controls the two motors that shoot the notes"""

    joint_left_motor: MotorController
    joint_right_motor: MotorController

    joint_speed = will_reset_to(0)

    def setup(self):
        self.joint_right_motor.setInverted(True)
        self.joint_motor_group = MotorControllerGroup(
            self.joint_left_motor, self.joint_right_motor
        )

    def set_joint_speed(self, speed: float):
        assert -1.0 < speed < 1.0, f"Improper intake joint speed: {speed}"
        self.joint_speed = speed

    def execute(self):
        self.joint_motor_group.set(self.joint_speed)
