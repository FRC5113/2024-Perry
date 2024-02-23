from wpilib import MotorControllerGroup
from wpilib.interfaces import MotorController
from magicbot import tunable, will_reset_to


class Indexer:
    """Component that controls the two motors that feed notes into the
    shooter (and later the two motors that control the indexer belt)
    """

    feed_left_motor: MotorController
    feed_right_motor: MotorController
    belt_motor: MotorController

    feed_enabled = will_reset_to(False)
    feed_speed = tunable(-0.3)
    belt_enabled = will_reset_to(False)
    belt_speed = -0.1

    def setup(self):
        self.feed_right_motor.setInverted(True)
        self.feed_motor_group = MotorControllerGroup(
            self.feed_left_motor, self.feed_right_motor
        )

    def enable_feed(self):
        self.feed_enabled = True

    def disable_feed(self):
        self.feed_enabled = False

    def set_belt_speed(self, speed: float):
        self.belt_speed = speed

    def enable_belt(self):
        self.belt_enabled = True

    def disable_belt(self):
        self.belt_enabled = False

    def execute(self):
        if self.feed_enabled:
            self.feed_motor_group.set(self.feed_speed)
        else:
            self.feed_motor_group.set(0)
        if self.belt_enabled:
            self.belt_motor.set(self.belt_speed)
        else:
            self.belt_motor.set(0)
