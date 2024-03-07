from phoenix5 import NeutralMode, WPI_TalonSRX
from wpilib import MotorControllerGroup
from wpilib.interfaces import MotorController
from magicbot import will_reset_to, tunable


class Climber:
    left_motor: WPI_TalonSRX
    right_motor: WPI_TalonSRX

    contracting_left = will_reset_to(False)
    contracting_right = will_reset_to(False)
    extending_left = will_reset_to(False)
    extending_right = will_reset_to(False)
    speed = tunable(0.3)

    def setup(self):
        self.left_motor.setNeutralMode(NeutralMode.Brake)
        self.right_motor.setNeutralMode(NeutralMode.Brake)

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
        self.extending_left = True
        self.extending_right = True

    def execute(self):
        if self.contracting_left and self.extending_left:
            self.extending_left = False
        if self.contracting_right and self.extending_right:
            self.extending_right = False

        if self.contracting_left:
            self.left_motor.set(-0.3)
        elif self.extending_left:
            self.left_motor.set(0.3)
        else:
            self.left_motor.set(0)

        if self.contracting_right:
            self.right_motor.set(0.3)
        elif self.extending_right:
            self.right_motor.set(-0.3)
        else:
            self.right_motor.set(0)
