import wpilib


class OI_Base:
    def drive_forward(self) -> float:
        return 0

    def drive_turn(self) -> float:
        return 0

    def soft_shoot(self) -> bool:
        """Only shoots if predicted to be able to score"""
        return False

    def hard_shoot(self) -> bool:
        """Shoots regardless of scoring prediction"""
        return False

    def intake(self) -> bool:
        return False

    def eject(self) -> bool:
        return False

    def intake_up(self) -> bool:
        return False

    def intake_down(self) -> bool:
        return False

    def move_intake(self) -> bool:
        return False

    def align(self) -> bool:
        return False

    def cancel_align(self) -> bool:
        return False

    def contract_left_climber(self) -> bool:
        return False

    def contract_right_climber(self) -> bool:
        return False

    def extend_left_climber(self) -> bool:
        return False

    def extend_right_climber(self) -> bool:
        return False


class Double_Xbox_OI(OI_Base):
    def __init__(
        self, xbox_port_1: int = 0, xbox_port_2: int = 1, deadband: float = 0.1
    ):
        self.xbox1 = wpilib.XboxController(xbox_port_1)
        self.xbox2 = wpilib.XboxController(xbox_port_2)
        self.deadband = deadband

    def drive_forward(self):
        return self.xbox1.getLeftY()

    def drive_turn(self):
        return -self.xbox1.getRightX()

    def soft_shoot(self):
        return self.xbox2.getBButton()

    def hard_shoot(self):
        return self.xbox2.getBButton() and self.xbox2.getAButton()

    def intake(self):
        return self.xbox2.getXButton()

    def eject(self):
        return self.xbox2.getYButton()

    def intake_up(self):
        return False

    def intake_down(self):
        return self.xbox2.getLeftY() > self.deadband

    def move_intake(self):
        return False

    def align(self):
        return self.xbox1.getXButton()

    def cancel_align(self):
        return self.xbox1.getBButton()

    def contract_left_climber(self):
        return self.xbox2.getLeftBumper()

    def contract_right_climber(self):
        return self.xbox2.getRightBumper()

    def extend_left_climber(self):
        return self.xbox2.getLeftTriggerAxis() > self.deadband

    def extend_right_climber(self):
        return self.xbox2.getRightTriggerAxis() > self.deadband


class Joystick_OI(OI_Base):
    def __init__(self, joystick_port: int = 1):
        self.joystick = wpilib.Joystick(joystick_port)

    def drive_forward(self):
        return self.joystick.getY()

    def drive_turn(self):
        return -self.joystick.getX()

    def soft_shoot(self):
        return self.joystick.getRawButton(7)

    def intake(self):
        return self.joystick.getRawButton(9)

    def eject(self):
        return self.joystick.getRawButton(10)

    def intake_up(self):
        return False

    def intake_down(self):
        return self.joystick.getRawButton(12)

    def move_intake(self):
        return False


class Sim_OI(OI_Base):
    def __init__(self, keyboard_port_left: int = 0, keyboard_port_right: int = 1):
        self.keyboard_left = wpilib.Joystick(keyboard_port_left)
        self.keyboard_right = wpilib.Joystick(keyboard_port_right)

    def drive_forward(self) -> float:
        return self.keyboard_left.getY()

    def drive_turn(self) -> float:
        return self.keyboard_left.getX()

    def soft_shoot(self) -> bool:
        """Only shoots if predicted to be able to score"""
        return self.keyboard_left.getRawButton(1)

    def hard_shoot(self) -> bool:
        """Shoots regardless of scoring prediction"""
        return self.keyboard_left.getRawButton(2)

    def intake(self) -> bool:
        return self.keyboard_left.getRawButton(3)

    def eject(self) -> bool:
        return self.keyboard_left.getRawButton(4)

    def intake_up(self) -> bool:
        return self.keyboard_right.getRawButton(1)

    def intake_down(self) -> bool:
        return False

    def move_intake(self) -> bool:
        return False

    def align(self) -> bool:
        return self.keyboard_right.getRawButton(2)

    def cancel_align(self) -> bool:
        return self.keyboard_right.getRawButton(3)

    def contract_left_climber(self) -> bool:
        return False

    def contract_right_climber(self) -> bool:
        return False

    def extend_left_climber(self) -> bool:
        return False

    def extend_right_climber(self) -> bool:
        return False
