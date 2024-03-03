import wpilib


class OI_Base:
    def drive_forward(self) -> float:
        return 0

    def drive_turn(self) -> float:
        return 0

    def shoot(self) -> bool:
        return False

    def intake(self) -> bool:
        return False

    def eject(self) -> bool:
        return False

    def feed(self) -> bool:
        return False

    def intake_up(self) -> bool:
        return False

    def intake_down(self) -> bool:
        return False

    def move_intake(self) -> bool:
        return False


class Double_OI(OI_Base):
    def __init__(self, xbox_port: int = 0, joystick_port: int = 1):
        self.xbox = wpilib.XboxController(xbox_port)
        self.joystick = wpilib.Joystick(joystick_port)

    def drive_forward(self):
        return self.xbox.getLeftY()

    def drive_turn(self):
        return -self.xbox.getLeftX()

    def shoot(self):
        return self.joystick.getRawButton(7)

    def intake(self):
        return self.joystick.getRawButton(9)

    def feed(self):
        return self.joystick.getRawButton(8)

    def eject(self):
        return self.joystick.getRawButton(10)

    def intake_up(self):
        return self.joystick.getRawButton(11)

    def intake_down(self):
        return self.joystick.getRawButton(12)

    def move_intake(self):
        return self.joystick.getTrigger()


class Joystick_OI(OI_Base):
    def __init__(self, joystick_port: int = 1):
        self.joystick = wpilib.Joystick(joystick_port)

    def drive_forward(self):
        return self.joystick.getY()

    def drive_turn(self):
        return -self.joystick.getX()

    def shoot(self):
        return self.joystick.getRawButton(7)

    def intake(self):
        return self.joystick.getRawButton(9)

    def feed(self):
        return self.joystick.getRawButton(8)

    def eject(self):
        return self.joystick.getRawButton(10)

    def intake_up(self):
        return self.joystick.getRawButton(11)

    def intake_down(self):
        return self.joystick.getRawButton(12)

    def move_intake(self):
        return self.joystick.getTrigger()


class Comp_OI(OI_Base):
    """Theoretical OI scheme for comp, tailored to the two FSMs"""

    def __init__(
        self, xbox_port: int = 0, joystick_port: int = 1, deadband: float = 0.1
    ):
        self.xbox = wpilib.XboxController(xbox_port)
        self.joystick = wpilib.Joystick(joystick_port)
        self.deadband = deadband

    def drive_forward(self):
        return self.xbox.getLeftY()

    def drive_turn(self):
        return -self.xbox.getLeftX()

    def shoot(self):
        return self.joystick.getTrigger()

    def intake(self):
        return self.joystick.getY() > self.deadband

    def eject(self):
        return self.joystick.getX() < -self.deadband

    def intake_up(self):
        return self.joystick.getRawButton(3)

    def intake_down(self):
        return self.joystick.getRawButton(4)