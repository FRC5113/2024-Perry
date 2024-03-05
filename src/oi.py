import wpilib


class OI:
    def __init__(self):
        self.xbox = wpilib.XboxController(0)
        self.xbox_secondary = wpilib.XboxController(0)
        # self.joystick = wpilib.Joystick(1)

        # OI.py
    def drive_forward(self):
        return self.xbox.getRightY()
    
    def drive_turn(self):
        return -self.xbox.getRightX()
    
    def shoot(self):
        return self.xbox_secondary.getXButton()

    def do_intake(self):
        return self.xbox_secondary.getAButton()

    def feed(self):
        return False

    def eject(self):
        return False

    def intake_down(self):
        return self.xbox_secondary.getRightBumper()
    
    def intake_up(self):
        return self.xbox_secondary.getLeftBumper()