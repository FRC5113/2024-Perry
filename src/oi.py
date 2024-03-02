import wpilib


class OI:
    def __init__(self):
        self.xbox = wpilib.XboxController(0)
        self.joystick = wpilib.Joystick(1)

        # OI.py
    def drive_forward(self):
        return self.xbox.getRightY()
    
    def drive_turn(self):
        return -self.xbox.getRightX()
    
    def shoot(self):
        return self.joystick.getRawButton(7)

    def do_intake(self):
        return self.joystick.getRawButton(9)

    def feed(self):
        return self.joystick.getRawButton(8)

    def eject(self):
        return self.joystick.getRawButton(10)

    def intake_down(self):
        return self.joystick.getRawButton(11)
    
    def intake_up(self):
        return self.joystick.getRawButton(12)