from enum import Enum
from typing import Callable

from wpilib.interfaces import MotorController
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.controls.duty_cycle_out import DutyCycleOut
from phoenix6.controls.voltage_out import VoltageOut
from phoenix6.signals import InvertedValue, NeutralModeValue
from rev import CANSparkBase


def clamp(value: float, min_value: float, max_value: float) -> float:
    """Restrict value between min_value and max_value."""
    return max(min(value, max_value), min_value)


def compensate(args: list[float | None]) -> float | None:
    """Finds the average of all non-None values in `args`"""
    values = list(filter(lambda i: i is not None, args))
    if len(values) == 0:
        return None
    return sum(values) / len(values)


def cyclic_distance(a: float, b: float, max_value: float = 1.0) -> float:
    """Calculates distance between two values (`a` and `b`) over a
    cyclic space. This is useful for things like encoders
    """
    return min(abs(a - b), max_value - abs(a - b))


def cyclic_contains(value: float, a: float, b: float, tolerance: float = 0.001) -> bool:
    """Determines if a value is between two bounds (`a` and `b`) over a
    cyclic space. This assumes that the two bounds are less than half
    a rotation apart in the real world.
    """
    return (
        cyclic_distance(value, a) + cyclic_distance(value, b)
        <= cyclic_distance(a, b) + tolerance
    )


def cyclic_average(a: float, b: float) -> float:
    if abs(a - b) <= 0.5:
        return (a + b) / 2.0
    return ((a + b) / 2.0 + 0.5) % 1


def curve(
    mapping: Callable[[float], float],
    offset: float,
    deadband: float,
    max_mag: float,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    """Return a function that applies a curve to an input.

    Arguments:
    mapping -- maps input to output
    offset -- added to output, even if the input is deadbanded
    deadband -- when the input magnitude is less than this,
        the input is treated as zero
    max_mag -- restricts the output magnitude to a maximum.
        If this is 0, no restriction is applied.
    absolute_offset -- If true, applies offset always (even when deadbanded),
        If false, adds sign(input_val) * offset or 0 in the deadband
    """

    def f(input_val: float) -> float:
        """Apply a curve to an input. Be sure to call this function to get an output, not curve."""
        if abs(input_val) < deadband:
            return offset if absolute_offset else 0
        applied_offset = (1 if absolute_offset else abs(input_val) / input_val) * offset
        output_val = mapping(input_val) + applied_offset
        if max_mag == 0:
            return output_val
        else:
            return clamp(output_val, -max_mag, max_mag)

    return f


def linear_curve(
    scalar: float = 1.0,
    offset: float = 0.0,
    deadband: float = 0.0,
    max_mag: float = 0.0,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    return curve(lambda x: scalar * x, offset, deadband, max_mag, absolute_offset)


def ollie_curve(
    scalar: float = 1.0,
    offset: float = 0.0,
    deadband: float = 0.0,
    max_mag: float = 0.0,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    return curve(
        lambda x: scalar * x * abs(x), offset, deadband, max_mag, absolute_offset
    )


def cubic_curve(
    scalar: float = 1.0,
    offset: float = 0.0,
    deadband: float = 0.0,
    max_mag: float = 0.0,
    absolute_offset: bool = True,
) -> Callable[[float], float]:
    return curve(lambda x: scalar * x**3, offset, deadband, max_mag, absolute_offset)


class EmptyController(MotorController):
    """Dummy class that implements wpilib MotorController.
    Only use this for testing.
    """

    def disable(self):
        pass

    def get(self):
        return 0

    def getInverted(self):
        return False

    def set(self, speed):
        pass

    def setIdleMode(self, mode):
        pass

    def setInverted(self, isInverted):
        pass

    def setVoltage(self, volts):
        pass

    def stopMotor(self):
        pass


class WPI_TalonFX(TalonFX, MotorController):
    """Wrapper for the phoenix6 TalonFX that implements
    the wpilib MotorController interface, making it possible
    to use TalonFX controllers in, for example, MotorControllerGroup
    and DifferentialDrive
    """

    def __init__(self, device_id: int, canbus: str = "", enable_foc: bool = False):
        TalonFX.__init__(self, device_id, canbus=canbus)
        MotorController.__init__(self)
        self.config = TalonFXConfiguration()
        self.duty_cycle_out = DutyCycleOut(0, enable_foc=enable_foc)
        self.voltage_out = VoltageOut(0, enable_foc=enable_foc)
        self.is_disabled = False

    def disable(self):
        self.stopMotor()
        self.is_disabled = True

    def get(self) -> float:
        return self.duty_cycle_out.output

    def getInverted(self) -> bool:
        return (
            self.config.motor_output.inverted
            == InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

    def set(self, speed: float):
        if not self.is_disabled:
            self.duty_cycle_out.output = speed
            self.set_control(self.duty_cycle_out)

    def setIdleMode(self, mode: NeutralModeValue):
        """Set the idle mode setting

        Arguments:
        mode -- Idle mode (coast or brake)
        """
        self.config.motor_output.neutral_mode = mode
        self.configurator.apply(self.config)  # type: ignore

    def setInverted(self, isInverted: bool):
        if isInverted:
            self.config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        else:
            self.config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.configurator.apply(self.config)  # type: ignore

    def setVoltage(self, volts: float):
        if not self.is_disabled:
            self.voltage_out.output = volts
            self.set_control(self.voltage_out)

    def stopMotor(self):
        self.set(0)
