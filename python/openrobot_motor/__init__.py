"""OpenRobot motor controllers over CAN (slcan, PCAN, simulator)."""
from . import protocol
from .bus import MotorBus, MotorError, ReplyTimeout
from .motor import Motor
from .protocol import fault_name

__version__ = "0.1.0"
__all__ = ["MotorBus", "Motor", "MotorError", "ReplyTimeout", "protocol", "fault_name"]
