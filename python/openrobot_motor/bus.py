"""CAN bus access over python-can: slcan, PCAN, or the built-in simulator.

    bus = MotorBus.open("slcan", "COM5")            # Windows
    bus = MotorBus.open("slcan", "/dev/ttyACM0")    # Linux
    bus = MotorBus.open("slcan", "/dev/tty.usbmodem1101")   # macOS
    bus = MotorBus.open("pcan", "PCAN_USBBUS1")
    bus = MotorBus.open("socketcan", "can0")       # Linux, bit rate set by ip link
    bus = MotorBus.open("sim")                      # no hardware
"""
import threading
import time

import can

from . import protocol as P

SIM_CHANNEL = "openrobot_sim"


class MotorError(Exception):
    pass


class ReplyTimeout(MotorError):
    pass


class MotorBus:
    """One CAN adapter. Sends a request and waits for the matching reply.

    Fault notices (0xBF) that arrive while waiting are kept in `faults` and
    passed to `on_fault` if set."""

    def __init__(self, bus: can.BusABC, sim=None):
        self.bus = bus
        self.sim = sim
        self.faults = []            # (motor_id, fault_code, time)
        self.on_fault = None
        self._lock = threading.Lock()

    @classmethod
    def open(cls, interface: str, channel: str = None,
             bitrate: int = P.DEFAULT_BITRATE, sim_ids=(1,)):
        """interface: "slcan", "pcan", "socketcan" or "sim". sim_ids: motor
        IDs the simulator answers to."""
        if interface == "sim":
            from .sim import SimBoards
            sim = SimBoards(SIM_CHANNEL, sim_ids)
            bus = can.Bus(interface="virtual", channel=SIM_CHANNEL,
                          receive_own_messages=False)
            return cls(bus, sim)
        if channel is None and interface == "pcan":
            channel = "PCAN_USBBUS1"            # the first PCAN adapter
        if channel is None:
            raise MotorError("a channel is required for %s, e.g. COM5, "
                             "/dev/ttyACM0 or PCAN_USBBUS1" % interface)
        kw = dict(interface=interface, channel=channel, receive_own_messages=False)
        if interface != "socketcan":            # socketcan: set by `ip link`
            kw["bitrate"] = bitrate
        last = None
        # An slcan port sometimes refuses the first open right after another
        # program closed it; a short retry clears that.
        for _ in range(3):
            try:
                return cls(can.Bus(**kw))
            except Exception as ex:
                last = ex
                time.sleep(0.5)
        raise MotorError("cannot open %s %s: %s (is another program using "
                         "the adapter?)" % (interface, channel, last))

    def close(self):
        try:
            self.bus.shutdown()
        finally:
            if self.sim:
                self.sim.stop()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------ io --

    def send(self, motor_id: int, data: bytes):
        self.bus.send(can.Message(arbitration_id=P.can_id(motor_id), data=data,
                                  is_extended_id=False))

    def request(self, motor_id: int, data: bytes, timeout: float = 0.02,
                retries: int = 1) -> bytes:
        """Send `data` to `motor_id` and return the reply payload."""
        want = P.REPLY_CODE.get(data[0], data[0])
        with self._lock:
            for _ in range(retries + 1):
                self._drain()
                self.send(motor_id, data)
                reply = self._wait(motor_id, want, timeout)
                if reply is not None:
                    return reply
        raise ReplyTimeout("motor %d: no reply to 0x%02X" % (motor_id, data[0]))

    def _wait(self, motor_id, code, timeout):
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            if left <= 0:
                return None
            msg = self.bus.recv(left)
            if msg is None:
                return None
            data = self._accept(msg)
            if data is None:
                continue
            if msg.arbitration_id == P.can_id(motor_id) and data[0] == code:
                return data

    def _accept(self, msg):
        """Filter one received frame. Returns its payload, or None if it is
        not a reply (own echo, extended frame, fault notice)."""
        if msg.is_extended_id or msg.is_error_frame or msg.dlc < 8:
            return None
        if not getattr(msg, "is_rx", True):
            return None                         # own frame echoed back
        data = bytes(msg.data)
        if data[0] == P.FAULT_NOTICE:
            mid = msg.arbitration_id - P.BASE_ID
            self.faults.append((mid, data[1], time.time()))
            if self.on_fault:
                self.on_fault(mid, data[1])
            return None
        return data

    def _drain(self):
        """Drop stale frames, still recording fault notices."""
        while True:
            msg = self.bus.recv(0)
            if msg is None:
                return
            self._accept(msg)

    def poll_faults(self):
        """Check for fault notices without sending anything."""
        with self._lock:
            self._drain()
        return self.faults

    def scan(self, ids=range(1, 33), timeout: float = 0.01):
        """Motor IDs that answer a harmless read (0x92)."""
        found = []
        for i in ids:
            try:
                self.request(i, P.request(P.READ_MULTI_TURN), timeout, retries=0)
                found.append(i)
            except ReplyTimeout:
                pass
        return found
