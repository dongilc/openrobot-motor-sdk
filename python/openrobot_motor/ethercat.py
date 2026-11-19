"""OpenRobot motor controllers over EtherCAT, with pysoem as the master.

    from openrobot_motor.ethercat import EcatBus

    with EcatBus.open() as bus:           # first adapter that has OpenRobot slaves
        m = bus.motors[0]
        m.speed_dps(360)                  # taken by the next PDO cycle
        print(m.status())
    # leaving the block sends Motor off to every slave and stops the cycle

A background thread exchanges the process data every `period` seconds. The
methods of EcatMotor only set what the next cycle sends; status() returns what
the last cycle received. Layout and units: docs/ethercat_protocol.md.

Needs `pip install pysoem`; on Windows also Npcap (https://npcap.com),
on Linux root or CAP_NET_RAW.
"""
import struct
import sys
import threading
import time
from dataclasses import dataclass

from . import protocol as P

VENDOR_ID = 0x00524F42
PRODUCT_CODE = 0x00000001

RX_FORMAT = "<BiiiHH"          # 17 bytes, master -> slave
TX_FORMAT = "<BBiiiHbbH"       # 20 bytes, slave -> master
SCALE = 10000.0


class EcatError(Exception):
    pass


# ------------------------------------------------------------ PDO codec --

def _i32(x: float) -> int:
    return int(max(-2**31, min(2**31 - 1, round(x * SCALE))))


def _u16(x: float, scale: float) -> int:
    return int(max(0, min(0xFFFF, round(x * scale))))


def pack_rx(control_mode: int, position: float = 0.0, velocity: float = 0.0,
            torque: float = 0.0, kp: float = 0.0, kd: float = 0.0) -> bytes:
    """RxPDO. position/velocity/torque in the unit of the mode (x10000 on the
    wire), kp x100, kd x1000."""
    return struct.pack(RX_FORMAT, control_mode, _i32(position), _i32(velocity), _i32(torque),
                       _u16(kp, 100.0), _u16(kd, 1000.0))


@dataclass
class EcatStatus:
    """TxPDO. Motor side, except the position in MIT mode."""
    control_mode: int
    fault_code: int
    position_deg: float      # motor multi-turn; in MIT mode output side from the MIT zero
    velocity_dps: float      # motor side, every mode
    current_a: float
    bus_voltage: float
    temp_motor_c: int
    temp_mosfet_c: int
    encoder_raw: int         # 14 bit

    @property
    def mode_name(self) -> str:
        return MODE_NAMES.get(self.control_mode, "0x%02X" % self.control_mode)

    @property
    def fault_name(self) -> str:
        return P.fault_name(self.fault_code)


MODE_NAMES = {0: "none", P.MOTOR_OFF: "off", P.MOTOR_STOP: "stop", P.MOTOR_START: "start",
              P.CURRENT: "current", P.SPEED: "speed (DPS)", P.POSITION: "position",
              P.POSITION_LIMITED: "position with speed limit", P.DUTY: "duty", P.MIT_CONTROL: "MIT"}


def unpack_tx(data: bytes) -> EcatStatus:
    mode, fault, pos, vel, cur, vbus, tm, tf, enc = struct.unpack(TX_FORMAT, bytes(data[:20]))
    return EcatStatus(mode, fault, pos / SCALE, vel / SCALE, cur / SCALE, vbus / 100.0, tm, tf, enc)


# ------------------------------------------------------------ one motor --

class EcatMotor:
    """One slave. Methods set the command of the next cycles."""

    def __init__(self, bus: "EcatBus", index: int, slave):
        self.bus = bus
        self.index = index
        self.slave = slave
        self._out = pack_rx(P.MOTOR_OFF)
        self._in = None

    # Commands. All values motor side unless noted.
    def _set(self, frame: bytes):
        self._out = frame

    def off(self):
        """Release; also clears a Big Error fault."""
        self._set(pack_rx(P.MOTOR_OFF))

    def stop(self):
        """Speed (DPS) 0: hold the present position."""
        self._set(pack_rx(P.MOTOR_STOP))

    def current(self, amps: float, damping: float = 0.0):
        self._set(pack_rx(P.CURRENT, torque=amps, kp=max(0.0, min(100.0, damping))))

    def speed_dps(self, dps: float):
        """Speed (DPS, position-tracked), deg/s. eRPM is not available over EtherCAT."""
        self._set(pack_rx(P.SPEED, velocity=dps))

    def position(self, deg: float, max_dps: float):
        """Multi-turn motor angle with a speed limit of 1..25000 deg/s (0xA4)."""
        if not 1 <= max_dps <= P.SPEED_LIMIT_MAX:
            raise ValueError("speed limit must be 1..%d deg/s" % P.SPEED_LIMIT_MAX)
        self._set(pack_rx(P.POSITION_LIMITED, position=deg, velocity=max_dps))

    def position_direct(self, deg: float):
        """Multi-turn motor angle, no speed limit (0xA3). Big Error checked."""
        self._set(pack_rx(P.POSITION, position=deg))

    def duty(self, fraction: float):
        self._set(pack_rx(P.DUTY, torque=max(-1.0, min(1.0, fraction))))

    def mit(self, p_rad: float, v_rads: float, kp: float, kd: float, t_nm: float = 0.0):
        """MIT, output side: torque = kp (p - p_act) + kd (v - v_act) + t_nm."""
        self._set(pack_rx(P.MIT_CONTROL, p_rad, v_rads, t_nm, kp, kd))

    # State
    def status(self) -> EcatStatus:
        """What the slave reported in the last cycle."""
        if self._in is None:
            raise EcatError("no process data received yet")
        return unpack_tx(self._in)

    # SDO (mailbox), usable while the cycle runs
    def sdo_read(self, index: int, sub: int, fmt: str):
        return struct.unpack(fmt, self.slave.sdo_read(index, sub))[0]

    def sdo_write(self, index: int, sub: int, fmt: str, value):
        self.slave.sdo_write(index, sub, struct.pack(fmt, value))

    @property
    def motor_id(self) -> int:
        return self.sdo_read(0x2070, 3, "<B")

    @property
    def gear_ratio(self) -> float:
        return self.sdo_read(0x2050, 4, "<f")

    @property
    def kt_output(self) -> float:
        """Output-side Kt, Nm/A."""
        return self.sdo_read(0x2020, 3, "<f")

    def mit_zero(self):
        """The present output position becomes 0 rad for MIT (not stored)."""
        self.sdo_write(0x2050, 5, "<B", 1)

    def clear_fault(self):
        self.sdo_write(0x2030, 1, "<B", 0)


# ------------------------------------------------------------ the bus --

def _adapters():
    import pysoem
    return list(pysoem.find_adapters())


def list_adapters():
    """(name, description) of the network adapters pysoem can open."""
    return [(a.name, a.desc.decode(errors="replace") if isinstance(a.desc, bytes) else a.desc)
            for a in _adapters()]


class EcatBus:
    """EtherCAT master for the OpenRobot slaves on one network adapter."""

    def __init__(self, master, period: float):
        self.master = master
        self.period = period
        self.motors = [EcatMotor(self, i, s) for i, s in enumerate(master.slaves)]
        self.cycles = 0
        self.wkc_errors = 0           # cycles where not every slave answered
        self.max_cycle_s = 0.0        # longest gap between cycles
        # Set to a list to record every cycle: (time s, slave, sent RxPDO,
        # received TxPDO or None). For diagnosis; grows by one entry per
        # slave and cycle.
        self.log = None
        self._stop = threading.Event()
        self._thread = None

    @classmethod
    def open(cls, adapter: str = None, period: float = 0.002) -> "EcatBus":
        """adapter: pysoem adapter name, or part of its description (e.g.
        "Ethernet"), or None for the first adapter with OpenRobot slaves.
        period: PDO cycle in seconds."""
        import pysoem
        names = []
        for name, desc in list_adapters():
            if adapter is None or adapter == name or adapter.lower() in desc.lower():
                names.append(name)
        if not names:
            raise EcatError("no network adapter matches %r; see list_adapters()" % adapter)
        last = "no slaves found"
        for name in names:
            m = pysoem.Master()
            m.always_release_gil = True
            try:
                m.open(name)
            except Exception as ex:          # adapter cannot be opened (Npcap, permissions)
                last = str(ex)
                continue
            n = m.config_init()
            if n > 0 and all(s.man == VENDOR_ID for s in m.slaves):
                bus = cls(m, period)
                bus._bring_up()
                return bus
            if n > 0:
                last = "slaves on %s are not OpenRobot controllers" % name
            m.close()
        raise EcatError("cannot open an EtherCAT bus: %s" % last)

    def _bring_up(self):
        import pysoem
        m = self.master
        # Some slaves stay in INIT after the scan: ask for PRE-OP explicitly.
        m.state = pysoem.PREOP_STATE
        m.write_state()
        if m.state_check(pysoem.PREOP_STATE, 3_000_000) != pysoem.PREOP_STATE:
            raise EcatError("slaves did not reach PRE-OP")
        m.config_map()
        if m.state_check(pysoem.SAFEOP_STATE, 3_000_000) != pysoem.SAFEOP_STATE:
            raise EcatError("slaves did not reach SAFE-OP")
        self._thread = threading.Thread(target=self._run, name="ecat-cycle", daemon=True)
        self._thread.start()
        m.state = pysoem.OP_STATE
        m.write_state()
        if m.state_check(pysoem.OP_STATE, 5_000_000) != pysoem.OP_STATE:
            self.close()
            raise EcatError("slaves did not reach OP")
        t0 = time.monotonic()
        while self.cycles < 5 and time.monotonic() - t0 < 1.0:
            time.sleep(0.01)

    def _run(self):
        m = self.master
        win = sys.platform == "win32"
        if win:                          # 1 ms sleeps instead of 15.6 ms
            import ctypes
            ctypes.windll.winmm.timeBeginPeriod(1)
        try:
            next_t = time.perf_counter()
            last = next_t
            while not self._stop.is_set():
                for mo in self.motors:
                    mo.slave.output = mo._out
                m.send_processdata()
                wkc = m.receive_processdata(2000)
                ok = wkc >= m.expected_wkc
                if ok:
                    for mo in self.motors:
                        mo._in = bytes(mo.slave.input)
                else:
                    self.wkc_errors += 1
                if self.log is not None:
                    t = time.perf_counter()
                    for mo in self.motors:
                        self.log.append((t, mo.index, mo._out, mo._in if ok else None))
                self.cycles += 1
                now = time.perf_counter()
                self.max_cycle_s = max(self.max_cycle_s, now - last) if self.cycles > 10 else 0.0
                last = now
                next_t += self.period
                delay = next_t - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_t = time.perf_counter()     # fell behind: do not try to catch up
        finally:
            if win:
                ctypes.windll.winmm.timeEndPeriod(1)

    def close(self):
        """Motor off to every slave for a few cycles, then INIT and close."""
        import pysoem
        if self._thread is not None and self._thread.is_alive():
            for mo in self.motors:
                mo.off()
            start = self.cycles
            t0 = time.monotonic()
            while self.cycles - start < 10 and time.monotonic() - t0 < 0.5:
                time.sleep(self.period)
            self._stop.set()
            self._thread.join(1.0)
        try:
            self.master.state = pysoem.INIT_STATE
            self.master.write_state()
        finally:
            self.master.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
