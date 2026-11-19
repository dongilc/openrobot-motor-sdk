"""OpenRobot CAN protocol: frame encoding and decoding, no I/O.

Every function here turns values into an 8-byte payload or back. The layout
follows docs/can_protocol.md; the bus layer (bus.py) does the sending.
"""
import struct
from dataclasses import dataclass

BASE_ID = 0x140
CURRENT_LSB = 33.0 / 2048.0          # A per LSB, current fields
ANGLE_14BIT = 16384                  # counts per turn, single-turn angles
DEFAULT_BITRATE = 1_000_000

# Command codes
MOTOR_OFF = 0x80
MOTOR_STOP = 0x81
MOTOR_START = 0x88
CURRENT = 0xA1
SPEED = 0xA2
POSITION = 0xA3
POSITION_LIMITED = 0xA4
DUTY = 0xA5
BRAKE = 0xA6
READ_STATUS = 0x9C
READ_PHASES = 0x9D
READ_MULTI_TURN = 0x92
READ_OUTPUT_ENCODER = 0x93
READ_ENCODER = 0x90
READ_ENCODER_STATUS = 0x9E
CLEAR_FAULT = 0x9B
READ_FAULTS = 0xB0
READ_PID = 0x30
WRITE_PID_RAM = 0x31
WRITE_PID_FLASH = 0x32
READ_LIMITS = 0xB1
WRITE_LIMITS_FLASH = 0xB2
WRITE_ENCODER_OFFSET = 0x91
SET_ZERO_HERE = 0x19
MIT_CONTROL = 0xC0
MIT_ENTER = 0xC1
MIT_EXIT = 0xC2
MIT_SET_ZERO = 0xC3
READ_MIT_PARAMS = 0xC4
WRITE_MIT_PARAMS = 0xC5
FAULT_NOTICE = 0xBF

# Commands whose reply carries a different code than the request.
REPLY_CODE = {WRITE_MIT_PARAMS: READ_MIT_PARAMS}

# Fixed MIT ranges (v_max and t_max are read from the board, 0xC4).
MIT_P_MAX = 12.56
MIT_KP_MAX = 500.0
MIT_KD_MAX = 5.0

CONTROL_MODES = {
    0: "none", 1: "released", 2: "current brake", 3: "duty", 4: "current",
    5: "damped current", 6: "position", 7: "position (multi-turn)",
    8: "speed (timeout)", 9: "speed", 10: "position with speed limit",
    11: "trajectory", 12: "MIT",
}

FAULTS = {
    0: "none", 1: "over voltage", 2: "under voltage", 3: "gate driver",
    4: "over current", 5: "over temperature, power stage",
    6: "over temperature, motor", 7: "gate driver over voltage",
    8: "gate driver under voltage", 9: "MCU under voltage",
    10: "watchdog reset at boot", 11: "encoder (SPI)",
    12: "encoder amplitude low", 13: "encoder amplitude high",
    14: "flash corruption", 15: "current sensor offset high, phase 1",
    16: "current sensor offset high, phase 2",
    17: "current sensor offset high, phase 3", 18: "unbalanced currents",
    19: "brake", 20: "resolver LOT", 21: "resolver DOS", 22: "resolver LOS",
    23: "flash corruption, app configuration",
    24: "flash corruption, motor configuration", 25: "encoder no magnet",
    26: "position error too large",
}


def fault_name(code: int) -> str:
    return FAULTS.get(code, "unknown (%d)" % code)


def can_id(motor_id: int) -> int:
    return BASE_ID + motor_id


# ------------------------------------------------------------------ helpers --

def _frame(cmd: int, body: bytes = b"") -> bytes:
    """cmd followed by body, zero-padded to 8 bytes."""
    data = bytes([cmd]) + body
    if len(data) > 8:
        raise ValueError("payload longer than 8 bytes")
    return data + bytes(8 - len(data))


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _i16(x: float) -> int:
    return int(_clamp(round(x), -32768, 32767))


def _amps_raw(amps: float) -> int:
    return _i16(amps / CURRENT_LSB)


# ----------------------------------------------------------- status reply --

@dataclass
class Status:
    """Common reply of motion commands and 0x9C. Motor side."""
    temperature_c: int
    current_a: float
    speed_dps: int
    angle_raw: int          # 0..16383

    @property
    def angle_deg(self) -> float:
        return self.angle_raw * 360.0 / ANGLE_14BIT


def decode_status(data: bytes) -> Status:
    temp, cur, spd, ang = struct.unpack_from("<bhhH", data, 1)
    return Status(temp, cur * CURRENT_LSB, spd, ang)


# ------------------------------------------------------------ motion --

def motor_off() -> bytes:
    return _frame(MOTOR_OFF)


def motor_stop() -> bytes:
    return _frame(MOTOR_STOP)


def motor_start() -> bytes:
    return _frame(MOTOR_START)


def current(amps: float, damping: float = 0.0) -> bytes:
    """0xA1. damping 0 = pure current control, otherwise 0.01..100."""
    kd = _i16(_clamp(damping, 0.0, 100.0) * 100.0)
    return _frame(CURRENT, bytes(3) + struct.pack("<hh", _amps_raw(amps), kd))


def speed_dps(dps: float) -> bytes:
    """0xA2 mode 0, Speed (DPS, position-tracked): motor speed in deg/s."""
    raw = int(_clamp(round(dps * 100.0), -2**31, 2**31 - 1))
    return _frame(SPEED, bytes([0, 0, 0]) + struct.pack("<i", raw))


def speed_erpm(erpm: float) -> bytes:
    """0xA2 mode 1, Speed (eRPM, VESC speed loop): electrical RPM."""
    raw = int(_clamp(round(erpm), -2**31, 2**31 - 1))
    return _frame(SPEED, bytes([1, 0, 0]) + struct.pack("<i", raw))


def _centideg(deg: float) -> int:
    return int(_clamp(round(deg * 100.0), -2**31, 2**31 - 1))


def position(deg: float) -> bytes:
    """0xA3, multi-turn motor angle, no speed limit."""
    return _frame(POSITION, bytes(3) + struct.pack("<i", _centideg(deg)))


SPEED_LIMIT_MAX = 25000


def position_limited(deg: float, max_dps: float) -> bytes:
    """0xA4, multi-turn motor angle with a speed limit of 1..25000 deg/s.

    The board ignores an out-of-range limit without telling (it replies but
    does not move), so it is refused here instead."""
    lim = int(round(max_dps))
    if not 1 <= lim <= SPEED_LIMIT_MAX:
        raise ValueError("speed limit must be 1..%d deg/s" % SPEED_LIMIT_MAX)
    return _frame(POSITION_LIMITED,
                  bytes(1) + struct.pack("<hi", lim, _centideg(deg)))


def duty(fraction: float) -> bytes:
    """0xA5, -1.0..1.0."""
    raw = _i16(_clamp(fraction, -1.0, 1.0) * 10000.0)
    return _frame(DUTY, bytes(3) + struct.pack("<h", raw))


def brake(amps: float) -> bytes:
    """0xA6, brake current (magnitude)."""
    return _frame(BRAKE, bytes(3) + struct.pack("<h", _amps_raw(abs(amps))))


# ------------------------------------------------------------ reads --

def request(cmd: int) -> bytes:
    """Any command that takes no arguments (reads, 0x9B, 0x19, 0xC1, 0xC2)."""
    return _frame(cmd)


def decode_multi_turn(data: bytes) -> float:
    """0x92 -> motor multi-turn angle, degrees."""
    raw = int.from_bytes(data[1:8], "little", signed=True)
    return raw / 100.0


@dataclass
class OutputEncoder:
    angle_deg: float        # single turn, 0..360
    multi_turn_deg: float
    valid: bool
    calibrated: bool
    dual_calibrated: bool


def decode_output_encoder(data: bytes) -> OutputEncoder:
    st, mt, flags = struct.unpack_from("<HiB", data, 1)
    return OutputEncoder(st * 360.0 / 65536.0, mt / 100.0,
                         bool(flags & 1), bool(flags & 2), bool(flags & 4))


@dataclass
class Encoder:
    error_bits: int
    angle_raw: int
    raw_angle_raw: int
    offset_raw: int


def decode_encoder(data: bytes) -> Encoder:
    return Encoder(data[1], *struct.unpack_from("<HHH", data, 2))


@dataclass
class EncoderStatus:
    air_gap_um: int
    signal_level: int
    error_rate_pct: float
    crc_status: int
    error: bool
    warning: bool


def decode_encoder_status(data: bytes) -> EncoderStatus:
    gap, sig, err, crc, flags = struct.unpack_from("<HHBBB", data, 1)
    return EncoderStatus(gap, sig, err * 0.5, crc, bool(flags & 1), bool(flags & 2))


@dataclass
class Phases:
    control_mode: int
    ia: float
    ib: float
    ic: float

    @property
    def control_mode_name(self) -> str:
        return CONTROL_MODES.get(self.control_mode, "unknown")


def decode_phases(data: bytes) -> Phases:
    a, b, c = struct.unpack_from("<hhh", data, 2)
    return Phases(data[1], a / 64.0, b / 64.0, c / 64.0)


def decode_faults(data: bytes) -> list:
    """0xB0 -> the last seven fault codes."""
    return list(data[1:8])


# ------------------------------------------------------------ settings --

@dataclass
class Pid:
    kp: float
    ki: float
    kd: float


def write_pid(pid: Pid, flash: bool = False) -> bytes:
    raw = (round(pid.kp * 1000), round(pid.ki * 100000), round(pid.kd * 100000))
    if not all(0 <= r <= 0xFFFF for r in raw):
        raise ValueError("PID gain out of range")
    return _frame(WRITE_PID_FLASH if flash else WRITE_PID_RAM,
                  bytes(1) + struct.pack("<HHH", *raw))


def decode_pid(data: bytes) -> Pid:
    p, i, d = struct.unpack_from("<HHH", data, 2)
    return Pid(p / 1000.0, i / 100000.0, d / 100000.0)


@dataclass
class Limits:
    motor_current_a: float
    abs_current_a: float
    battery_current_a: float
    gate_oc_mode: int


def write_limits(lim: Limits) -> bytes:
    raw = (round(lim.motor_current_a * 100), round(lim.abs_current_a * 100),
           round(lim.battery_current_a * 100))
    if not all(0 <= r <= 0xFFFF for r in raw):
        raise ValueError("current limit out of range")
    return _frame(WRITE_LIMITS_FLASH, bytes([lim.gate_oc_mode & 0xFF]) +
                  struct.pack("<HHH", *raw))


def decode_limits(data: bytes) -> Limits:
    m, a, b = struct.unpack_from("<HHH", data, 2)
    return Limits(m / 100.0, a / 100.0, b / 100.0, data[1])


def write_encoder_offset(offset_raw: int) -> bytes:
    return _frame(WRITE_ENCODER_OFFSET, bytes(5) + struct.pack("<H", offset_raw & 0xFFFF))


def decode_offset(data: bytes) -> int:
    """0x91 / 0x19 reply -> stored offset, 14-bit."""
    return struct.unpack_from("<H", data, 6)[0]


# ------------------------------------------------------------ MIT --

def _f2u(x: float, lo: float, hi: float, bits: int) -> int:
    span = (1 << bits) - 1
    return int(round((_clamp(x, lo, hi) - lo) / (hi - lo) * span))


def _u2f(u: int, lo: float, hi: float, bits: int) -> float:
    return u * (hi - lo) / ((1 << bits) - 1) + lo


@dataclass
class MitParams:
    v_max: float            # rad/s, output side
    t_max: float            # Nm, output side
    kt: float               # reply: output-side Kt, Nm/A
    gear_ratio: float
    kd_scale: int           # 0 in the reply = firmware without scale, use 1

    @property
    def effective_kd_scale(self) -> int:
        return self.kd_scale or 1


def decode_mit_params(data: bytes) -> MitParams:
    v, t, kt, gr, scale = struct.unpack_from("<BBHHB", data, 1)
    return MitParams(float(v), float(t), kt / 1000.0, gr / 100.0, scale)


def write_mit_params(v_max: float, t_max: float, motor_kt: float,
                     gear_ratio: float, kd_scale: int = 0) -> bytes:
    """0xC5. motor_kt is the motor-side Kt (0 = from the motor setup);
    kd_scale 0 keeps the stored scale."""
    if not 0 <= kd_scale <= 8:
        raise ValueError("kd_scale must be 0 (keep) or 1..8")
    return _frame(WRITE_MIT_PARAMS, struct.pack(
        "<BBHHB", int(_clamp(round(v_max), 0, 255)), int(_clamp(round(t_max), 0, 255)),
        int(_clamp(round(motor_kt * 1000), 0, 0xFFFF)),
        int(_clamp(round(gear_ratio * 100), 0, 0xFFFF)), kd_scale))


def mit_control(p_rad: float, v_rads: float, kp: float, kd: float, t_nm: float,
                params: MitParams) -> bytes:
    """0xC0. Output-side targets; v and t ranges come from the board (0xC4)."""
    kd_max = MIT_KD_MAX * params.effective_kd_scale
    p = _f2u(p_rad, -MIT_P_MAX, MIT_P_MAX, 16)
    v = _f2u(v_rads, -params.v_max, params.v_max, 12)
    k = _f2u(kp, 0.0, MIT_KP_MAX, 12)
    d = _f2u(kd, 0.0, kd_max, 8)
    t = _f2u(t_nm, -params.t_max, params.t_max, 8)
    return bytes([MIT_CONTROL, p >> 8, p & 0xFF, v >> 4,
                  ((v & 0x0F) << 4) | (k >> 8), k & 0xFF, d, t])


def _u2f_sym(u: int, hi: float, bits: int) -> float:
    """Signed field as SPN-MC V2 firmware 261109+ decodes it: the middle code
    (what 0 encodes to) is exactly 0, every other code as before."""
    if u == 1 << (bits - 1):
        return 0.0
    return _u2f(u, -hi, hi, bits)


def unpack_mit_control(data: bytes, params: MitParams):
    """Inverse of mit_control, as the board decodes it (used by the simulator
    and the tests). SPN-MC V2 firmware 261109+: a signed field of 0 decodes to
    exactly 0 (older firmware: +max/(2^bits - 1))."""
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    k = ((data[4] & 0x0F) << 8) | data[5]
    kd_max = MIT_KD_MAX * params.effective_kd_scale
    return (_u2f_sym(p, MIT_P_MAX, 16),
            _u2f_sym(v, params.v_max, 12),
            _u2f(k, 0.0, MIT_KP_MAX, 12),
            _u2f(data[6], 0.0, kd_max, 8),
            _u2f_sym(data[7], params.t_max, 8))


@dataclass
class MitState:
    """0xC0 reply. Output side, position relative to the MIT zero."""
    temperature_c: int
    current_a: float
    speed_dps: int
    position_rad: float


def decode_mit_state(data: bytes) -> MitState:
    temp, cur, spd, pos = struct.unpack_from("<bhhh", data, 1)
    return MitState(temp, cur * CURRENT_LSB, spd, pos * MIT_P_MAX / 32767.0)


def mit_set_zero(offset_deg: float = 0.0) -> bytes:
    """0xC3. The present output position becomes offset_deg."""
    return _frame(MIT_SET_ZERO, bytes(5) + struct.pack("<h", _i16(offset_deg * 100.0)))
