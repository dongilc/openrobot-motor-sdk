"""Simulated controllers on python-can's in-process "virtual" bus.

Lets every example and test run without hardware. The motor model is a plain
inertia with viscous friction and simple control loops: good enough to see
the protocol work, not a model of a particular actuator.

The simulator does not model the command watchdog (protocol section 8): it
keeps the last command running when the host goes quiet, like firmware
without the watchdog.
"""
import math
import struct
import threading
import time

import can

from . import protocol as P


class SimMotor:
    def __init__(self, motor_id: int, gear_ratio: float = 9.0, kt: float = 0.08,
                 pole_pairs: int = 14):
        self.id = motor_id
        self.gear_ratio = gear_ratio
        self.kt = kt                    # Nm/A, motor side
        self.pole_pairs = pole_pairs
        self.inertia = 2e-4             # kg m^2, motor side incl. reflected load
        self.friction = 2e-3            # Nm per rad/s
        self._integ = 0.0               # speed loop integrator, A
        self.i_max = 20.0
        self.limits = P.Limits(20.0, 40.0, 15.0, 3)
        self.pid = P.Pid(0.03, 0.0, 0.0004)
        self.mit = P.MitParams(45.0, 33.0, kt * gear_ratio, gear_ratio, 1)
        self.angle = 0.0                # deg, motor multi-turn
        self.omega = 0.0                # rad/s, motor
        self.i = 0.0
        self.temp = 30
        self.mode = 1                   # released
        self.target = 0.0
        self.aux = 0.0
        self.mit_cmd = (0.0, 0.0, 0.0, 0.0, 0.0)
        self.mit_zero = 0.0             # deg, motor side
        self.fault = 0
        self.history = [0] * 7
        self.offset = 0

    # ------------------------------------------------------------ physics --

    def _speed_loop(self, target_dps, dt):
        err = math.radians(target_dps) - self.omega
        self._integ = max(-self.i_max, min(self.i_max, self._integ + 0.5 * err * dt))
        return 0.05 * err + self._integ

    def step(self, dt: float):
        dps = math.degrees(self.omega)
        m = self.mode
        if m not in (3, 9, 10):
            self._integ = 0.0
        if self.fault or m in (0, 1):
            i = 0.0
        elif m == 4:
            i = self.target
        elif m == 5:
            i = self.target - self.aux * 0.01 * self.omega
        elif m == 2:
            i = -math.copysign(self.target, self.omega) if abs(dps) > 5 else 0.0
        elif m == 3:
            i = self._speed_loop(self.target * 30000.0, dt)
        elif m == 9:
            i = self._speed_loop(self.target, dt)
        elif m == 7:
            err = self.target - self.angle
            i = 0.02 * err - 0.002 * dps
        elif m == 10:
            err = self.target - self.angle
            v = max(-self.aux, min(self.aux, 8.0 * err))
            i = self._speed_loop(v, dt)
        elif m == 12:
            p_des, v_des, kp, kd, t_ff = self.mit_cmd
            gr = self.gear_ratio
            p = math.radians(self.angle - self.mit_zero) / gr
            v = self.omega / gr
            tau = kp * (p_des - p) + kd * (v_des - v) + t_ff
            i = tau / (self.kt * gr)
        else:
            i = 0.0
        self.i = max(-self.i_max, min(self.i_max, i))
        acc = (self.kt * self.i - self.friction * self.omega) / self.inertia
        self.omega += acc * dt
        self.angle += math.degrees(self.omega) * dt

    def status(self, cmd):
        return struct.pack("<BbhhH", cmd, self.temp,
                           int(round(self.i / P.CURRENT_LSB)),
                           int(max(-32768, min(32767, round(math.degrees(self.omega))))),
                           int(round(self.angle / 360.0 * P.ANGLE_14BIT)) % P.ANGLE_14BIT)

    def raise_fault(self, code: int):
        self.fault = code
        self.history = self.history[1:] + [code]
        self.mode = 1

    # ------------------------------------------------------------ protocol --

    def handle(self, d: bytes):
        """Return the reply payload for request d, or None for no reply."""
        c = d[0]
        if c == P.MOTOR_OFF or c == P.MIT_EXIT:
            self.mode = 1
        elif c == P.MOTOR_STOP:
            self.mode, self.target = 9, 0.0
        elif c == P.MOTOR_START:
            self.mode, self.target = 3, 0.0
        elif c == P.CURRENT:
            amps, kd = struct.unpack_from("<hh", d, 4)
            self.mode = 4 if kd == 0 else 5
            self.target, self.aux = amps * P.CURRENT_LSB, min(max(kd / 100.0, 0), 100)
        elif c == P.SPEED:
            v = struct.unpack_from("<i", d, 4)[0]
            if d[1] == 1:
                self.mode, self.target = 9, v * 6.0 / self.pole_pairs
            else:
                self.mode, self.target = 9, v / 100.0
        elif c == P.POSITION:
            self.mode, self.target = 7, struct.unpack_from("<i", d, 4)[0] / 100.0
        elif c == P.POSITION_LIMITED:
            lim, pos = struct.unpack_from("<hi", d, 2)
            if 0 < lim <= P.SPEED_LIMIT_MAX:
                self.mode, self.target, self.aux = 10, pos / 100.0, float(lim)
        elif c == P.DUTY:
            self.mode, self.target = 3, struct.unpack_from("<h", d, 4)[0] / 10000.0
        elif c == P.BRAKE:
            self.mode, self.target = 2, abs(struct.unpack_from("<h", d, 4)[0]) * P.CURRENT_LSB
        elif c == P.MIT_CONTROL:
            self.mit_cmd = P.unpack_mit_control(d, self.mit)
            self.mode = 12
            gr = self.gear_ratio
            pos = math.radians(self.angle - self.mit_zero) / gr
            return struct.pack("<Bbhhh", c, self.temp, int(round(self.i / P.CURRENT_LSB)),
                               int(round(math.degrees(self.omega) / gr)),
                               int(max(-32767, min(32767, round(pos / P.MIT_P_MAX * 32767)))))
        elif c == P.MIT_ENTER:
            return bytes([c, self.mit.kd_scale]) + bytes(6)
        elif c == P.MIT_SET_ZERO:
            off = struct.unpack_from("<h", d, 6)[0] / 100.0
            self.mit_zero = self.angle - off * self.gear_ratio
            return bytes([c]) + bytes(5) + d[6:8]
        elif c in (P.READ_MIT_PARAMS, P.WRITE_MIT_PARAMS):
            if c == P.WRITE_MIT_PARAMS and self.mode in (1,):
                v, t, kt, gr, sc = struct.unpack_from("<BBHHB", d, 1)
                self.gear_ratio = gr / 100.0 or self.gear_ratio
                self.mit = P.MitParams(float(v), float(t),
                                       (kt / 1000.0 or self.kt) * self.gear_ratio,
                                       self.gear_ratio, sc or self.mit.kd_scale)
            m = self.mit
            return struct.pack("<BBBHHB", P.READ_MIT_PARAMS, int(m.v_max), int(m.t_max),
                               int(round(m.kt * 1000)), int(round(m.gear_ratio * 100)),
                               m.kd_scale)
        elif c == P.READ_STATUS:
            pass
        elif c == P.READ_PHASES:
            return struct.pack("<BBhhh", c, self.mode, int(self.i * 64), 0, 0)
        elif c == P.READ_MULTI_TURN:
            return bytes([c]) + int(round(self.angle * 100)).to_bytes(7, "little", signed=True)
        elif c == P.READ_OUTPUT_ENCODER:
            return None                 # this simulated build has no output encoder
        elif c == P.READ_ENCODER:
            raw = int(round(self.angle / 360.0 * P.ANGLE_14BIT)) % P.ANGLE_14BIT
            return struct.pack("<BBHHH", c, 0, raw, (raw + self.offset) % P.ANGLE_14BIT,
                               self.offset)
        elif c == P.CLEAR_FAULT:
            self.fault = 0
            return bytes([c, 0]) + bytes(6)
        elif c == P.READ_FAULTS:
            return bytes([c] + self.history)
        elif c in (P.READ_PID, P.WRITE_PID_RAM, P.WRITE_PID_FLASH):
            if c != P.READ_PID:
                self.pid = P.decode_pid(d)
            return bytes([c, 0]) + struct.pack("<HHH", round(self.pid.kp * 1000),
                                               round(self.pid.ki * 1e5), round(self.pid.kd * 1e5))
        elif c in (P.READ_LIMITS, P.WRITE_LIMITS_FLASH):
            if c != P.READ_LIMITS:
                self.limits = P.decode_limits(d)
                self.i_max = self.limits.motor_current_a
            lm = self.limits
            return struct.pack("<BBHHH", c, lm.gate_oc_mode, round(lm.motor_current_a * 100),
                               round(lm.abs_current_a * 100), round(lm.battery_current_a * 100))
        elif c in (P.SET_ZERO_HERE, P.WRITE_ENCODER_OFFSET):
            self.angle = 0.0
            return bytes([c]) + bytes(5) + struct.pack("<H", self.offset)
        else:
            return None
        return self.status(c)


class SimBoards(threading.Thread):
    """Runs simulated motors on one virtual CAN channel."""

    def __init__(self, channel: str, ids=(1,), **motor_kw):
        super().__init__(daemon=True)
        self.bus = can.Bus(interface="virtual", channel=channel,
                           receive_own_messages=False)
        self.motors = {i: SimMotor(i, **motor_kw) for i in ids}
        self._run = True
        self.start()

    def stop(self):
        self._run = False
        self.join(1.0)
        self.bus.shutdown()

    def raise_fault(self, motor_id: int, code: int):
        """Make a motor fault, as the board does: stop and send 0xBF."""
        self.motors[motor_id].raise_fault(code)
        self.bus.send(can.Message(arbitration_id=P.can_id(motor_id), is_extended_id=False,
                                  data=bytes([P.FAULT_NOTICE, code]) + bytes(6)))

    def run(self):
        last = time.monotonic()
        while self._run:
            msg = self.bus.recv(0.001)
            now = time.monotonic()
            dt, last = now - last, now
            n = max(1, int(dt / 0.0005))
            for m in self.motors.values():
                for _ in range(n):
                    m.step(dt / n)
            if msg is None or msg.is_extended_id or msg.dlc != 8:
                continue
            m = self.motors.get(msg.arbitration_id - P.BASE_ID)
            if m is None:
                continue
            reply = m.handle(bytes(msg.data))
            if reply is not None:
                self.bus.send(can.Message(arbitration_id=msg.arbitration_id,
                                          is_extended_id=False, data=reply))
