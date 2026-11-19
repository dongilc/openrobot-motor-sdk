"""One motor on a MotorBus, with plain-unit methods.

    with MotorBus.open("slcan", "COM5") as bus, Motor(bus, 1) as m:
        m.speed_dps(360)        # deg/s, motor side
        ...
    # leaving the block releases the motor, also on Ctrl+C or an error
"""
from . import protocol as P
from .bus import MotorBus, ReplyTimeout


class Motor:
    def __init__(self, bus: MotorBus, motor_id: int, timeout: float = 0.02):
        self.bus = bus
        self.id = motor_id
        self.timeout = timeout
        self._mit = None            # MitParams, read on mit_enter()
        # Speed (eRPM) is ignored while another motion mode is active on the
        # board, so speed_erpm() releases first unless the last motion
        # command was a release or eRPM itself.
        self._erpm_ready = False

    _MOTION = {P.MOTOR_STOP, P.MOTOR_START, P.CURRENT, P.SPEED, P.POSITION, P.POSITION_LIMITED,
               P.DUTY, P.BRAKE, P.MIT_ENTER, P.MIT_CONTROL}

    def _req(self, data: bytes) -> bytes:
        reply = self.bus.request(self.id, data, self.timeout)
        if data[0] in (P.MOTOR_OFF, P.MIT_EXIT) or (data[0] == P.SPEED and data[1] == 1):
            self._erpm_ready = True
        elif data[0] in self._MOTION:
            self._erpm_ready = False
        return reply

    # Leaving a with-block always releases the motor, whatever happened.
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        try:
            self.off()
        except Exception:
            pass

    # ------------------------------------------------------------ motion --

    def off(self) -> P.Status:
        """Release: no torque, free shaft."""
        return P.decode_status(self._req(P.motor_off()))

    def stop(self) -> P.Status:
        """Hold at zero speed."""
        return P.decode_status(self._req(P.motor_stop()))

    def current(self, amps: float, damping: float = 0.0) -> P.Status:
        return P.decode_status(self._req(P.current(amps, damping)))

    def speed_dps(self, dps: float) -> P.Status:
        """Speed (DPS, position-tracked), 0xA2 mode 0: a target angle advances
        at dps (motor side) and the position loop follows it."""
        return P.decode_status(self._req(P.speed_dps(dps)))

    def speed_erpm(self, erpm: float) -> P.Status:
        """Speed (eRPM, VESC speed loop), 0xA2 mode 1: electrical RPM.
        Sends Motor off first when coming from another motion mode."""
        if not self._erpm_ready:
            self.off()
        return P.decode_status(self._req(P.speed_erpm(erpm)))

    def speed(self, dps: float) -> P.Status:
        """Same as speed_dps(), kept for existing code."""
        return self.speed_dps(dps)

    def position(self, deg: float, max_dps: float) -> P.Status:
        """Multi-turn motor angle with a speed limit (0xA4)."""
        return P.decode_status(self._req(P.position_limited(deg, max_dps)))

    def position_direct(self, deg: float) -> P.Status:
        """Multi-turn motor angle, no speed limit (0xA3). A big step runs at
        full current; prefer position()."""
        return P.decode_status(self._req(P.position(deg)))

    def duty(self, fraction: float) -> P.Status:
        return P.decode_status(self._req(P.duty(fraction)))

    def brake(self, amps: float) -> P.Status:
        return P.decode_status(self._req(P.brake(amps)))

    # ------------------------------------------------------------ reads --

    def status(self) -> P.Status:
        return P.decode_status(self._req(P.request(P.READ_STATUS)))

    def multi_turn_deg(self) -> float:
        return P.decode_multi_turn(self._req(P.request(P.READ_MULTI_TURN)))

    def phases(self) -> P.Phases:
        return P.decode_phases(self._req(P.request(P.READ_PHASES)))

    def output_encoder(self):
        """Output-shaft encoder, or None if this build has none."""
        try:
            return P.decode_output_encoder(self._req(P.request(P.READ_OUTPUT_ENCODER)))
        except ReplyTimeout:
            return None

    def encoder(self) -> P.Encoder:
        return P.decode_encoder(self._req(P.request(P.READ_ENCODER)))

    def fault_history(self) -> list:
        return P.decode_faults(self._req(P.request(P.READ_FAULTS)))

    def clear_fault(self) -> int:
        """Returns the fault code left after clearing (0 = cleared)."""
        return self._req(P.request(P.CLEAR_FAULT))[1]

    def pid(self) -> P.Pid:
        return P.decode_pid(self._req(P.request(P.READ_PID)))

    def set_pid(self, pid: P.Pid, flash: bool = False) -> P.Pid:
        return P.decode_pid(self._req(P.write_pid(pid, flash)))

    def limits(self) -> P.Limits:
        return P.decode_limits(self._req(P.request(P.READ_LIMITS)))

    def set_zero_here(self) -> int:
        """Store the present angle as zero (writes flash; motor must be
        stopped). Returns the stored offset."""
        return P.decode_offset(self._req(P.request(P.SET_ZERO_HERE)))

    # ------------------------------------------------------------ MIT --

    def mit_params(self) -> P.MitParams:
        return P.decode_mit_params(self._req(P.request(P.READ_MIT_PARAMS)))

    def mit_enter(self) -> P.MitParams:
        """Read the board's MIT ranges, then enter MIT mode."""
        self._mit = self.mit_params()
        self._req(P.request(P.MIT_ENTER))
        return self._mit

    def mit_zero(self, offset_deg: float = 0.0):
        """The present output position becomes offset_deg (not stored)."""
        self._req(P.mit_set_zero(offset_deg))

    def mit(self, p_rad: float, v_rads: float, kp: float, kd: float,
            t_nm: float = 0.0) -> P.MitState:
        if self._mit is None:
            raise RuntimeError("call mit_enter() first")
        return P.decode_mit_state(self._req(
            P.mit_control(p_rad, v_rads, kp, kd, t_nm, self._mit)))

    def mit_exit(self):
        self._req(P.request(P.MIT_EXIT))
        self._mit = None
