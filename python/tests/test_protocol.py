"""Frame layouts checked byte by byte against the firmware handler."""
import os
import struct
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from openrobot_motor import protocol as P  # noqa: E402


def test_frames_are_8_bytes():
    params = P.MitParams(45, 33, 0.7, 9, 1)
    for f in (P.motor_off(), P.current(1.0), P.speed_dps(10), P.position(1),
              P.position_limited(1, 100), P.duty(0.1), P.brake(1),
              P.mit_control(0, 0, 0, 0, 0, params), P.mit_set_zero(1),
              P.write_pid(P.Pid(0.1, 0, 0)), P.write_mit_params(45, 33, 0, 9)):
        assert len(f) == 8


def test_current_layout():
    f = P.current(33.0, damping=1.5)
    assert f[0] == 0xA1
    assert struct.unpack_from("<hh", f, 4) == (2048, 150)


def test_speed_layout():
    f = P.speed_dps(-123.45)
    assert f[1] == 0 and struct.unpack_from("<i", f, 4)[0] == -12345
    assert P.speed_erpm(5000)[1] == 1


def test_position_limited_layout_and_guard():
    f = P.position_limited(-90.5, 720)
    assert struct.unpack_from("<hi", f, 2) == (720, -9050)
    with pytest.raises(ValueError):
        P.position_limited(0, 0)
    with pytest.raises(ValueError):
        P.position_limited(0, 25001)


def test_status_decode():
    d = struct.pack("<BbhhH", 0x9C, -5, -2048, 300, 8192)
    st = P.decode_status(d)
    assert st.temperature_c == -5
    assert st.current_a == pytest.approx(-33.0)
    assert st.speed_dps == 300
    assert st.angle_deg == pytest.approx(180.0)


def test_multi_turn_negative():
    d = bytes([0x92]) + (-123456).to_bytes(7, "little", signed=True)
    assert P.decode_multi_turn(d) == pytest.approx(-1234.56)


def test_mit_roundtrip_matches_firmware_decode():
    params = P.MitParams(45.0, 33.0, 0.72, 9.0, 1)
    want = (1.234, -3.0, 50.0, 1.2, -4.0)
    got = P.unpack_mit_control(P.mit_control(*want, params), params)
    lsb = (25.12 / 65535, 90 / 4095, 500 / 4095, 5 / 255, 66 / 255)
    for g, w, q in zip(got, want, lsb):
        assert abs(g - w) <= q / 2 + 1e-9


def test_mit_zero_decodes_exactly_zero():
    params = P.MitParams(45.0, 33.0, 0.72, 9.0, 1)
    p, v, kp, kd, t = P.unpack_mit_control(P.mit_control(0, 0, 10, 1, 0, params), params)
    assert p == 0.0 and v == 0.0 and t == 0.0
    # one code either side is unchanged
    t1 = P.unpack_mit_control(P.mit_control(0, 0, 10, 1, 0.4, params), params)[4]   # raw 129
    assert abs(t1 - (129 * 66 / 255 - 33.0)) < 1e-9


def test_mit_zero_command_is_mid_scale():
    params = P.MitParams(45.0, 33.0, 0.72, 9.0, 1)
    f = P.mit_control(0, 0, 0, 0, 0, params)
    assert (f[1] << 8 | f[2]) == 32768                    # p = 0
    assert ((f[3] << 4) | (f[4] >> 4)) == 2048            # v = 0
    assert f[6] == 0 and f[7] == 128


def test_mit_kd_scale_widens_range():
    p1 = P.MitParams(45, 33, 0.7, 9, 1)
    p4 = P.MitParams(45, 33, 0.7, 9, 4)
    assert P.mit_control(0, 0, 0, 5.0, 0, p1)[6] == 255
    assert P.mit_control(0, 0, 0, 5.0, 0, p4)[6] == round(255 / 4)


def test_mit_params_decode():
    d = struct.pack("<BBBHHB", 0xC4, 45, 33, 720, 900, 0)
    m = P.decode_mit_params(d)
    assert (m.v_max, m.t_max, m.kt, m.gear_ratio) == (45, 33, 0.72, 9.0)
    assert m.effective_kd_scale == 1


def test_mit_state_decode():
    d = struct.pack("<Bbhhh", 0xC0, 40, 100, -50, 32767)
    s = P.decode_mit_state(d)
    assert s.position_rad == pytest.approx(12.56)
    assert s.speed_dps == -50
