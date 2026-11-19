"""End to end through the bus layer against the simulator."""
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from openrobot_motor import Motor, MotorBus, ReplyTimeout  # noqa: E402


@pytest.fixture
def bus():
    b = MotorBus.open("sim", sim_ids=(1, 2))
    yield b
    b.close()


def test_scan(bus):
    assert bus.scan(range(1, 6)) == [1, 2]


def test_missing_motor_times_out(bus):
    with pytest.raises(ReplyTimeout):
        Motor(bus, 7).status()


def test_speed_reaches_target(bus):
    with Motor(bus, 1) as m:
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            st = m.speed_dps(360)
            time.sleep(0.01)
        assert st.speed_dps == pytest.approx(360, abs=30)
    assert Motor(bus, 1).phases().control_mode == 1      # released on exit


def test_position_limited(bus):
    with Motor(bus, 2) as m:
        start = m.multi_turn_deg()
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            m.position(start + 90, 720)
            if abs(m.multi_turn_deg() - (start + 90)) < 1.0:
                break
            time.sleep(0.01)
        assert m.multi_turn_deg() == pytest.approx(start + 90, abs=1.0)


def test_mit_holds_zero(bus):
    with Motor(bus, 1) as m:
        m.mit_enter()
        m.mit_zero()
        for _ in range(50):
            st = m.mit(0.3, 0, 20, 1)
            time.sleep(0.01)
        assert st.position_rad == pytest.approx(0.3, abs=0.05)
        m.mit_exit()


def test_fault_notice(bus):
    seen = []
    bus.on_fault = lambda mid, code: seen.append((mid, code))
    bus.sim.raise_fault(2, 4)
    time.sleep(0.05)
    bus.poll_faults()
    assert seen == [(2, 4)]
    m = Motor(bus, 2)
    assert m.fault_history()[-1] == 4
    assert m.clear_fault() == 0


def test_erpm_releases_first_after_another_mode(bus):
    # The board ignores eRPM while another motion mode runs, so speed_erpm()
    # must send Motor off (0x80) first unless the last motion was a release
    # or eRPM.
    m = Motor(bus, 1)
    sent = []
    real = bus.request

    def spy(motor_id, data, timeout=0.02):
        sent.append(data[0] if data[0] != 0xA2 else ("A2", data[1]))
        return real(motor_id, data, timeout)

    bus.request = spy
    try:
        m.speed_dps(100)
        m.speed_erpm(1000)
        m.speed_erpm(1200)
        m.off()
        m.speed_erpm(800)
    finally:
        bus.request = real
        m.off()
    assert sent == [("A2", 0), 0x80, ("A2", 1), ("A2", 1), 0x80, ("A2", 1)]
