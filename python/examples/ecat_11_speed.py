"""EtherCAT step 1: Speed (DPS, position-tracked), forward, backward, hold, release.

The slave ramps target_velocity by at most 10 deg/s per PDO cycle.
"""
import time

from ecat_common import parser, open_bus, confirm_motion

p = parser("Run forward and backward at a set speed over EtherCAT")
p.add_argument("--dps", type=float, default=360.0, help="motor speed, deg/s (default 360)")
args = p.parse_args()
confirm_motion("turn the motor at %.0f deg/s both ways" % args.dps)


def run(m, dps, seconds):
    m.speed_dps(dps)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        st = m.status()
        print("\rtarget %6.0f  actual %7.1f deg/s  current %5.2f A  %s"
              % (dps, st.velocity_dps, st.current_a, st.mode_name), end="")
        if st.fault_code:
            raise SystemExit("\nfault: %s" % st.fault_name)
        time.sleep(0.05)
    print()


with open_bus(args) as bus:             # leaving the block releases every motor
    m = bus.motors[args.slave]
    run(m, args.dps, 2.0)
    run(m, -args.dps, 2.0)
    m.stop()                            # hold at zero speed
    time.sleep(0.5)
    print("cycles %d, missed %d, longest gap %.1f ms" % (bus.cycles, bus.wkc_errors, bus.max_cycle_s * 1e3))
