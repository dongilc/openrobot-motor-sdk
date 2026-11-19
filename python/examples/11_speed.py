"""Step 1b: speed control. Forward, backward, hold, release.

Two speed controllers share command 0xA2 (see docs/can_protocol.md):

    python 11_speed.py --dps 360       # Speed (DPS, position-tracked), deg/s
    python 11_speed.py --erpm 3000     # Speed (eRPM, VESC speed loop)
"""
import time

from common import parser, open_bus, confirm_motion

from openrobot_motor import Motor

p = parser("Run forward and backward at a set speed")
g = p.add_mutually_exclusive_group()
g.add_argument("--dps", type=float, default=360.0,
               help="Speed (DPS, position-tracked): motor speed, deg/s (default 360)")
g.add_argument("--erpm", type=float,
               help="Speed (eRPM, VESC speed loop): electrical RPM = RPM x pole pairs")
args = p.parse_args()
if args.erpm is not None:
    target, unit, label = args.erpm, "eRPM", "Speed (eRPM)"
else:
    target, unit, label = args.dps, "deg/s", "Speed (DPS)"
confirm_motion(args, "turn the motor at %.0f %s both ways" % (target, unit))


def run(m, value, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        st = m.speed_erpm(value) if args.erpm is not None else m.speed_dps(value)
        print("\r%s target %6.0f %s  actual %6d deg/s  current %5.2f A"
              % (label, value, unit, st.speed_dps, st.current_a), end="")
        time.sleep(0.01)
    print()


with open_bus(args) as bus, Motor(bus, args.id) as m:
    run(m, target, 2.0)
    run(m, -target, 2.0)
    if args.erpm is not None:
        m.speed_erpm(0)                 # eRPM: speed 0, no position hold
    else:
        m.stop()                        # DPS: hold where it is
    time.sleep(0.5)
