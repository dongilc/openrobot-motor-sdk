"""Step 3: several motors on one bus, each following its own trajectory.

One loop sends a command to every motor in turn and reads each reply, so all
motors are updated at the same rate. If any motor stops answering, every
motor is released.
"""
import math
import time

from common import parser, open_bus, confirm_motion

from openrobot_motor import Motor

p = parser("Drive several motors with phase-shifted speed waves", ids=True)
p.add_argument("--dps", type=float, default=360.0, help="peak motor speed, deg/s")
p.add_argument("--seconds", type=float, default=6.0)
args = p.parse_args()
confirm_motion(args, "turn motors %s" % args.ids)

with open_bus(args) as bus:
    motors = [Motor(bus, i) for i in args.ids]
    try:
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < args.seconds:
            line = []
            for k, m in enumerate(motors):
                phase = 2 * math.pi * k / len(motors)
                st = m.speed_dps(args.dps * math.sin(2 * math.pi * 0.5 * t + phase))
                line.append("%d:%6d" % (m.id, st.speed_dps))
            print("\r" + "  ".join(line) + " deg/s", end="")
            time.sleep(0.01)
        print()
    finally:
        for m in motors:                # release all, even after an error
            try:
                m.off()
            except Exception:
                pass
