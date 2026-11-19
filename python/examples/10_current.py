"""Step 1a: current (torque) control. Applies a small current, then releases."""
import time

from common import parser, open_bus, confirm_motion

from openrobot_motor import Motor

p = parser("Apply a constant current for a few seconds")
p.add_argument("--amps", type=float, default=0.5, help="current, A (default 0.5)")
p.add_argument("--seconds", type=float, default=2.0)
args = p.parse_args()
confirm_motion(args, "apply %.2f A for %.1f s" % (args.amps, args.seconds))

with open_bus(args) as bus, Motor(bus, args.id) as m:
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        st = m.current(args.amps)        # resend periodically, every 10 ms
        print("\rcurrent %6.2f A  speed %6d deg/s" % (st.current_a, st.speed_dps), end="")
        time.sleep(0.01)
    print()
# leaving the with-block released the motor
