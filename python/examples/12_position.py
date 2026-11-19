"""Step 1c: position control with a speed limit (0xA4).

Angles in this command are on the motor side. For an actuator with a gearbox,
pass --gear to give the move in output-shaft degrees instead.
"""
import time

from common import parser, open_bus, confirm_motion

from openrobot_motor import Motor

p = parser("Move by an angle and back")
p.add_argument("--deg", type=float, default=90.0, help="move, deg (default 90)")
p.add_argument("--gear", type=float, default=1.0,
               help="gear ratio; --deg is then on the output shaft")
p.add_argument("--max-dps", type=float, default=720.0, help="motor speed limit, deg/s")
args = p.parse_args()
confirm_motion(args, "move %.1f deg and back" % args.deg)


def move_to(m, target, tol=0.5, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        m.position(target, args.max_dps)
        now = m.multi_turn_deg()
        print("\rtarget %9.2f  now %9.2f deg (motor)" % (target, now), end="")
        if abs(now - target) < tol:
            break
        time.sleep(0.01)
    print()


with open_bus(args) as bus, Motor(bus, args.id) as m:
    start = m.multi_turn_deg()
    move_to(m, start + args.deg * args.gear)
    time.sleep(0.5)
    move_to(m, start)
    time.sleep(0.5)
