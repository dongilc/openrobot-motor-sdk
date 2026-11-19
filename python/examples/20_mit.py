"""Step 2: MIT impedance control on the output shaft.

    torque = kp * (p_des - p) + kd * (v_des - v) + t_ff

Takes the present position as zero, follows a slow sine, then exits.
Start with low kp; raise it once the joint behaves.
"""
import math
import time

from common import parser, open_bus, confirm_motion

from openrobot_motor import Motor

p = parser("Follow a sine in MIT mode")
p.add_argument("--amp", type=float, default=0.5, help="amplitude, rad output (default 0.5)")
p.add_argument("--hz", type=float, default=0.5, help="frequency (default 0.5 Hz)")
p.add_argument("--kp", type=float, default=10.0, help="Nm/rad (default 10)")
p.add_argument("--kd", type=float, default=0.5, help="Nm s/rad (default 0.5)")
p.add_argument("--seconds", type=float, default=6.0)
p.add_argument("--rate", type=float, default=200.0, help="command rate, Hz")
args = p.parse_args()
confirm_motion(args, "follow a +/-%.2f rad sine on the output shaft" % args.amp)

with open_bus(args) as bus, Motor(bus, args.id) as m:
    par = m.mit_enter()
    print("v_max %.0f rad/s, t_max %.0f Nm, gear %.2f, Kd scale %d"
          % (par.v_max, par.t_max, par.gear_ratio, par.effective_kd_scale))
    m.mit_zero()                        # here = 0 rad
    w = 2 * math.pi * args.hz
    period = 1.0 / args.rate
    t0 = time.monotonic()
    next_t = t0
    while (t := time.monotonic() - t0) < args.seconds:
        p_des = args.amp * math.sin(w * t)
        v_des = args.amp * w * math.cos(w * t)
        st = m.mit(p_des, v_des, args.kp, args.kd)
        print("\rdes %6.3f  act %6.3f rad  current %5.2f A"
              % (p_des, st.position_rad, st.current_a), end="")
        next_t += period
        time.sleep(max(0.0, next_t - time.monotonic()))
    print()
    m.mit_exit()
