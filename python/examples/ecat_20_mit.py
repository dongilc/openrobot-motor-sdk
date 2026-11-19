"""EtherCAT step 2: MIT impedance control on the output shaft.

    torque = kp * (p_des - p) + kd * (v_des - v) + t_ff

Takes the present position as zero (SDO 0x2050:05), follows a slow sine,
then releases. The command is refreshed every PDO cycle by the bus thread;
this loop only updates the target.
"""
import math
import time

from ecat_common import parser, open_bus, confirm_motion

p = parser("Follow a sine in MIT mode over EtherCAT")
p.add_argument("--amp", type=float, default=0.5, help="amplitude, rad output (default 0.5)")
p.add_argument("--hz", type=float, default=0.5, help="frequency (default 0.5 Hz)")
p.add_argument("--kp", type=float, default=10.0, help="Nm/rad (default 10)")
p.add_argument("--kd", type=float, default=0.5, help="Nm s/rad (default 0.5)")
p.add_argument("--seconds", type=float, default=6.0)
args = p.parse_args()
confirm_motion("follow a +/-%.2f rad sine on the output shaft" % args.amp)

with open_bus(args) as bus:
    m = bus.motors[args.slave]
    m.mit_zero()                        # here = 0 rad; command 0 first (Big Error check)
    m.mit(0.0, 0.0, args.kp, args.kd)
    time.sleep(0.1)
    w = 2 * math.pi * args.hz
    t0 = time.monotonic()
    n, err_sum = 0, 0.0
    while (t := time.monotonic() - t0) < args.seconds:
        p_des = args.amp * math.sin(w * t)
        m.mit(p_des, args.amp * w * math.cos(w * t), args.kp, args.kd)
        time.sleep(0.005)
        st = m.status()
        if st.fault_code:
            raise SystemExit("\nfault: %s" % st.fault_name)
        p_act = math.radians(st.position_deg)          # output side in MIT mode
        n, err_sum = n + 1, err_sum + abs(p_des - p_act)
        print("\rdes %6.3f  act %6.3f rad  current %5.2f A" % (p_des, p_act, st.current_a), end="")
    print("\nmean |error| %.3f rad over %d samples; cycles %d, missed %d, longest gap %.1f ms"
          % (err_sum / max(n, 1), n, bus.cycles, bus.wkc_errors, bus.max_cycle_s * 1e3))
