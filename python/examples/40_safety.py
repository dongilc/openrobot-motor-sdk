"""Step 4: safe control loop.

Shows the three things every application should do:

1. Watch for faults: the board sends a fault notice (0xBF) when it stops;
   on_fault reports it at once.
2. Watch the link: if a motor misses replies, release everything.
   Do not rely on the board stopping by itself when commands stop.
3. Release on exit, including Ctrl+C.

With --sim, a fault is injected after 2 s to show the reaction.
"""
import time

from common import parser, open_bus, confirm_motion

from openrobot_motor import Motor, ReplyTimeout, fault_name

p = parser("Speed loop with fault and link supervision")
p.add_argument("--dps", type=float, default=180.0)
p.add_argument("--seconds", type=float, default=10.0)
p.add_argument("--max-missed", type=int, default=3,
               help="consecutive missed replies before stopping (default 3)")
args = p.parse_args()
confirm_motion(args, "turn the motor at %.0f deg/s; press Ctrl+C to stop" % args.dps)

faulted = []


def on_fault(motor_id, code):
    faulted.append((motor_id, code))
    print("\nFAULT motor %d: %s" % (motor_id, fault_name(code)))


with open_bus(args) as bus, Motor(bus, args.id) as m:
    bus.on_fault = on_fault
    if m.clear_fault():
        raise SystemExit("A fault is still active; fix its cause first.")
    missed = 0
    injected = False
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < args.seconds and not faulted:
            if args.sim and not injected and time.monotonic() - t0 > 2.0:
                bus.sim.raise_fault(args.id, 2)         # under voltage
                injected = True
            try:
                st = m.speed_dps(args.dps)
                missed = 0
                print("\rspeed %6d deg/s  current %5.2f A  temp %d C"
                      % (st.speed_dps, st.current_a, st.temperature_c), end="")
            except ReplyTimeout:
                missed += 1
                if missed >= args.max_missed:
                    print("\nNo reply %d times in a row: stopping." % missed)
                    break
            bus.poll_faults()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nCtrl+C: stopping.")
    print()
    if faulted:
        print("Fault history:", [fault_name(c) for c in m.fault_history() if c])
# the with-block released the motor
