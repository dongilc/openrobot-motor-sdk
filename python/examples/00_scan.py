"""Step 0: find the motors on the bus and print their state. Nothing moves."""
from common import parser, open_bus

from openrobot_motor import Motor, fault_name

args = parser("Find motors and read their status").parse_args()

with open_bus(args) as bus:
    ids = bus.scan(range(1, 33))
    if not ids:
        raise SystemExit("No motor answered. Check power, wiring, termination "
                         "(120 ohm at both ends), bit rate and the adapter channel.")
    print("Found motor IDs:", ids)
    for i in ids:
        m = Motor(bus, i)
        st = m.status()
        ph = m.phases()
        lim = m.limits()
        mit = m.mit_params()
        print("\nMotor %d" % i)
        print("  mode            %s" % ph.control_mode_name)
        print("  temperature     %d C" % st.temperature_c)
        print("  current         %.2f A" % st.current_a)
        print("  speed           %d deg/s (motor)" % st.speed_dps)
        print("  angle           %.2f deg (motor, multi-turn)" % m.multi_turn_deg())
        print("  current limit   %.1f A" % lim.motor_current_a)
        print("  gear ratio      %.2f" % mit.gear_ratio)
        faults = [fault_name(c) for c in m.fault_history() if c]
        print("  recent faults   %s" % (", ".join(faults) or "none"))
        out = m.output_encoder()
        if out:
            print("  output encoder  %.2f deg (valid %s)" % (out.multi_turn_deg, out.valid))
