"""EtherCAT step 0: find the slaves, read their settings and state. Nothing moves."""
from ecat_common import parser, open_bus

args = parser("Find OpenRobot EtherCAT slaves").parse_args()
with open_bus(args) as bus:
    print("%d slave(s), PDO cycle %.1f ms" % (len(bus.motors), bus.period * 1e3))
    for m in bus.motors:
        st = m.status()
        fw = m.slave.sdo_read(0x2070, 1).rstrip(b"\0").decode(errors="replace")
        print("\nSlave %d: %s, firmware %s, motor ID %d" % (m.index, m.slave.name, fw, m.motor_id))
        print("  gear ratio      %.2f" % m.gear_ratio)
        print("  Kt, output      %.3f Nm/A" % m.kt_output)
        print("  PDO timeout     %d ms" % m.sdo_read(0x2030, 7, "<I"))
        print("  Big Error       %.0f deg (output)" % m.sdo_read(0x2030, 8, "<f"))
        print("  mode            %s" % st.mode_name)
        print("  fault           %s" % st.fault_name)
        print("  bus voltage     %.1f V" % st.bus_voltage)
        print("  temperature     motor %d C, MOSFET %d C" % (st.temp_motor_c, st.temp_mosfet_c))
        print("  angle           %.2f deg (motor, multi-turn)" % st.position_deg)
