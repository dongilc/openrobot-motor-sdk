// Step 0: find motors and print their state. Nothing moves.
#include "common.hpp"

using namespace openrobot;

int main(int argc, char **argv) {
    Args a = parse_args(argc, argv);
    try {
        auto t = open_transport(a.interface, a.channel);
        MotorBus bus(*t);
        uint8_t ids[32];
        int n = bus.scan(ids, 32);
        if (n == 0) {
            std::printf("No motor answered. Check power, wiring, 120 ohm termination, bit rate and channel.\n");
            return 1;
        }
        for (int k = 0; k < n; ++k) {
            Motor m(bus, ids[k], /*release_on_exit=*/false);   // read only
            Status s;
            Phases p;
            Limits l;
            MitParams mp;
            double deg = 0;
            uint8_t faults[7];
            if (m.status(s) || m.phases(p) || m.limits(l) || m.mit_params(mp) || m.multi_turn_deg(deg) ||
                m.fault_history(faults)) {
                std::printf("Motor %d: no reply\n", ids[k]);
                continue;
            }
            std::printf("\nMotor %d\n", ids[k]);
            std::printf("  mode            %s\n", control_mode_name(p.control_mode));
            std::printf("  temperature     %d C\n", s.temperature_c);
            std::printf("  current         %.2f A\n", s.current_a);
            std::printf("  speed           %d deg/s (motor)\n", s.speed_dps);
            std::printf("  angle           %.2f deg (motor, multi-turn)\n", deg);
            std::printf("  current limit   %.1f A\n", l.motor_current_a);
            std::printf("  gear ratio      %.2f\n", mp.gear_ratio);
            std::printf("  recent faults  ");
            bool any = false;
            for (int i = 0; i < 7; ++i)
                if (faults[i]) { std::printf(" [%s]", fault_name(faults[i])); any = true; }
            std::printf("%s\n", any ? "" : " none");
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
