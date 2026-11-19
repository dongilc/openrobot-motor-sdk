// Step 1: speed control, forward then backward, with link supervision.
// Leaving main (normally, on Ctrl+C or on an error) releases the motor.
//
//   speed pcan PCAN_USBBUS1 1 360          Speed (DPS, position-tracked), deg/s
//   speed pcan PCAN_USBBUS1 1 3000 erpm    Speed (eRPM, VESC speed loop)
#include <chrono>
#include <cstring>
#include <thread>

#include "common.hpp"

using namespace openrobot;

static bool g_erpm = false;

static bool run(Motor &m, float value, double seconds) {
    auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    int missed = 0;
    while (std::chrono::steady_clock::now() < end && !g_stop) {
        Status s;
        Result r = g_erpm ? m.speed_erpm(value, &s) : m.speed_dps(value, &s);
        if (r == OK) {
            missed = 0;
            std::printf("\r%s target %6.0f %s  actual %6d deg/s  current %5.2f A   ", g_erpm ? "Speed (eRPM)" : "Speed (DPS)",
                        value, g_erpm ? "eRPM" : "deg/s", s.speed_dps, s.current_a);
            std::fflush(stdout);
        } else if (++missed >= 3) {
            std::printf("\nno reply 3 times in a row: stopping\n");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::printf("\n");
    return !g_stop;
}

int main(int argc, char **argv) {
    Args a = parse_args(argc, argv);
    float value = argc > 4 ? (float)std::atof(argv[4]) : 360.0f;
    g_erpm = argc > 5 && std::strcmp(argv[5], "erpm") == 0;
    confirm_motion("turn the motor both ways");
    try {
        auto t = open_transport(a.interface, a.channel);
        MotorBus bus(*t);
        Motor m(bus, (uint8_t)a.id);       // released when main returns
        if (run(m, value, 2.0) && run(m, -value, 2.0)) {
            if (g_erpm) m.speed_erpm(0);   // eRPM: speed 0, no position hold
            else m.stop();                 // DPS: hold where it is
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
