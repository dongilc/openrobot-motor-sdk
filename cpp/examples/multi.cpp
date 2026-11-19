// Step 3: several motors on one bus, each following its own speed wave.
//   multi pcan PCAN_USBBUS1 1 2 3
// One loop sends a command to every motor in turn and reads each reply, so
// all motors are updated at the same rate. If any motor stops answering,
// every motor is released.
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include "common.hpp"

using namespace openrobot;

int main(int argc, char **argv) {
    Args a = parse_args(argc, argv);
    std::vector<int> ids;
    for (int i = 3; i < argc; ++i) ids.push_back(std::atoi(argv[i]));
    if (ids.empty()) ids.push_back(1);
    const double peak_dps = 360.0, hz = 0.5, seconds = 6.0;
    confirm_motion("turn every listed motor both ways");
    try {
        auto t = open_transport(a.interface, a.channel);
        MotorBus bus(*t);
        std::vector<std::unique_ptr<Motor>> motors;   // each released when main returns
        for (int id : ids) motors.emplace_back(new Motor(bus, (uint8_t)id));
        std::vector<int> missed(ids.size(), 0);
        const double pi = 3.14159265358979;
        auto t0 = std::chrono::steady_clock::now(), next = t0;
        long cycles = 0;
        bool ok = true;
        while (ok && !g_stop) {
            double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (t >= seconds) break;
            std::printf("\r");
            for (size_t k = 0; k < motors.size(); ++k) {
                double phase = 2 * pi * k / motors.size();
                Status s{};
                if (motors[k]->speed_dps((float)(peak_dps * std::sin(2 * pi * hz * t + phase)), &s) == OK) {
                    missed[k] = 0;
                } else if (++missed[k] >= 3) {
                    std::printf("\nmotor %d: no reply 3 times in a row: stopping\n", ids[k]);
                    ok = false;
                }
                std::printf("%d:%6d  ", ids[k], s.speed_dps);
            }
            std::printf("deg/s   ");
            std::fflush(stdout);
            ++cycles;
            next += std::chrono::milliseconds(10);
            std::this_thread::sleep_until(next);
        }
        std::printf("\n%ld cycles\n", cycles);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
