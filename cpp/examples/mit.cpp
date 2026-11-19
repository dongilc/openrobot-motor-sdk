// Step 2: MIT impedance control on the output shaft, a 0.5 Hz sine of
// +/-0.5 rad at 200 Hz. torque = kp (p_des - p) + kd (v_des - v) + t_ff
#include <chrono>
#include <cmath>
#include <thread>

#include "common.hpp"

using namespace openrobot;

int main(int argc, char **argv) {
    Args a = parse_args(argc, argv);
    const float amp = 0.5f, hz = 0.5f, kp = 10.0f, kd = 0.5f, seconds = 6.0f;
    confirm_motion("follow a +/-0.5 rad sine on the output shaft");
    try {
        auto t = open_transport(a.interface, a.channel);
        MotorBus bus(*t);
        Motor m(bus, (uint8_t)a.id);
        if (m.mit_enter() != OK || m.mit_zero() != OK) {
            std::fprintf(stderr, "motor %d does not answer\n", a.id);
            return 1;
        }
        using clk = std::chrono::steady_clock;
        const auto t0 = clk::now();
        auto next = t0;
        int sent = 0, missed = 0;
        double err_sum = 0;
        for (;;) {
            double t_s = std::chrono::duration<double>(clk::now() - t0).count();
            if (t_s >= seconds || g_stop) break;
            float w = 2.0f * 3.14159265f * hz;
            float p_des = amp * std::sin(w * (float)t_s), v_des = amp * w * std::cos(w * (float)t_s);
            MitState s;
            if (m.mit(p_des, v_des, kp, kd, 0.0f, &s) == OK) {
                ++sent;
                err_sum += std::fabs(p_des - s.position_rad);
                std::printf("\rdes %6.3f  act %6.3f rad  current %5.2f A   ", p_des, s.position_rad, s.current_a);
                std::fflush(stdout);
            } else if (++missed >= 3) {
                std::printf("\nno reply 3 times in a row: stopping\n");
                break;
            }
            next += std::chrono::microseconds(5000);
            std::this_thread::sleep_until(next);
        }
        m.mit_exit();
        std::printf("\n%d commands, %d missed, mean |error| %.3f rad\n", sent, missed, sent ? err_sum / sent : 0.0);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
