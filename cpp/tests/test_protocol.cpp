// Checks the C++ protocol core against vectors generated from the Python
// reference (gen_vectors.py) and a few decode cases.
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "openrobot_motor/protocol.hpp"
#include "openrobot_motor/bus.hpp"

using namespace openrobot;

#include "test_vectors.inc"

static int failures = 0;

static void check(bool cond, const char *what) {
    if (!cond) {
        printf("FAIL %s\n", what);
        ++failures;
    }
}

static void check_frame(const Frame &got, const uint8_t *want, const char *name) {
    if (memcmp(got.data, want, 8) != 0) {
        printf("FAIL %s\n  got ", name);
        for (int i = 0; i < 8; ++i) printf("%02X ", got.data[i]);
        printf("\n  want");
        for (int i = 0; i < 8; ++i) printf(" %02X", want[i]);
        printf("\n");
        ++failures;
    }
}

static bool near(double a, double b, double tol) { return fabs(a - b) <= tol; }

// A board that answers every request with its own code and records what it got.
class EchoTransport : public Transport {
public:
    uint8_t sent[32][2];
    int n = 0;
    bool pending = false;
    uint16_t last_id = 0;
    uint8_t last[8];
    bool send(uint16_t id, const uint8_t data[8]) override {
        if (n < 32) { sent[n][0] = data[0]; sent[n][1] = data[1]; ++n; }
        last_id = id;
        memcpy(last, data, 8);
        last[0] = reply_code(data[0]);
        pending = true;
        return true;
    }
    bool recv(uint16_t &id, uint8_t data[8], uint32_t) override {
        if (!pending) return false;
        pending = false;
        id = last_id;
        memcpy(data, last, 8);
        return true;
    }
    uint32_t now_ms() override { return 0; }
};

// speed_erpm() must release first unless the last motion was a release or eRPM.
static void test_erpm_releases_first() {
    EchoTransport t;
    MotorBus bus(t);
    Motor m(bus, 1, false);
    m.speed_dps(100);
    m.speed_erpm(1000);
    m.speed_erpm(1200);
    m.off();
    m.speed_erpm(800);
    const uint8_t want[][2] = {{SPEED, 0}, {MOTOR_OFF, 0}, {SPEED, 1}, {SPEED, 1}, {MOTOR_OFF, 0}, {SPEED, 1}};
    bool ok = t.n == 6;
    for (int i = 0; ok && i < 6; ++i) ok = t.sent[i][0] == want[i][0] && t.sent[i][1] == want[i][1];
    check(ok, "speed_erpm releases first after another mode");
}

int main() {
    for (size_t i = 0; i < sizeof(VECTORS) / sizeof(VECTORS[0]); ++i)
        check_frame(VECTORS[i].got, VECTORS[i].want, VECTORS[i].name);
    for (int i = 0; i < N_CHECKED; ++i) {
        CVec v = checked_vectors(i);
        check(v.ok, v.name);
        check_frame(v.got, v.want, v.name);
    }

    Frame f;
    check(!position_limited(0, 0, f), "position_limited rejects 0");
    check(!position_limited(0, 25001, f), "position_limited rejects 25001");
    check(!write_mit_params(45, 33, 0, 8, 9, f), "write_mit_params rejects scale 9");

    const uint8_t st[8] = {0x9C, 0xFB, 0x00, 0xF8, 0x2C, 0x01, 0x00, 0x20};
    Status s = decode_status(st);
    check(s.temperature_c == -5 && near(s.current_a, -33.0, 1e-4) && s.speed_dps == 300 &&
          near(s.angle_deg(), 180.0, 1e-4), "decode_status");

    // -123456 as 56-bit little-endian
    const uint8_t mt[8] = {0x92, 0xC0, 0x1D, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF};
    check(near(decode_multi_turn(mt), -1234.56, 1e-6), "decode_multi_turn negative");
    const uint8_t mt2[8] = {0x92, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
    check(near(decode_multi_turn(mt2), 4294967296.0 / 100.0, 1e-3), "decode_multi_turn bit 32");

    const uint8_t mp[8] = {0xC4, 45, 33, 0xD0, 0x02, 0x84, 0x03, 0x00};
    MitParams m = decode_mit_params(mp);
    check(m.v_max == 45 && m.t_max == 33 && near(m.kt, 0.72, 1e-6) && near(m.gear_ratio, 9.0, 1e-6) &&
          m.effective_kd_scale() == 1, "decode_mit_params");

    const uint8_t ms[8] = {0xC0, 40, 100, 0, 0xCE, 0xFF, 0xFF, 0x7F};
    MitState x = decode_mit_state(ms);
    check(near(x.position_rad, 12.56, 1e-4) && x.speed_dps == -50, "decode_mit_state");

    check(reply_code(WRITE_MIT_PARAMS) == READ_MIT_PARAMS, "reply_code 0xC5");
    check(strcmp(fault_name(26), "position error too large") == 0, "fault_name");
    check(strcmp(control_mode_name(12), "MIT") == 0, "control_mode_name");

    test_erpm_releases_first();

    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("all protocol tests passed\n");
    return 0;
}
