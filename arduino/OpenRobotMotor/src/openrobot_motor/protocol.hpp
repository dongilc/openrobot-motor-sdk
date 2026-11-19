// OpenRobot CAN protocol: frame encoding and decoding, no I/O.
//
// Header-only C++11. No heap, no exceptions, no STL, so the same file builds
// for a PC, ROS 2, Android NDK and Arduino (AVR included). The layout follows
// docs/can_protocol.md; python/openrobot_motor/protocol.py is the reference
// implementation and tests/test_vectors.inc is generated from it.
//
// SPDX-License-Identifier: MIT
#ifndef OPENROBOT_MOTOR_PROTOCOL_HPP
#define OPENROBOT_MOTOR_PROTOCOL_HPP

#include <stdint.h>

namespace openrobot {

static const uint16_t BASE_ID = 0x140;
static const float CURRENT_LSB = 33.0f / 2048.0f;     // A per LSB
static const uint16_t ANGLE_14BIT = 16384;
static const uint32_t DEFAULT_BITRATE = 1000000UL;
static const int16_t SPEED_LIMIT_MAX = 25000;

static const float MIT_P_MAX = 12.56f;
static const float MIT_KP_MAX = 500.0f;
static const float MIT_KD_MAX = 5.0f;

enum Command : uint8_t {
    MOTOR_OFF = 0x80, MOTOR_STOP = 0x81, MOTOR_START = 0x88,
    CURRENT = 0xA1, SPEED = 0xA2, POSITION = 0xA3, POSITION_LIMITED = 0xA4,
    DUTY = 0xA5, BRAKE = 0xA6,
    READ_STATUS = 0x9C, READ_PHASES = 0x9D, READ_MULTI_TURN = 0x92,
    READ_OUTPUT_ENCODER = 0x93, READ_ENCODER = 0x90, READ_ENCODER_STATUS = 0x9E,
    CLEAR_FAULT = 0x9B, READ_FAULTS = 0xB0,
    READ_PID = 0x30, WRITE_PID_RAM = 0x31, WRITE_PID_FLASH = 0x32,
    READ_LIMITS = 0xB1, WRITE_LIMITS_FLASH = 0xB2,
    WRITE_ENCODER_OFFSET = 0x91, SET_ZERO_HERE = 0x19,
    MIT_CONTROL = 0xC0, MIT_ENTER = 0xC1, MIT_EXIT = 0xC2, MIT_SET_ZERO = 0xC3,
    READ_MIT_PARAMS = 0xC4, WRITE_MIT_PARAMS = 0xC5,
    FAULT_NOTICE = 0xBF
};

struct Frame {
    uint8_t data[8];
};

inline uint16_t can_id(uint8_t motor_id) { return BASE_ID + motor_id; }

// Code the reply carries for a request (0xC5 is answered with 0xC4).
inline uint8_t reply_code(uint8_t request) {
    return request == WRITE_MIT_PARAMS ? (uint8_t)READ_MIT_PARAMS : request;
}

namespace detail {

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

inline long roundl_(float x) { return x >= 0.0f ? (long)(x + 0.5f) : -(long)(-x + 0.5f); }

inline int16_t to_i16(float x) { return (int16_t)roundl_(clampf(x, -32768.0f, 32767.0f)); }

inline int32_t to_i32(float x) {
    // Clamp in double-free float space; 2^31 is exactly representable.
    if (x >= 2147483520.0f) return 2147483647L;
    if (x <= -2147483648.0f) return -2147483647L - 1;
    return (int32_t)roundl_(x);
}

inline Frame frame(uint8_t cmd) {
    Frame f;
    for (int i = 0; i < 8; ++i) f.data[i] = 0;
    f.data[0] = cmd;
    return f;
}

inline void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline void put_i16(uint8_t *p, int16_t v) { put_u16(p, (uint16_t)v); }
inline void put_i32(uint8_t *p, int32_t v) {
    uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)u; p[1] = (uint8_t)(u >> 8); p[2] = (uint8_t)(u >> 16); p[3] = (uint8_t)(u >> 24);
}
inline uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
inline int16_t get_i16(const uint8_t *p) { return (int16_t)get_u16(p); }
inline int32_t get_i32(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

inline uint16_t f2u(float x, float lo, float hi, int bits) {
    float span = (float)((1UL << bits) - 1);
    return (uint16_t)roundl_((clampf(x, lo, hi) - lo) / (hi - lo) * span);
}
inline float u2f(uint16_t u, float lo, float hi, int bits) {
    return (float)u * (hi - lo) / (float)((1UL << bits) - 1) + lo;
}

inline int16_t amps_raw(float a) { return to_i16(a / CURRENT_LSB); }

}  // namespace detail

// ---------------------------------------------------------------- status --

// Common reply of motion commands and 0x9C. Motor side.
struct Status {
    int8_t temperature_c;
    float current_a;
    int16_t speed_dps;
    uint16_t angle_raw;              // 0..16383
    float angle_deg() const { return angle_raw * 360.0f / ANGLE_14BIT; }
};

inline Status decode_status(const uint8_t *d) {
    Status s;
    s.temperature_c = (int8_t)d[1];
    s.current_a = detail::get_i16(d + 2) * CURRENT_LSB;
    s.speed_dps = detail::get_i16(d + 4);
    s.angle_raw = detail::get_u16(d + 6);
    return s;
}

// ---------------------------------------------------------------- motion --

inline Frame request(uint8_t cmd) { return detail::frame(cmd); }
inline Frame motor_off() { return detail::frame(MOTOR_OFF); }
inline Frame motor_stop() { return detail::frame(MOTOR_STOP); }
inline Frame motor_start() { return detail::frame(MOTOR_START); }

// 0xA1. damping 0 = pure current control, otherwise 0.01..100.
inline Frame current(float amps, float damping = 0.0f) {
    Frame f = detail::frame(CURRENT);
    detail::put_i16(f.data + 4, detail::amps_raw(amps));
    detail::put_i16(f.data + 6, detail::to_i16(detail::clampf(damping, 0.0f, 100.0f) * 100.0f));
    return f;
}

// 0xA2 mode 0, Speed (DPS, position-tracked): motor speed in deg/s.
inline Frame speed_dps(float dps) {
    Frame f = detail::frame(SPEED);
    detail::put_i32(f.data + 4, detail::to_i32(dps * 100.0f));
    return f;
}

// 0xA2 mode 1, Speed (eRPM, VESC speed loop): electrical RPM.
inline Frame speed_erpm(float erpm) {
    Frame f = detail::frame(SPEED);
    f.data[1] = 1;
    detail::put_i32(f.data + 4, detail::to_i32(erpm));
    return f;
}

// 0xA3, multi-turn motor angle, no speed limit.
inline Frame position(float deg) {
    Frame f = detail::frame(POSITION);
    detail::put_i32(f.data + 4, detail::to_i32(deg * 100.0f));
    return f;
}

// 0xA4, multi-turn motor angle with a speed limit of 1..25000 deg/s.
// Returns false for an out-of-range limit: the board would reply but not move.
inline bool position_limited(float deg, float max_dps, Frame &out) {
    long lim = detail::roundl_(max_dps);
    if (lim < 1 || lim > SPEED_LIMIT_MAX) return false;
    out = detail::frame(POSITION_LIMITED);
    detail::put_i16(out.data + 2, (int16_t)lim);
    detail::put_i32(out.data + 4, detail::to_i32(deg * 100.0f));
    return true;
}

// 0xA5, -1.0..1.0.
inline Frame duty(float fraction) {
    Frame f = detail::frame(DUTY);
    detail::put_i16(f.data + 4, detail::to_i16(detail::clampf(fraction, -1.0f, 1.0f) * 10000.0f));
    return f;
}

// 0xA6, brake current (magnitude).
inline Frame brake(float amps) {
    Frame f = detail::frame(BRAKE);
    detail::put_i16(f.data + 4, detail::amps_raw(amps < 0 ? -amps : amps));
    return f;
}

// ---------------------------------------------------------------- reads --

// 0x92 -> motor multi-turn angle, degrees (56-bit signed, 0.01 deg).
// double on a PC; on AVR double is 32-bit, which keeps 0.01 deg resolution
// up to about +/-460 turns and gets coarser beyond.
inline double decode_multi_turn(const uint8_t *d) {
    // Split into a signed high and an unsigned low part instead of 64-bit
    // integer math, which AVR handles poorly.
    int32_t hi = (int32_t)((uint32_t)d[5] | ((uint32_t)d[6] << 8) | ((uint32_t)d[7] << 16));
    if (hi & 0x800000L) hi -= 0x1000000L;               // 24-bit signed
    uint32_t lo = (uint32_t)d[1] | ((uint32_t)d[2] << 8) | ((uint32_t)d[3] << 16) | ((uint32_t)d[4] << 24);
    return ((double)hi * 4294967296.0 + (double)lo) / 100.0;
}

struct OutputEncoder {
    float angle_deg;                 // single turn, 0..360
    float multi_turn_deg;
    bool valid, calibrated, dual_calibrated;
};

inline OutputEncoder decode_output_encoder(const uint8_t *d) {
    OutputEncoder o;
    o.angle_deg = detail::get_u16(d + 1) * 360.0f / 65536.0f;
    o.multi_turn_deg = detail::get_i32(d + 3) / 100.0f;
    o.valid = (d[7] & 1) != 0;
    o.calibrated = (d[7] & 2) != 0;
    o.dual_calibrated = (d[7] & 4) != 0;
    return o;
}

struct Encoder {
    uint8_t error_bits;
    uint16_t angle_raw, raw_angle_raw, offset_raw;
};

inline Encoder decode_encoder(const uint8_t *d) {
    Encoder e;
    e.error_bits = d[1];
    e.angle_raw = detail::get_u16(d + 2);
    e.raw_angle_raw = detail::get_u16(d + 4);
    e.offset_raw = detail::get_u16(d + 6);
    return e;
}

struct EncoderStatus {
    uint16_t air_gap_um, signal_level;
    float error_rate_pct;
    uint8_t crc_status;
    bool error, warning;
};

inline EncoderStatus decode_encoder_status(const uint8_t *d) {
    EncoderStatus e;
    e.air_gap_um = detail::get_u16(d + 1);
    e.signal_level = detail::get_u16(d + 3);
    e.error_rate_pct = d[5] * 0.5f;
    e.crc_status = d[6];
    e.error = (d[7] & 1) != 0;
    e.warning = (d[7] & 2) != 0;
    return e;
}

struct Phases {
    uint8_t control_mode;            // see control_mode_name()
    float ia, ib, ic;
};

inline Phases decode_phases(const uint8_t *d) {
    Phases p;
    p.control_mode = d[1];
    p.ia = detail::get_i16(d + 2) / 64.0f;
    p.ib = detail::get_i16(d + 4) / 64.0f;
    p.ic = detail::get_i16(d + 6) / 64.0f;
    return p;
}

inline const char *control_mode_name(uint8_t m) {
    static const char *const names[] = {
        "none", "released", "current brake", "duty", "current", "damped current",
        "position", "position (multi-turn)", "speed (timeout)", "speed",
        "position with speed limit", "trajectory", "MIT"};
    return m < sizeof(names) / sizeof(names[0]) ? names[m] : "unknown";
}

// 0xB0 -> the last seven fault codes, copied to out[0..6].
inline void decode_faults(const uint8_t *d, uint8_t out[7]) {
    for (int i = 0; i < 7; ++i) out[i] = d[i + 1];
}

inline const char *fault_name(uint8_t code) {
    static const char *const names[] = {
        "none", "over voltage", "under voltage", "gate driver", "over current",
        "over temperature, power stage", "over temperature, motor",
        "gate driver over voltage", "gate driver under voltage", "MCU under voltage",
        "watchdog reset at boot", "encoder (SPI)", "encoder amplitude low",
        "encoder amplitude high", "flash corruption",
        "current sensor offset high, phase 1", "current sensor offset high, phase 2",
        "current sensor offset high, phase 3", "unbalanced currents", "brake",
        "resolver LOT", "resolver DOS", "resolver LOS",
        "flash corruption, app configuration", "flash corruption, motor configuration",
        "encoder no magnet", "position error too large"};
    return code < sizeof(names) / sizeof(names[0]) ? names[code] : "unknown";
}

// ---------------------------------------------------------------- settings --

struct Pid {
    float kp, ki, kd;
};

// 0x31 (RAM) or 0x32 (flash). Returns false if a gain does not fit.
inline bool write_pid(const Pid &pid, bool flash, Frame &out) {
    long p = detail::roundl_(pid.kp * 1000.0f), i = detail::roundl_(pid.ki * 100000.0f),
         d = detail::roundl_(pid.kd * 100000.0f);
    if (p < 0 || p > 0xFFFF || i < 0 || i > 0xFFFF || d < 0 || d > 0xFFFF) return false;
    out = detail::frame(flash ? WRITE_PID_FLASH : WRITE_PID_RAM);
    detail::put_u16(out.data + 2, (uint16_t)p);
    detail::put_u16(out.data + 4, (uint16_t)i);
    detail::put_u16(out.data + 6, (uint16_t)d);
    return true;
}

inline Pid decode_pid(const uint8_t *d) {
    Pid p;
    p.kp = detail::get_u16(d + 2) / 1000.0f;
    p.ki = detail::get_u16(d + 4) / 100000.0f;
    p.kd = detail::get_u16(d + 6) / 100000.0f;
    return p;
}

struct Limits {
    float motor_current_a, abs_current_a, battery_current_a;
    uint8_t gate_oc_mode;
};

// 0xB2 (flash). Returns false if a limit does not fit.
inline bool write_limits(const Limits &l, Frame &out) {
    long m = detail::roundl_(l.motor_current_a * 100.0f), a = detail::roundl_(l.abs_current_a * 100.0f),
         b = detail::roundl_(l.battery_current_a * 100.0f);
    if (m < 0 || m > 0xFFFF || a < 0 || a > 0xFFFF || b < 0 || b > 0xFFFF) return false;
    out = detail::frame(WRITE_LIMITS_FLASH);
    out.data[1] = l.gate_oc_mode;
    detail::put_u16(out.data + 2, (uint16_t)m);
    detail::put_u16(out.data + 4, (uint16_t)a);
    detail::put_u16(out.data + 6, (uint16_t)b);
    return true;
}

inline Limits decode_limits(const uint8_t *d) {
    Limits l;
    l.gate_oc_mode = d[1];
    l.motor_current_a = detail::get_u16(d + 2) / 100.0f;
    l.abs_current_a = detail::get_u16(d + 4) / 100.0f;
    l.battery_current_a = detail::get_u16(d + 6) / 100.0f;
    return l;
}

inline Frame write_encoder_offset(uint16_t offset_raw) {
    Frame f = detail::frame(WRITE_ENCODER_OFFSET);
    detail::put_u16(f.data + 6, offset_raw);
    return f;
}

// 0x91 / 0x19 reply -> stored offset, 14-bit.
inline uint16_t decode_offset(const uint8_t *d) { return detail::get_u16(d + 6); }

// ---------------------------------------------------------------- MIT --

struct MitParams {
    float v_max;                     // rad/s, output side
    float t_max;                     // Nm, output side
    float kt;                        // reply: output-side Kt, Nm/A
    float gear_ratio;
    uint8_t kd_scale;                // 0 in the reply = firmware without scale
    uint8_t effective_kd_scale() const { return kd_scale ? kd_scale : 1; }
};

inline MitParams decode_mit_params(const uint8_t *d) {
    MitParams m;
    m.v_max = d[1];
    m.t_max = d[2];
    m.kt = detail::get_u16(d + 3) / 1000.0f;
    m.gear_ratio = detail::get_u16(d + 5) / 100.0f;
    m.kd_scale = d[7];
    return m;
}

// 0xC5. motor_kt is the motor-side Kt (0 = from the motor setup);
// kd_scale 0 keeps the stored scale. Returns false for kd_scale > 8.
inline bool write_mit_params(float v_max, float t_max, float motor_kt, float gear_ratio,
                             uint8_t kd_scale, Frame &out) {
    if (kd_scale > 8) return false;
    out = detail::frame(WRITE_MIT_PARAMS);
    out.data[1] = (uint8_t)detail::roundl_(detail::clampf(v_max, 0.0f, 255.0f));
    out.data[2] = (uint8_t)detail::roundl_(detail::clampf(t_max, 0.0f, 255.0f));
    detail::put_u16(out.data + 3, (uint16_t)detail::roundl_(detail::clampf(motor_kt * 1000.0f, 0.0f, 65535.0f)));
    detail::put_u16(out.data + 5, (uint16_t)detail::roundl_(detail::clampf(gear_ratio * 100.0f, 0.0f, 65535.0f)));
    out.data[7] = kd_scale;
    return true;
}

// 0xC0. Output-side targets; v and t ranges come from the board (0xC4).
inline Frame mit_control(float p_rad, float v_rads, float kp, float kd, float t_nm,
                         const MitParams &par) {
    float kd_max = MIT_KD_MAX * par.effective_kd_scale();
    uint16_t p = detail::f2u(p_rad, -MIT_P_MAX, MIT_P_MAX, 16);
    uint16_t v = detail::f2u(v_rads, -par.v_max, par.v_max, 12);
    uint16_t k = detail::f2u(kp, 0.0f, MIT_KP_MAX, 12);
    uint16_t d = detail::f2u(kd, 0.0f, kd_max, 8);
    uint16_t t = detail::f2u(t_nm, -par.t_max, par.t_max, 8);
    Frame f;
    f.data[0] = MIT_CONTROL;
    f.data[1] = (uint8_t)(p >> 8);
    f.data[2] = (uint8_t)p;
    f.data[3] = (uint8_t)(v >> 4);
    f.data[4] = (uint8_t)(((v & 0x0F) << 4) | (k >> 8));
    f.data[5] = (uint8_t)k;
    f.data[6] = (uint8_t)d;
    f.data[7] = (uint8_t)t;
    return f;
}

// 0xC0 reply. Output side, position relative to the MIT zero.
struct MitState {
    int8_t temperature_c;
    float current_a;
    int16_t speed_dps;
    float position_rad;
};

inline MitState decode_mit_state(const uint8_t *d) {
    MitState s;
    s.temperature_c = (int8_t)d[1];
    s.current_a = detail::get_i16(d + 2) * CURRENT_LSB;
    s.speed_dps = detail::get_i16(d + 4);
    s.position_rad = detail::get_i16(d + 6) * MIT_P_MAX / 32767.0f;
    return s;
}

// 0xC3. The present output position becomes offset_deg.
inline Frame mit_set_zero(float offset_deg = 0.0f) {
    Frame f = detail::frame(MIT_SET_ZERO);
    detail::put_i16(f.data + 6, detail::to_i16(offset_deg * 100.0f));
    return f;
}

}  // namespace openrobot

#endif  // OPENROBOT_MOTOR_PROTOCOL_HPP
