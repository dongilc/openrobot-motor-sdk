// Request/reply on top of any CAN transport, and a Motor with plain-unit
// methods. Header-only C++11, no heap and no exceptions, so it also runs on
// a microcontroller: implement Transport for your CAN hardware.
//
// SPDX-License-Identifier: MIT
#ifndef OPENROBOT_MOTOR_BUS_HPP
#define OPENROBOT_MOTOR_BUS_HPP

#include "protocol.hpp"

namespace openrobot {

// One CAN adapter or controller. Standard 11-bit frames, DLC 8.
class Transport {
public:
    virtual ~Transport() {}
    virtual bool send(uint16_t id, const uint8_t data[8]) = 0;
    // Wait up to timeout_ms for one standard frame. false on timeout.
    virtual bool recv(uint16_t &id, uint8_t data[8], uint32_t timeout_ms) = 0;
    virtual uint32_t now_ms() = 0;
};

enum Result { OK = 0, NO_REPLY = 1, BAD_ARGUMENT = 2 };

typedef void (*FaultCallback)(uint8_t motor_id, uint8_t fault_code, void *user);

class MotorBus {
public:
    explicit MotorBus(Transport &t) : t_(t), on_fault_(0), user_(0), last_fault_id_(0), last_fault_(0) {}

    // Called for every fault notice (0xBF) seen on the bus.
    void on_fault(FaultCallback cb, void *user = 0) { on_fault_ = cb; user_ = user; }
    uint8_t last_fault() const { return last_fault_; }
    uint8_t last_fault_motor() const { return last_fault_id_; }

    // Send req to motor_id and copy the matching reply into reply[8].
    Result request(uint8_t motor_id, const Frame &req, uint8_t reply[8],
                   uint32_t timeout_ms = 20, int retries = 1) {
        const uint8_t want = reply_code(req.data[0]);
        const uint16_t id = can_id(motor_id);
        for (int a = 0; a <= retries; ++a) {
            drain();
            if (!t_.send(id, req.data)) continue;
            const uint32_t start = t_.now_ms();
            for (;;) {
                uint32_t spent = t_.now_ms() - start;
                if (spent >= timeout_ms) break;
                uint16_t rid;
                uint8_t d[8];
                if (!t_.recv(rid, d, timeout_ms - spent)) break;
                if (!accept(rid, d)) continue;
                if (rid == id && d[0] == want) {
                    for (int i = 0; i < 8; ++i) reply[i] = d[i];
                    return OK;
                }
            }
        }
        return NO_REPLY;
    }

    // Check for fault notices without sending anything.
    void poll() { drain(); }

    // Fill ids[] with the motors that answer a harmless read (0x92) in
    // first..last. Returns how many were found.
    int scan(uint8_t *ids, int max_ids, uint8_t first = 1, uint8_t last = 32) {
        int n = 0;
        uint8_t r[8];
        for (int i = first; i <= last && n < max_ids; ++i)
            if (request((uint8_t)i, openrobot::request(READ_MULTI_TURN), r, 10, 0) == OK) ids[n++] = (uint8_t)i;
        return n;
    }

private:
    // false for frames that are not replies (fault notices are recorded).
    bool accept(uint16_t id, const uint8_t *d) {
        if (d[0] != FAULT_NOTICE) return true;
        last_fault_id_ = (uint8_t)(id - BASE_ID);
        last_fault_ = d[1];
        if (on_fault_) on_fault_(last_fault_id_, last_fault_, user_);
        return false;
    }

    void drain() {
        uint16_t id;
        uint8_t d[8];
        while (t_.recv(id, d, 0)) accept(id, d);
    }

    Transport &t_;
    FaultCallback on_fault_;
    void *user_;
    uint8_t last_fault_id_, last_fault_;
};

// One motor. By default the destructor releases it (0x80), so a Motor on
// the stack stops the motor whenever the scope is left. Pass
// release_on_exit = false for read-only use, e.g. to inspect a joint that is
// holding position under another program's control.
class Motor {
public:
    Motor(MotorBus &bus, uint8_t id, bool release_on_exit = true, uint32_t timeout_ms = 20)
        : bus_(bus), id_(id), timeout_ms_(timeout_ms), have_mit_(false), release_(release_on_exit),
          erpm_ready_(false) {}
    ~Motor() {
        if (release_) off();
    }

    uint8_t id() const { return id_; }

    // Motion. Each returns OK and fills the motor-side status, or NO_REPLY.
    Result off(Status *s = 0) { return status_req(motor_off(), s); }
    Result stop(Status *s = 0) { return status_req(motor_stop(), s); }
    Result current(float amps, Status *s = 0, float damping = 0) { return status_req(openrobot::current(amps, damping), s); }
    // Speed (DPS, position-tracked), 0xA2 mode 0: deg/s, motor side.
    Result speed_dps(float dps, Status *s = 0) { return status_req(openrobot::speed_dps(dps), s); }
    // Speed (eRPM, VESC speed loop), 0xA2 mode 1: electrical RPM. The board
    // ignores it while another motion mode is active, so this releases first
    // unless the last motion command was a release or eRPM itself.
    Result speed_erpm(float erpm, Status *s = 0) {
        if (!erpm_ready_) {
            Result e = off();
            if (e != OK) return e;
        }
        return status_req(openrobot::speed_erpm(erpm), s);
    }
    // Same as speed_dps(), kept for existing code.
    Result speed(float dps, Status *s = 0) { return speed_dps(dps, s); }
    Result position(float deg, float max_dps, Status *s = 0) {
        Frame f;
        if (!position_limited(deg, max_dps, f)) return BAD_ARGUMENT;
        return status_req(f, s);
    }
    Result duty(float fraction, Status *s = 0) { return status_req(openrobot::duty(fraction), s); }
    Result brake(float amps, Status *s = 0) { return status_req(openrobot::brake(amps), s); }

    // Reads
    Result status(Status &s) { return status_req(request(READ_STATUS), &s); }
    Result multi_turn_deg(double &deg) {
        uint8_t r[8];
        Result e = bus_.request(id_, request(READ_MULTI_TURN), r, timeout_ms_);
        if (e == OK) deg = decode_multi_turn(r);
        return e;
    }
    Result phases(Phases &p) {
        uint8_t r[8];
        Result e = bus_.request(id_, request(READ_PHASES), r, timeout_ms_);
        if (e == OK) p = decode_phases(r);
        return e;
    }
    Result limits(Limits &l) {
        uint8_t r[8];
        Result e = bus_.request(id_, request(READ_LIMITS), r, timeout_ms_);
        if (e == OK) l = decode_limits(r);
        return e;
    }
    Result fault_history(uint8_t codes[7]) {
        uint8_t r[8];
        Result e = bus_.request(id_, request(READ_FAULTS), r, timeout_ms_);
        if (e == OK) decode_faults(r, codes);
        return e;
    }
    // remaining = fault code left after clearing (0 = cleared)
    Result clear_fault(uint8_t &remaining) {
        uint8_t r[8];
        Result e = bus_.request(id_, request(CLEAR_FAULT), r, timeout_ms_);
        if (e == OK) remaining = r[1];
        return e;
    }
    Result mit_params(MitParams &m) {
        uint8_t r[8];
        Result e = bus_.request(id_, request(READ_MIT_PARAMS), r, timeout_ms_);
        if (e == OK) m = decode_mit_params(r);
        return e;
    }

    // MIT: reads the board's ranges, then enters MIT mode.
    Result mit_enter() {
        Result e = mit_params(mit_);
        if (e != OK) return e;
        uint8_t r[8];
        e = bus_.request(id_, request(MIT_ENTER), r, timeout_ms_);
        have_mit_ = (e == OK);
        erpm_ready_ = false;
        return e;
    }
    Result mit_zero(float offset_deg = 0) {
        uint8_t r[8];
        return bus_.request(id_, mit_set_zero(offset_deg), r, timeout_ms_);
    }
    Result mit(float p_rad, float v_rads, float kp, float kd, float t_nm, MitState *s = 0) {
        if (!have_mit_) return BAD_ARGUMENT;
        uint8_t r[8];
        Result e = bus_.request(id_, mit_control(p_rad, v_rads, kp, kd, t_nm, mit_), r, timeout_ms_);
        if (e == OK && s) *s = decode_mit_state(r);
        return e;
    }
    Result mit_exit() {
        uint8_t r[8];
        have_mit_ = false;
        Result e = bus_.request(id_, request(MIT_EXIT), r, timeout_ms_);
        if (e == OK) erpm_ready_ = true;
        return e;
    }

private:
    Result status_req(const Frame &f, Status *s) {
        uint8_t r[8];
        Result e = bus_.request(id_, f, r, timeout_ms_);
        if (e == OK && s) *s = decode_status(r);
        if (e == OK) erpm_ready_ = f.data[0] == MOTOR_OFF || (f.data[0] == SPEED && f.data[1] == 1);
        return e;
    }

    MotorBus &bus_;
    uint8_t id_;
    uint32_t timeout_ms_;
    MitParams mit_;
    bool have_mit_;
    bool release_;
    bool erpm_ready_;
};

}  // namespace openrobot

#endif  // OPENROBOT_MOTOR_BUS_HPP
