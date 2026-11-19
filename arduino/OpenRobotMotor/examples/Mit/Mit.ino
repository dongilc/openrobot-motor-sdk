// Step 2: MIT impedance control on the output shaft.
// torque = kp (p_des - p) + kd (v_des - v) + t_ff
//
// Takes the present position as zero and follows a 0.5 Hz sine of +/-0.5 rad
// for 6 s, then releases. Type "yes" in the Serial Monitor to start; any key
// stops. The command rate this reaches depends on the board (prints at end).
#include <OpenRobotMotor.h>
#include <math.h>

using namespace openrobot;

const uint8_t CAN_CS_PIN = 9;     // Seeed CAN-BUS Shield V2: 9, V1.x: 10
const uint8_t MOTOR_ID = 1;
const float AMP = 0.5f, HZ = 0.5f, KP = 10.0f, KD = 0.5f;
const uint32_t PERIOD_US = 5000;  // 200 Hz
const uint32_t RUN_MS = 6000;

Mcp2515Transport can(CAN_CS_PIN);
MotorBus bus(can);
Motor motor(bus, MOTOR_ID);

bool waitForYes() {
    Serial.println(F("This sketch moves the output shaft +/-0.5 rad. Make sure it is free."));
    Serial.println(F("Type yes to start:"));
    String s;
    for (;;) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (s.length()) return s == "yes";
            } else {
                s += c;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    if (!can.begin(CAN_1000KBPS, MCP_16MHZ)) {
        Serial.println(F("MCP2515 not found: check CAN_CS_PIN and the shield."));
        return;
    }
    if (!waitForYes()) {
        Serial.println(F("cancelled"));
        return;
    }
    if (motor.mit_enter() != OK || motor.mit_zero() != OK) {
        Serial.println(F("motor does not answer"));
        return;
    }
    uint32_t t0 = micros(), next = t0;
    uint32_t sent = 0, missed = 0;
    float err_sum = 0;
    while (micros() - t0 < RUN_MS * 1000UL && !Serial.available()) {
        float t = (micros() - t0) * 1e-6f;
        float w = 2.0f * (float)M_PI * HZ;
        float p_des = AMP * sinf(w * t), v_des = AMP * w * cosf(w * t);
        MitState s;
        if (motor.mit(p_des, v_des, KP, KD, 0.0f, &s) == OK) {
            ++sent;
            err_sum += fabsf(p_des - s.position_rad);
        } else if (++missed >= 3) {
            Serial.println(F("no reply 3 times in a row"));
            break;
        }
        next += PERIOD_US;
        while ((int32_t)(micros() - next) < 0) {}
    }
    motor.mit_exit();
    motor.off();
    Serial.print(F("commands ")); Serial.print(sent);
    Serial.print(F(", missed ")); Serial.print(missed);
    Serial.print(F(", mean |error| ")); Serial.print(sent ? err_sum / sent : 0.0f, 3); Serial.println(F(" rad"));
}

void loop() {}
