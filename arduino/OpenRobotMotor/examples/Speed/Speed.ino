// Step 1: speed control, forward then backward, then release.
// USE_ERPM picks the controller (see docs/can_protocol.md, 0xA2):
//   false: Speed (DPS, position-tracked), SPEED_DPS in deg/s
//   true:  Speed (eRPM, VESC speed loop), SPEED_ERPM in electrical RPM
//
// Type "yes" in the Serial Monitor (115200 baud, newline) to start. Any key
// while running stops the motor. If the motor stops answering, the sketch
// stops sending; note that the controller keeps running a speed command
// when commands stop, so keep an emergency stop on the motor power.
#include <OpenRobotMotor.h>

using namespace openrobot;

const uint8_t CAN_CS_PIN = 9;     // Seeed CAN-BUS Shield V2: 9, V1.x: 10
const uint8_t MOTOR_ID = 1;
const bool USE_ERPM = false;
const float SPEED_DPS = 360.0f;   // motor side
const float SPEED_ERPM = 3000.0f; // electrical RPM = RPM x pole pairs

Mcp2515Transport can(CAN_CS_PIN);
MotorBus bus(can);
Motor motor(bus, MOTOR_ID);

bool waitForYes() {
    Serial.println(F("This sketch turns the motor both ways. Make sure the shaft is free."));
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

// Runs one speed for `ms`. false if stopped by a key or by missed replies.
bool run(float value, uint32_t ms) {
    uint32_t start = millis();
    int missed = 0;
    while (millis() - start < ms) {
        if (Serial.available()) return false;
        Status s;
        Result r = USE_ERPM ? motor.speed_erpm(value, &s) : motor.speed_dps(value, &s);
        if (r == OK) {
            missed = 0;
            Serial.print(F("target ")); Serial.print(value, 0);
            Serial.print(USE_ERPM ? F(" eRPM") : F(" deg/s"));
            Serial.print(F("  actual ")); Serial.print(s.speed_dps);
            Serial.print(F(" deg/s  current ")); Serial.print(s.current_a, 2); Serial.println(F(" A"));
        } else if (++missed >= 3) {
            Serial.println(F("no reply 3 times in a row"));
            return false;
        }
        delay(10);
    }
    return true;
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
    const float v = USE_ERPM ? SPEED_ERPM : SPEED_DPS;
    if (run(v, 2000) && run(-v, 2000)) {
        if (USE_ERPM) motor.speed_erpm(0);   // speed 0, no position hold
        else motor.stop();                   // DPS: hold where it is
        delay(500);
    }
    motor.off();
    Serial.println(F("released"));
}

void loop() {}
