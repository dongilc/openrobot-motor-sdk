// Step 3: several motors on one bus, each following its own speed wave.
//
// One loop sends a command to every motor in turn and reads each reply, so
// all motors are updated at the same rate. If any motor stops answering,
// every motor is released. Type "yes" in the Serial Monitor to start; any
// key stops.
#include <OpenRobotMotor.h>
#include <math.h>

using namespace openrobot;

const uint8_t CAN_CS_PIN = 9;     // Seeed CAN-BUS Shield V2: 9, V1.x: 10
const uint8_t MOTOR_IDS[] = {1, 2};
const int N = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);
const float PEAK_DPS = 360.0f;    // motor side
const float HZ = 0.5f;
const uint32_t RUN_MS = 6000;
const uint32_t PERIOD_MS = 10;

Mcp2515Transport can(CAN_CS_PIN);
MotorBus bus(can);
Motor *motors[N];

bool waitForYes() {
    Serial.println(F("This sketch turns every listed motor both ways. Make sure the shafts are free."));
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

void releaseAll() {
    for (int k = 0; k < N; ++k) motors[k]->off();
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    alignas(Motor) static uint8_t storage[N][sizeof(Motor)];   // Motor objects are built here once
    for (int k = 0; k < N; ++k) motors[k] = new (storage[k]) Motor(bus, MOTOR_IDS[k]);
    if (!can.begin(CAN_1000KBPS, MCP_16MHZ)) {
        Serial.println(F("MCP2515 not found: check CAN_CS_PIN and the shield."));
        return;
    }
    if (!waitForYes()) {
        Serial.println(F("cancelled"));
        return;
    }
    uint32_t t0 = millis(), next = t0, cycles = 0, last_print = 0;
    int missed[N] = {0};
    bool ok = true;
    while (ok && millis() - t0 < RUN_MS && !Serial.available()) {
        float t = (millis() - t0) * 1e-3f;
        Status s[N] = {};
        for (int k = 0; k < N; ++k) {
            float phase = 2.0f * (float)M_PI * k / N;
            float dps = PEAK_DPS * sinf(2.0f * (float)M_PI * HZ * t + phase);
            if (motors[k]->speed_dps(dps, &s[k]) == OK) {
                missed[k] = 0;
            } else if (++missed[k] >= 3) {
                Serial.print(F("motor ")); Serial.print(MOTOR_IDS[k]);
                Serial.println(F(": no reply 3 times in a row"));
                ok = false;
            }
        }
        ++cycles;
        if (millis() - last_print >= 200) {          // printing every cycle would slow the loop
            last_print = millis();
            for (int k = 0; k < N; ++k) {
                Serial.print(MOTOR_IDS[k]); Serial.print(':'); Serial.print(s[k].speed_dps);
                Serial.print(F("  "));
            }
            Serial.println(F("deg/s"));
        }
        next += PERIOD_MS;
        while ((int32_t)(millis() - next) < 0) {}
    }
    releaseAll();
    Serial.print(F("released, ")); Serial.print(cycles); Serial.println(F(" cycles"));
}

void loop() {}
