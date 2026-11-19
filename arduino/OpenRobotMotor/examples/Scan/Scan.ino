// Step 0: find motors on the bus and print their state. Nothing moves.
//
// Hardware: any Arduino + MCP2515 CAN shield or module, 16 MHz crystal.
// Open the Serial Monitor at 115200 baud; send any key to scan again.
#include <OpenRobotMotor.h>

using namespace openrobot;

const uint8_t CAN_CS_PIN = 9;     // Seeed CAN-BUS Shield V2: 9, V1.x: 10

Mcp2515Transport can(CAN_CS_PIN);
MotorBus bus(can);

bool can_ok = false;

void scan() {
    if (!can_ok) {
        Serial.println(F("MCP2515 not found: check CAN_CS_PIN and the shield."));
        return;
    }
    uint8_t ids[32];
    int n = bus.scan(ids, 32);
    if (n == 0) {
        Serial.println(F("No motor answered. Check power, wiring, termination and bit rate."));
        return;
    }
    for (int k = 0; k < n; ++k) {
        Motor m(bus, ids[k], /*release_on_exit=*/false);   // read only
        Status s;
        Phases p;
        double deg = 0;
        Serial.print(F("\nMotor "));
        Serial.println(ids[k]);
        if (m.status(s) != OK || m.phases(p) != OK || m.multi_turn_deg(deg) != OK) {
            Serial.println(F("  no reply"));
            continue;
        }
        Serial.print(F("  mode         ")); Serial.println(control_mode_name(p.control_mode));
        Serial.print(F("  temperature  ")); Serial.print(s.temperature_c); Serial.println(F(" C"));
        Serial.print(F("  current      ")); Serial.print(s.current_a, 2); Serial.println(F(" A"));
        Serial.print(F("  speed        ")); Serial.print(s.speed_dps); Serial.println(F(" deg/s (motor)"));
        Serial.print(F("  angle        ")); Serial.print(deg, 2); Serial.println(F(" deg (motor, multi-turn)"));
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    can_ok = can.begin(CAN_1000KBPS, MCP_16MHZ);
    scan();
}

void loop() {
    if (Serial.available()) {
        while (Serial.available()) Serial.read();
        scan();
    }
}
