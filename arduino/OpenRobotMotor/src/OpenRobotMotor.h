// OpenRobot motor controllers over CAN, for Arduino.
//
// The protocol core (openrobot_motor/protocol.hpp, bus.hpp) is the same code
// as the C++ library. This header adds a Transport for MCP2515 CAN shields
// and modules (autowp-mcp2515 library). For other CAN hardware, implement
// openrobot::Transport the same way.
//
// SPDX-License-Identifier: MIT
#ifndef OPENROBOT_MOTOR_ARDUINO_H
#define OPENROBOT_MOTOR_ARDUINO_H

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <new>

#include "openrobot_motor/protocol.hpp"
#include "openrobot_motor/bus.hpp"

namespace openrobot {

// The MCP2515 driver object is created in begin(), not at construction: its
// constructor sets up SPI, and on some boards (SAMD, e.g. Arduino Zero) a
// global object is constructed before the core's init(), which then undoes
// that setup. So a global Mcp2515Transport is safe on every board.
class Mcp2515Transport : public Transport {
public:
    explicit Mcp2515Transport(uint8_t cs_pin, uint32_t spi_clock = 10000000UL, SPIClass *spi = &SPI)
        : cs_(cs_pin), clock_(spi_clock), spi_(spi), mcp_(0) {}

    // Call from setup(). crystal: MCP_16MHZ on most shields (Seeed CAN-BUS
    // Shield), MCP_8MHZ on many small modules; 1 Mbit/s needs 16 MHz.
    bool begin(CAN_SPEED speed = CAN_1000KBPS, CAN_CLOCK crystal = MCP_16MHZ) {
        if (!mcp_) {
            // The driver starts SPI itself only when given no SPI object.
            spi_->begin();
            mcp_ = new (storage_) MCP2515(cs_, clock_, spi_);
        }
        if (mcp_->reset() != MCP2515::ERROR_OK) return false;
        if (mcp_->setBitrate(speed, crystal) != MCP2515::ERROR_OK) return false;
        return mcp_->setNormalMode() == MCP2515::ERROR_OK;
    }

    bool send(uint16_t id, const uint8_t data[8]) override {
        can_frame f;
        f.can_id = id;                               // standard frame
        f.can_dlc = 8;
        for (int i = 0; i < 8; ++i) f.data[i] = data[i];
        return mcp_ && mcp_->sendMessage(&f) == MCP2515::ERROR_OK;
    }

    bool recv(uint16_t &id, uint8_t data[8], uint32_t timeout_ms) override {
        if (!mcp_) return false;
        const uint32_t start = millis();
        for (;;) {
            can_frame f;
            if (mcp_->readMessage(&f) == MCP2515::ERROR_OK) {
                if ((f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) || f.can_dlc != 8) continue;
                id = (uint16_t)(f.can_id & CAN_SFF_MASK);
                for (int i = 0; i < 8; ++i) data[i] = f.data[i];
                return true;
            }
            if (millis() - start >= timeout_ms) return false;
        }
    }

    uint32_t now_ms() override { return millis(); }

private:
    uint8_t cs_;
    uint32_t clock_;
    SPIClass *spi_;
    MCP2515 *mcp_;
    alignas(MCP2515) uint8_t storage_[sizeof(MCP2515)];
};

}  // namespace openrobot

#endif  // OPENROBOT_MOTOR_ARDUINO_H
