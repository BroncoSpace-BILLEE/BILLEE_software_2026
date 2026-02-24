#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cstdint>


const int BAUD_RATE = 500000;

namespace ODriveCommands {
    const uint8_t ENCODER_ESTIMATES = 0x09;
    const uint8_t SET_INPUT_VEL = 0x0d;
    const uint8_t AXIS_STATE_CMD_ID = 0x07;
    const uint8_t CLOSED_LOOP_CONTROL_STATE = 0x08;
    const uint8_t HEARTBEAT_CMD_ID = 0x01;
}

#endif // CONSTANTS_HPP