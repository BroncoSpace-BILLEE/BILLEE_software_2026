#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <linux/can.h>

namespace odesc {
namespace can {

/**
 * @brief CANSimple command IDs used by ODrive-compatible devices.
 */
constexpr uint8_t CMD_HEARTBEAT = 0x01;
constexpr uint8_t CMD_SET_AXIS_STATE = 0x07;
constexpr uint8_t CMD_GET_ENCODER_ESTIMATES = 0x09;
constexpr uint8_t CMD_SET_INPUT_VEL = 0x0d;

/**
 * @brief ODrive axis state values.
 */
constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;

/**
 * @brief Opens and binds a SocketCAN socket to the specified CAN interface.
 * @param ifname Name of the CAN network interface (e.g., "can0")
 * @param logger ROS2 logger for reporting failures
 * @return File descriptor for the CAN socket on success, -1 on failure
 */
int open_can_socket(const std::string &ifname, const rclcpp::Logger &logger);

/**
 * @brief Opens and binds a SocketCAN socket to the specified CAN interface.
 * @details Uses a default logger named "odesc" for diagnostics.
 * @param ifname Name of the CAN network interface (e.g., "can0")
 * @return File descriptor for the CAN socket on success, -1 on failure
 */
inline int open_can_socket(const std::string &ifname)
{
	return open_can_socket(ifname, rclcpp::get_logger("odesc"));
}

/**
 * @brief Closes a SocketCAN socket and resets the file descriptor to -1.
 * @param socket_fd Socket file descriptor to close
 */
void close_can_socket(int &socket_fd);

/**
 * @brief Constructs a CANSimple message ID from node_id and command ID.
 * @details According to ODrive CANSimple protocol, the message ID is formed by shifting
 * node_id left by 5 bits and ORing with the command ID. This ensures a valid 11-bit standard CAN ID.
 * @param node_id The ODrive node ID (0-63)
 * @param cmd_id The command ID (0-31)
 * @return The constructed 11-bit CAN message ID
 */
uint32_t make_can_id(uint8_t node_id, uint8_t cmd_id);

/**
 * @brief Sends a Set_Axis_State command over CAN (cmd 0x07).
 * @details Encodes the requested axis state as a 32-bit little-endian value into a CAN frame
 * and transmits it over the socket. Used to request axis state transitions such as CLOSED_LOOP_CONTROL.
 * @param socket_fd File descriptor for the CAN socket
 * @param node_id Target ODrive node ID
 * @param requested_state The desired axis state (e.g., 8 for CLOSED_LOOP_CONTROL)
 * @return true if the frame was successfully written, false otherwise
 */
bool send_set_axis_state(int socket_fd, uint8_t node_id, uint32_t requested_state);

/**
 * @brief Sends a Set_Input_Vel command over CAN (cmd 0x0d).
 * @details Encodes velocity and optional torque feedforward as IEEE 754 floats in little-endian
 * format and transmits over CAN. Velocity is in turns/s, torque feedforward in Nm.
 * @param socket_fd File descriptor for the CAN socket
 * @param node_id Target ODrive node ID
 * @param velocity Desired velocity in turns/s (IEEE 754 float at bytes 0-3)
 * @param torque_ff Torque feedforward in Nm (IEEE 754 float at bytes 4-7)
 * @return true if the frame was successfully written, false otherwise
 */
bool send_set_input_vel(int socket_fd, uint8_t node_id, float velocity, float torque_ff);

/**
 * @brief Reads a CAN frame with a timeout.
 * @details Uses select() to wait for data on the socket. A timeout of 0 makes this call non-blocking.
 * @param socket_fd File descriptor for the CAN socket
 * @param frame Output CAN frame
 * @param timeout_ms Timeout in milliseconds
 * @param logger ROS2 logger for reporting failures
 * @return true if a frame was read, false if timeout or error occurred
 */
bool read_can_frame(int socket_fd, struct can_frame &frame, int timeout_ms, const rclcpp::Logger &logger);

/**
 * @brief Reads a CAN frame with a timeout.
 * @details Uses a default logger named "odesc" for diagnostics.
 * @param socket_fd File descriptor for the CAN socket
 * @param frame Output CAN frame
 * @param timeout_ms Timeout in milliseconds
 * @return true if a frame was read, false if timeout or error occurred
 */
inline bool read_can_frame(int socket_fd, struct can_frame &frame, int timeout_ms)
{
	return read_can_frame(socket_fd, frame, timeout_ms, rclcpp::get_logger("odesc"));
}

/**
 * @brief Parses a Get_Encoder_Estimates CAN frame (cmd 0x09).
 * @details If the frame matches the provided node_id and command ID, extracts position and velocity
 * floats from the payload.
 * @param frame CAN frame to parse
 * @param node_id Expected node ID
 * @param position Output position (turns)
 * @param velocity Output velocity (turns/s)
 * @return true if the frame contained encoder estimates, false otherwise
 */
bool parse_encoder_estimates(const struct can_frame &frame, uint8_t node_id, float &position, float &velocity);

}  // namespace can
}  // namespace odesc
