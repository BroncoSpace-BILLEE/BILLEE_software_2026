#ifndef ODRIVE_CAN_UTILS_HPP
#define ODRIVE_CAN_UTILS_HPP

#include <cstdint>
#include <string>

struct can_frame;  // forward declaration (from linux/can.h)

namespace odrive {

/**
 * @brief Utility class for ODrive CAN communication.
 * 
 * Provides common CAN socket operations, message encoding/decoding,
 * and protocol utilities for ODrive CANSimple protocol.
 */
class ODriveCAN {
public:
  /**
   * @brief Opens and binds a SocketCAN socket to the specified CAN interface.
   * @param ifname Name of the CAN network interface (e.g., "can0")
   * @return File descriptor for the CAN socket on success, -1 on failure
   */
  static int open_can_socket(const std::string &ifname);

  /**
   * @brief Constructs a CANSimple message ID from node_id and command ID.
   * @details According to ODrive CANSimple protocol, the message ID is formed by shifting
   * node_id left by 5 bits and ORing with the command ID. This ensures a valid 11-bit standard CAN ID.
   * @param node_id The ODrive node ID (0-63)
   * @param cmd_id The command ID (0-31)
   * @return The constructed 11-bit CAN message ID
   */
  static uint32_t make_can_id(uint8_t node_id, uint8_t cmd_id);

  /**
   * @brief Sends a Set_Axis_State command over CAN (cmd 0x07).
   * @details Encodes the requested axis state as a 32-bit little-endian value into a CAN frame
   * and transmits it over the socket. Used to request axis state transitions such as CLOSED_LOOP_CONTROL.
   * @param sock File descriptor for the CAN socket
   * @param node_id Target ODrive node ID
   * @param requested_state The desired axis state (e.g., 8 for CLOSED_LOOP_CONTROL)
   * @return true if the frame was successfully written, false otherwise
   */
  static bool send_set_axis_state(int sock, uint8_t node_id, uint32_t requested_state);

  /**
   * @brief Sends a Set_Input_Vel command over CAN (cmd 0x0d).
   * @details Encodes velocity and optional torque feedforward as IEEE 754 floats in little-endian
   * format and transmits over CAN. Velocity is in turns/s, torque feedforward in Nm.
   * @param sock File descriptor for the CAN socket
   * @param node_id Target ODrive node ID
   * @param vel Desired velocity in turns/s (IEEE 754 float at bytes 0-3)
   * @param torque_ff Torque feedforward in Nm (IEEE 754 float at bytes 4-7)
   * @return true if the frame was successfully written, false otherwise
   */
  static bool send_set_input_vel(int sock, uint8_t node_id, float vel, float torque_ff);

  /**
   * @brief Waits for a heartbeat message indicating the axis has reached a desired state.
   * @details Polls the CAN socket using select() with 100ms timeouts until either the
   * heartbeat message indicates the desired axis state or the timeout expires.
   * @param sock File descriptor for the CAN socket
   * @param node_id Target ODrive node ID
   * @param desired_state The axis state value to wait for (e.g., 8 for CLOSED_LOOP_CONTROL)
   * @param timeout_ms Total timeout in milliseconds
   * @return true if the desired state is confirmed within the timeout, false if timeout expires
   */
  static bool wait_for_axis_state(int sock, uint8_t node_id, uint8_t desired_state, int timeout_ms);

  /**
   * @brief Converts meters per second to turns per second.
   * @param mps Velocity in meters per second
   * @param wheel_radius Wheel radius in meters
   * @return Velocity in turns per second
   */
  static float meters_per_sec_to_turns_per_sec(float mps, float wheel_radius);

  /**
   * @brief Converts turns to radians.
   * @param turns Position in turns
   * @return Position in radians
   */
  static float turns_to_radians(float turns);

  /**
   * @brief Converts turns per second to radians per second.
   * @param tps Velocity in turns per second
   * @return Velocity in radians per second
   */
  static float turns_per_sec_to_rad_per_sec(float tps);

  /**
   * @brief Converts radians per second to turns per second.
   * @param rad_per_sec Velocity in radians per second
   * @return Velocity in turns per second
   */
  static float rad_per_sec_to_turns_per_sec(float rad_per_sec);
};

}  // namespace odrive

#endif  // ODRIVE_CAN_UTILS_HPP
