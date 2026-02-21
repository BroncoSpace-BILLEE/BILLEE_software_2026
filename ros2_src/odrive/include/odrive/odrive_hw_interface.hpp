#ifndef ODRIVE_HW_INTERFACE_HPP
#define ODRIVE_HW_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>
#include <mutex>

struct can_frame;  // forward declaration (from linux/can.h)

namespace odrive {

/**
 * @brief ros2_control SystemInterface for ODrive motor controllers over CAN.
 *
 * Manages multiple joints on a single CAN bus. Each joint maps to one
 * ODrive axis identified by its CAN node_id (set as a per-joint URDF param).
 *
 * URDF example:
 *   <ros2_control name="ODriveSystem" type="system">
 *     <hardware>
 *       <plugin>odrive/ODriveSystemInterface</plugin>
 *       <param name="can_interface">can0</param>
 *     </hardware>
 *     <joint name="joint_wheel_r1">
 *       <param name="node_id">0</param>
 *       <command_interface name="velocity"/>
 *       <state_interface name="velocity"/>
 *       <state_interface name="position"/>
 *     </joint>
 *     ...
 *   </ros2_control>
 */
class ODriveSystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ODriveSystemInterface)

  // Lifecycle callbacks
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // Interface export
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read/write cycle
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ---- CAN helpers (same protocol as odrive.cpp) ----
  int  open_can_socket(const std::string & ifname);
  static uint32_t make_can_id(uint8_t node_id, uint8_t cmd_id);
  bool send_set_axis_state(uint8_t node_id, uint32_t requested_state);
  bool send_set_input_vel(uint8_t node_id, float vel, float torque_ff);
  bool wait_for_axis_state(uint8_t node_id, uint8_t desired_state, int timeout_ms);
  void can_read_loop();
  void process_can_frame(const ::can_frame & frame);

  float meters_per_sec_to_turns_per_sec(float mps) const;
  float turns_to_radians(float turns) const;
  float turns_per_sec_to_rad_per_sec(float tps) const;

  // ---- Per-joint data ----
  struct JointData {
    std::string name;
    uint8_t     node_id = 0;

    // State (updated from CAN reader thread)
    double hw_position = 0.0;   // rad
    double hw_velocity = 0.0;   // rad/s

    // Command (set by ros2_control write())
    double cmd_velocity = 0.0;  // rad/s

    // Scale factor for velocity command
    double cmd_vel_scale = 1.0;

    // Health tracking
    bool   connected = false;                       // true once first heartbeat/encoder msg received
    std::chrono::steady_clock::time_point last_feedback_time{};  // last encoder estimate timestamp
    uint32_t write_fail_count = 0;                  // consecutive CAN write failures
    bool     stale_warned = false;                  // avoid log-spam for stale feedback
  };

  std::vector<JointData> joints_;
  std::mutex state_mutex_;       // protects hw_position / hw_velocity

  // CAN
  std::string can_interface_;
  int         sock_ = -1;
  std::thread reader_thread_;
  std::atomic<bool> running_{false};

  // Physical constants (matching your existing odrive.cpp)
  static constexpr double kWheelRadius = 0.11;  // m

  // Timeout: warn if no encoder feedback for this long
  static constexpr int kFeedbackTimeoutMs = 2000;
};

}  // namespace odrive

#endif  // ODRIVE_HW_INTERFACE_HPP
