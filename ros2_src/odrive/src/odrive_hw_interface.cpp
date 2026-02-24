#include "odrive/odrive_hw_interface.hpp"
#include "odrive/odrive_can_utils.hpp"
#include "odrive/Constants.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <chrono>

using namespace ODriveCommands;

namespace odrive {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn ODriveSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Hardware-level param: CAN interface
  if (info.hardware_parameters.count("can_interface")) {
    can_interface_ = info.hardware_parameters.at("can_interface");
  } else {
    can_interface_ = "can0";
  }

  // Per-joint params
  joints_.resize(info.joints.size());
  for (size_t i = 0; i < info.joints.size(); ++i) {
    joints_[i].name = info.joints[i].name;

    if (info.joints[i].parameters.count("node_id")) {
      joints_[i].node_id = static_cast<uint8_t>(
        std::stoi(info.joints[i].parameters.at("node_id")));
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveSystemInterface"),
        "Joint '%s' is missing required parameter 'node_id'", joints_[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (info.joints[i].parameters.count("cmd_vel_scale")) {
      joints_[i].cmd_vel_scale = std::stod(info.joints[i].parameters.at("cmd_vel_scale"));
    }

    RCLCPP_INFO(rclcpp::get_logger("ODriveSystemInterface"),
      "Joint '%s' -> CAN node_id %d on %s",
      joints_[i].name.c_str(), joints_[i].node_id, can_interface_.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveSystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Open the CAN socket using shared utility
  sock_ = ODriveCAN::open_can_socket(can_interface_);
  if (sock_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveSystemInterface"),
      "\n"
      "========================================================\n"
      "  CAN INTERFACE ERROR\n"
      "  Could not open '%s': %s\n"
      "\n"
      "  Make sure the interface exists and is up:\n"
      "    sudo ip link set %s up type can bitrate 500000\n"
      "========================================================",
      can_interface_.c_str(), strerror(errno), can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("ODriveSystemInterface"),
    "CAN socket opened on '%s'", can_interface_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  int connected_count = 0;
  int failed_count = 0;

  // Put each axis into CLOSED_LOOP_CONTROL
  for (auto & j : joints_) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveSystemInterface"),
      "[%s] Requesting CLOSED_LOOP_CONTROL (node_id %d on %s)...",
      j.name.c_str(), j.node_id, can_interface_.c_str());

    if (!ODriveCAN::send_set_axis_state(sock_, j.node_id, CLOSED_LOOP_CONTROL_STATE)) {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveSystemInterface"),
        "[%s] FAILED to send Set_Axis_State to node_id %d – "
        "check CAN wiring and that the ODrive is powered on",
        j.name.c_str(), j.node_id);
      j.connected = false;
      failed_count++;
      continue;  // try remaining motors
    }

    if (ODriveCAN::wait_for_axis_state(sock_, j.node_id, CLOSED_LOOP_CONTROL_STATE, 5000)) {
      RCLCPP_INFO(rclcpp::get_logger("ODriveSystemInterface"),
        "[%s] node_id %d => CLOSED_LOOP_CONTROL  [OK]", j.name.c_str(), j.node_id);
      j.connected = true;
      j.last_feedback_time = std::chrono::steady_clock::now();
      connected_count++;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("ODriveSystemInterface"),
        "[%s] node_id %d did NOT confirm CLOSED_LOOP_CONTROL within 5 s – "
        "motor may be in an error state or not responding",
        j.name.c_str(), j.node_id);
      j.connected = false;
      failed_count++;
    }

    // Zero out the command
    j.cmd_velocity = 0.0;
    j.write_fail_count = 0;
    j.stale_warned = false;
  }

  // Print a summary
  RCLCPP_INFO(rclcpp::get_logger("ODriveSystemInterface"),
    "\n"
    "========================================================\n"
    "  ODrive Activation Summary  (%s)\n"
    "  Motors connected: %d / %zu\n"
    "  Motors failed:    %d / %zu\n"
    "========================================================",
    can_interface_.c_str(),
    connected_count, joints_.size(),
    failed_count, joints_.size());

  if (connected_count == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveSystemInterface"),
      "No motors responded – aborting activation. "
      "Check CAN bus wiring, ODrive power, and node IDs.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Start the CAN reader thread
  running_.store(true);
  reader_thread_ = std::thread([this]() { this->can_read_loop(); });

  RCLCPP_INFO(rclcpp::get_logger("ODriveSystemInterface"), "Activated – CAN reader running");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop all motors (send zero velocity)
  for (auto & j : joints_) {
    ODriveCAN::send_set_input_vel(sock_, j.node_id, 0.0f, 0.0f);
  }

  // Stop reader thread
  running_.store(false);
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }

  RCLCPP_INFO(rclcpp::get_logger("ODriveSystemInterface"), "Deactivated – motors stopped");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveSystemInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (sock_ >= 0) {
    close(sock_);
    sock_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interface export
// ---------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface>
ODriveSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & j : joints_) {
    interfaces.emplace_back(
      j.name, hardware_interface::HW_IF_POSITION, &j.hw_position);
    interfaces.emplace_back(
      j.name, hardware_interface::HW_IF_VELOCITY, &j.hw_velocity);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ODriveSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & j : joints_) {
    interfaces.emplace_back(
      j.name, hardware_interface::HW_IF_VELOCITY, &j.cmd_velocity);
  }
  return interfaces;
}

// ---------------------------------------------------------------------------
// Read / Write (called every control cycle by ros2_control_node)
// ---------------------------------------------------------------------------

hardware_interface::return_type ODriveSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  auto now = std::chrono::steady_clock::now();
  for (auto & j : joints_) {
    if (!j.connected) continue;

    auto ms_since = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - j.last_feedback_time).count();

    if (ms_since > kFeedbackTimeoutMs && !j.stale_warned) {
      RCLCPP_WARN(rclcpp::get_logger("ODriveSystemInterface"),
        "[%s] No encoder feedback from node_id %d for %ld ms – "
        "motor may have disconnected or entered an error state",
        j.name.c_str(), j.node_id, ms_since);
      j.stale_warned = true;
    } else if (ms_since <= kFeedbackTimeoutMs && j.stale_warned) {
      RCLCPP_INFO(rclcpp::get_logger("ODriveSystemInterface"),
        "[%s] Encoder feedback from node_id %d recovered",
        j.name.c_str(), j.node_id);
      j.stale_warned = false;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ODriveSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & j : joints_) {
    // ros2_control gives us rad/s; convert to turns/s
    float vel_turns = ODriveCAN::rad_per_sec_to_turns_per_sec(
      static_cast<float>(j.cmd_velocity * j.cmd_vel_scale));
    
    if (!ODriveCAN::send_set_input_vel(sock_, j.node_id, vel_turns, 0.0f)) {
      j.write_fail_count++;
      if (j.write_fail_count == 1 || j.write_fail_count % 1000 == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ODriveSystemInterface"),
          "[%s] CAN write FAILED for node_id %d (%u consecutive failures) – "
          "check CAN bus connection",
          j.name.c_str(), j.node_id, j.write_fail_count);
      }
    } else {
      if (j.write_fail_count > 0) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveSystemInterface"),
          "[%s] CAN write to node_id %d recovered after %u failures",
          j.name.c_str(), j.node_id, j.write_fail_count);
      }
      j.write_fail_count = 0;
    }
  }
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// CAN frame processing (background thread)
// ---------------------------------------------------------------------------

void ODriveSystemInterface::can_read_loop()
{
  while (running_.load()) {
    struct can_frame frame{};
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sock_, &readfds);
    struct timeval tv{0, 200000};  // 200 ms
    int rv = select(sock_ + 1, &readfds, nullptr, nullptr, &tv);
    if (rv > 0 && FD_ISSET(sock_, &readfds)) {
      ssize_t n = ::read(sock_, &frame, sizeof(frame));
      if (n > 0) process_can_frame(frame);
    } else if (rv < 0 && errno != EINTR) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

void ODriveSystemInterface::process_can_frame(const struct can_frame & frame)
{
  uint32_t id = frame.can_id & CAN_SFF_MASK;
  uint8_t  frame_node = static_cast<uint8_t>(id >> 5);
  uint8_t  cmd_id     = static_cast<uint8_t>(id & 0x1F);

  if (cmd_id != ENCODER_ESTIMATES || frame.can_dlc < 8) return;

  // Find matching joint
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (auto & j : joints_) {
    if (j.node_id == frame_node) {
      float pos_turns = 0.0f, vel_turns = 0.0f;
      std::memcpy(&pos_turns, &frame.data[0], sizeof(float));
      std::memcpy(&vel_turns, &frame.data[4], sizeof(float));
      j.hw_position = ODriveCAN::turns_to_radians(pos_turns);
      j.hw_velocity = ODriveCAN::turns_per_sec_to_rad_per_sec(vel_turns);
      j.last_feedback_time = std::chrono::steady_clock::now();
      if (!j.connected) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveSystemInterface"),
          "[%s] First encoder feedback received from node_id %d",
          j.name.c_str(), j.node_id);
        j.connected = true;
      }
      return;
    }
  }
}

}  // namespace odrive

// Register the plugin with pluginlib so ros2_control can discover it
PLUGINLIB_EXPORT_CLASS(odrive::ODriveSystemInterface, hardware_interface::SystemInterface)
