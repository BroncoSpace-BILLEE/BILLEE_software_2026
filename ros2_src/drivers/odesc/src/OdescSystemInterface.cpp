#include "odesc/OdescSystemInterface.hpp"
#include <sstream>
#include <cmath>
#include "odesc/CanUtils.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace odesc {

hardware_interface::CallbackReturn OdescSystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info.joints.size() != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("odesc"), "Expected exactly one joint, got %zu", info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_name_ = info.joints[0].name;

  if (info.hardware_parameters.count("can_interface")) {
    can_interface_ = info.hardware_parameters.at("can_interface");
  }

  if (info.hardware_parameters.count("node_id")) {
    node_id_ = static_cast<uint8_t>(std::stoi(info.hardware_parameters.at("node_id")));
  }

  if (info.hardware_parameters.count("joint_name")) {
    joint_name_ = info.hardware_parameters.at("joint_name");
  }

  if (info.joints[0].command_interfaces.size() != 1 ||
      info.joints[0].command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
    RCLCPP_ERROR(rclcpp::get_logger("odesc"), "Joint must have a single velocity command interface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info.joints[0].state_interfaces.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("odesc"), "Joint must have position and velocity state interfaces");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OdescSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION, &position_);
  state_interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_VELOCITY, &velocity_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OdescSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_VELOCITY, &command_velocity_);
  return command_interfaces;
}

hardware_interface::CallbackReturn OdescSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  socket_fd_ = can::open_can_socket(can_interface_, rclcpp::get_logger("odesc"));
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("odesc"), "Failed to open CAN interface %s", can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!can::send_set_axis_state(socket_fd_, node_id_, can::AXIS_STATE_CLOSED_LOOP_CONTROL)) {
    RCLCPP_ERROR(rclcpp::get_logger("odesc"), "Failed to send Set_Axis_State");
    can::close_can_socket(socket_fd_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdescSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  can::close_can_socket(socket_fd_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OdescSystemInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!active_ || socket_fd_ < 0) {
    return hardware_interface::return_type::OK;
  }

  struct can_frame frame{};
  // Non-blocking read of all pending frames
  while (can::read_can_frame(socket_fd_, frame, 0, rclcpp::get_logger("odesc"))) {
    float pos = 0.0f;
    float vel = 0.0f;
    if (can::parse_encoder_estimates(frame, node_id_, pos, vel)) {
      position_ = static_cast<double>(pos);
      velocity_ = static_cast<double>(vel);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OdescSystemInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!active_ || socket_fd_ < 0) {
    return hardware_interface::return_type::OK;
  }

  float vel = static_cast<float>(command_velocity_) / (2 * M_PI); // convert from rad/s to turns/s
  if (!can::send_set_input_vel(socket_fd_, node_id_, vel, 0.0f)) {
    RCLCPP_WARN(rclcpp::get_logger("odesc"), "Failed to send Set_Input_Vel");
  }

  return hardware_interface::return_type::OK;
}

}  // namespace odesc

PLUGINLIB_EXPORT_CLASS(odesc::OdescSystemInterface, hardware_interface::SystemInterface)
