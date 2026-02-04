#include "roboclaw_driver.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace roboclaw_driver
{

hardware_interface::CallbackReturn RoboClawDriver::on_init(const hardware_interface::HardwareInfo & info)
{
  // Call base
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from hardware info
  port_ = info.hardware_parameters.at("port");
  baudrate_ = std::stoi(info.hardware_parameters.at("baudrate"));
  address_ = std::stoi(info.hardware_parameters.at("address"));

  std::string mock_param = (info.hardware_parameters.at("use_mock_hardware"));
  use_mock_hardware_ = mock_param == "1" ? true : false;

  // Get joint names
  for (const auto & motor : info.joints) {
    motor_names_.push_back(motor.name);
  }

  if (motor_names_.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("RoboClawDriver"), "Expected 2 motors, got %zu", motor_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state and command vectors
  hw_positions_.resize(motor_names_.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(motor_names_.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(motor_names_.size(), std::numeric_limits<double>::quiet_NaN());

  // Initialize RoboClaw driver
  try {

    if(!use_mock_hardware_) {
      roboclaw_ = std::make_unique<libroboclaw::driver>(port_, baudrate_);
    }
    else
      RCLCPP_WARN(rclcpp::get_logger("RoboClawDriver"), "Using mock hardware for RoboClaw");

    RCLCPP_INFO(rclcpp::get_logger("RoboClawDriver"), "Connected to RoboClaw on %s", port_.c_str());

    // Reset encoders

    if(!use_mock_hardware_)
      roboclaw_->reset_encoders(address_);

    RCLCPP_INFO(rclcpp::get_logger("RoboClawDriver"), "Reset encoders");

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("RoboClawDriver"), "Failed to initialize RoboClaw: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoboClawDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < motor_names_.size(); ++i) {
    state_interfaces.emplace_back(
      motor_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      motor_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoboClawDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < motor_names_.size(); ++i) {
    command_interfaces.emplace_back(
      motor_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type RoboClawDriver::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  if(use_mock_hardware_){
    // Simulate some data for testing
    hw_positions_[0] += hw_commands_[0] * 0.1;
    hw_positions_[1] += hw_commands_[1] * 0.1;
    hw_velocities_[0] = hw_commands_[0];
    hw_velocities_[1] = hw_commands_[1];
  }
  else{
    try {
      // Read encoders
      auto encoders = roboclaw_->get_encoders(address_);
      hw_positions_[0] = static_cast<double>(encoders.first);
      hw_positions_[1] = static_cast<double>(encoders.second);

      // Read velocities
      auto velocities = roboclaw_->get_velocity(address_);
      hw_velocities_[0] = static_cast<double>(velocities.first);
      hw_velocities_[1] = static_cast<double>(velocities.second);

    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("RoboClawDriver"), "Failed to read from RoboClaw: %s", e.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboClawDriver::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if(!use_mock_hardware_){
    try {
      // Convert commands to int (assuming commands are in appropriate units)
      int left_speed = static_cast<int>(hw_commands_[0]);
      int right_speed = static_cast<int>(hw_commands_[1]);

      roboclaw_->set_velocity(address_, std::make_pair(left_speed, right_speed));

    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("RoboClawDriver"), "Failed to write to RoboClaw: %s", e.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

} 

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(roboclaw_driver::RoboClawDriver, hardware_interface::SystemInterface)