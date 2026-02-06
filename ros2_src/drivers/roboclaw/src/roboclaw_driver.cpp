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

  const double PI = 3.14159265358979323846;
  const double MAX_VELOCITY_RAD_PER_SEC = 3; // Example max velocity, TODO: replace with actual value

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
    encoder_ticks_per_rev_ = std::stod(info.hardware_parameters.at("encoder_ticks_per_rev"));
    encoder_pulses_per_revolution_ = std::stod(info.hardware_parameters.at("encoder_pulses_per_revolution"));

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
    hw_positions_.resize(motor_names_.size(), std::numeric_limits<double>::quiet_NaN()); //TODO: check whether initiializing to NaN is appropriate for boot up
    hw_velocities_.resize(motor_names_.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocity_commands_.resize(motor_names_.size(), std::numeric_limits<double>::quiet_NaN());
    hw_position_commands_.resize(motor_names_.size(), std::numeric_limits<double>::quiet_NaN());

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

  hardware_interface::CallbackReturn RoboClawDriver::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("RoboClawDriver"), "Activating RoboClaw driver");

    for (size_t i = 0; i < motor_names_.size(); ++i) {
      hw_velocity_commands_[i] = hw_velocities_[i] = 0.0;

      if(use_mock_hardware_)
        hw_position_commands_[i] = hw_positions_[i] = 0.0;
      else{
        auto encoders = roboclaw_->get_encoders(address_);
        hw_positions_[0] = static_cast<double>(encoders.first);
        hw_positions_[1] = static_cast<double>(encoders.second);
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RoboClawDriver::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("RoboClawDriver"), "Deactivating RoboClaw driver");

    for (size_t i = 0; i < motor_names_.size(); ++i) {
      hw_velocity_commands_[i] =  hw_velocities_[i] = 0.0;
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
        motor_names_[i], hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);

      command_interfaces.emplace_back(
        motor_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]);
    }

    return command_interfaces;
  }

  hardware_interface::return_type RoboClawDriver::read(const rclcpp::Time &,  const rclcpp::Duration & period)
  {

    if(use_mock_hardware_){

      for(size_t i = 0; i < motor_names_.size(); i++){
        if(std::isnan(hw_velocity_commands_[i])){
          RCLCPP_WARN(rclcpp::get_logger("RoboClawDriver"), "velocity is flagged as NaN for motors %s", motor_names_[i].c_str());
          hw_velocity_commands_[i] = 0.0;
        }
        
        else{
          //simple P controller in sim...
          double dt = period.seconds();
          double ds = hw_position_commands_[i] - hw_positions_[i];

          hw_velocities_[i] = ds/dt;
          hw_positions_[i] += ds;
        }
      }
    }
    else{
      try {
        // Read encoders
        auto encoders = roboclaw_->get_encoders(address_);
        hw_positions_[0] = encoder_ticks_to_rad(static_cast<double>(encoders.first));
        hw_positions_[1] = encoder_ticks_to_rad(static_cast<double>(encoders.second)) ;

        // Read velocities
        auto velocities = roboclaw_->get_velocity(address_);
        hw_velocities_[0] = static_cast<double>(pulses_per_sec_to_rad_per_sec(velocities.first)); //TODO: check if encoder velocity is in units of qpps
        hw_velocities_[1] = static_cast<double>(pulses_per_sec_to_rad_per_sec(velocities.second));

      } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("RoboClawDriver"), "Failed to read from RoboClaw: %s", e.what());
        return hardware_interface::return_type::ERROR;
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type RoboClawDriver::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    if(!use_mock_hardware_){
      try {
        int left_speed = static_cast<int>(
          rad_per_sec_to_pulses_per_sec(
            hw_velocity_commands_[0] / MAX_VELOCITY_RAD_PER_SEC)
        );
        int right_speed = static_cast<int>(
          rad_per_sec_to_pulses_per_sec(
            hw_velocity_commands_[1] / MAX_VELOCITY_RAD_PER_SEC)
        );

        roboclaw_->set_velocity(address_, std::make_pair(left_speed, right_speed));

      } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("RoboClawDriver"), "Failed to write to RoboClaw: %s", e.what());
        return hardware_interface::return_type::ERROR;
      }
    }

    return hardware_interface::return_type::OK;
  }

  double RoboClawDriver::encoder_ticks_to_rad(double ticks) const{
    return (ticks / encoder_ticks_per_rev_) * 2.0 * PI;
  }

  double RoboClawDriver::rad_per_sec_to_pulses_per_sec(double vel_rad_sec) const{
    double revs_per_sec = vel_rad_sec / (2.0 * PI);
    return 4 * revs_per_sec * encoder_pulses_per_revolution_;
  }

  double RoboClawDriver::pulses_per_sec_to_rad_per_sec(double pulse_per_sec) const{
    double revs_per_sec = pulse_per_sec / (4 * encoder_pulses_per_revolution_);
    return revs_per_sec * 2.0 * PI;
  }


} 

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(roboclaw_driver::RoboClawDriver, hardware_interface::SystemInterface)