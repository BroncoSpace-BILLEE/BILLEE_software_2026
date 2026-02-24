#pragma once

#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace odesc {

/**
 * @brief ROS2 Control SystemInterface for ODrive-compatible CAN devices.
 * @details Exposes a single joint with velocity command and position/velocity state,
 * using SocketCAN to send Set_Input_Vel and read Get_Encoder_Estimates frames.
 */
class OdescSystemInterface : public hardware_interface::SystemInterface
{
public:
  /**
   * @brief Initialize the hardware interface from URDF hardware info.
   * @param info hardware_interface::HardwareInfo from controller manager
   * @return CallbackReturn::SUCCESS if initialized, ERROR otherwise
   */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Export position and velocity state interfaces.
   * @return Vector of StateInterface objects
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Export velocity command interfaces.
   * @return Vector of CommandInterface objects
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Activate the hardware interface.
   * @details Opens CAN socket and requests CLOSED_LOOP_CONTROL.
   * @return CallbackReturn::SUCCESS if activation succeeded, ERROR otherwise
   */
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate the hardware interface and close the CAN socket.
   * @return CallbackReturn::SUCCESS
   */
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Read state from hardware.
   * @details Polls CAN socket for encoder estimates and updates joint state.
   * @return return_type::OK on success
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * @brief Write command to hardware.
   * @details Sends Set_Input_Vel message using the current command velocity.
   * @return return_type::OK on success
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::string can_interface_ = "can2";
  uint8_t node_id_ = 0;
  std::string joint_name_ = "axis0";

  int socket_fd_ = -1;
  bool active_ = false;

  double position_ = 0.0;
  double velocity_ = 0.0;
  double command_velocity_ = 0.0;
};

}  // namespace odesc
