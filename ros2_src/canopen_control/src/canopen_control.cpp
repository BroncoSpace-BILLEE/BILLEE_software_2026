
#include <memory>
#include <vector>
#include <cstring>
#include <chrono>
#include <thread>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <csignal>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using std::placeholders::_1;

class JoyToPDONode : public rclcpp::Node {
public:
  JoyToPDONode()
  : Node("joy_to_pdo")
  {
    // Parameters
    this->declare_parameter<std::string>("can_interface", "can0");
    this->declare_parameter<int>("canopen_node_id", 1);
    this->declare_parameter<int>("max_velocity", 1000);
    this->declare_parameter<int>("joy_axis", 1);
    this->declare_parameter<std::string>("operating_mode", "position");  // "position" or "velocity"
    this->declare_parameter<int>("max_position", 100000);  // max position in pulses
    this->declare_parameter<int>("profile_velocity", 10000);  // velocity used during position moves (pulses/s)
    this->declare_parameter<int>("profile_acceleration", 1000000);  // pulses/s²
    this->declare_parameter<int>("profile_deceleration", 1000000);  // pulses/s²
    
    can_interface_ = this->get_parameter("can_interface").as_string();
    node_id_ = static_cast<uint8_t>(this->get_parameter("canopen_node_id").as_int());
    max_velocity_ = this->get_parameter("max_velocity").as_int();
    joy_axis_ = this->get_parameter("joy_axis").as_int();
    operating_mode_ = this->get_parameter("operating_mode").as_string();
    max_position_ = this->get_parameter("max_position").as_int();
    profile_velocity_ = this->get_parameter("profile_velocity").as_int();
    profile_acceleration_ = this->get_parameter("profile_acceleration").as_int();
    profile_deceleration_ = this->get_parameter("profile_deceleration").as_int();

    // Validate operating mode
    if (operating_mode_ != "position" && operating_mode_ != "velocity") {
      RCLCPP_ERROR(this->get_logger(), "Invalid operating_mode '%s'. Must be 'position' or 'velocity'.",
                   operating_mode_.c_str());
      throw std::runtime_error("Invalid operating_mode parameter");
    }

    // Initialize socketCAN
    if (!init_can_socket()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN socket on interface %s", 
                   can_interface_.c_str());
      throw std::runtime_error("CAN socket initialization failed");
    }

    // Subscribe to joystick topic
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyToPDONode::joy_callback, this, _1));

    // Run the full motor initialization sequence
    initialize_motor();
    
    RCLCPP_INFO(this->get_logger(), 
      "JoyToPDONode initialized on %s for node %u, mode=%s",
      can_interface_.c_str(), node_id_, operating_mode_.c_str());
  }

  ~JoyToPDONode()
  {
    shutdown_motor();
  }

  /**
   * @brief Gracefully stop the motor and disable the drive.
   *
   * 1. Command zero velocity / halt
   * 2. Disable operation via CiA 402 controlword
   * 3. Send NMT Stop to the node
   * 4. Close the CAN socket
   *
   * Safe to call multiple times — subsequent calls are no-ops.
   */
  void shutdown_motor()
  {
    if (shutdown_complete_) {
      return;
    }
    shutdown_complete_ = true;

    if (can_socket_ < 0) {
      return;
    }

    const auto delay = std::chrono::milliseconds(50);

    RCLCPP_INFO(this->get_logger(), "=== Motor Shutdown Start (node %u) ===", node_id_);

    // Step 1: Command zero velocity / position hold
    if (operating_mode_ == "velocity") {
      // Set Target Velocity (0x60FF:00) to 0
      send_sdo_write(0x60FF, 0x00, 0);
      std::this_thread::sleep_for(delay);
    }
    // In position mode the motor holds position, so no extra command needed.

    // Step 2: Controlword — Disable Voltage (transition 9 in CiA 402)
    // Writing 0x0000 to controlword forces "Switch On Disabled" state
    send_sdo_write_2byte(0x6040, 0x00, 0x0000);
    std::this_thread::sleep_for(delay);

    // Step 3: NMT Stop
    send_nmt_command(0x02);  // 0x02 = Stop
    std::this_thread::sleep_for(delay);

    RCLCPP_INFO(this->get_logger(), "=== Motor Shutdown Complete ===");

    // Step 4: Close socket
    if (can_socket_ >= 0) {
      close(can_socket_);
      can_socket_ = -1;
    }
  }

private:
  bool init_can_socket()
  {
    // Create a raw CAN socket
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket: %s", strerror(errno));
      return false;
    }

    // Get the CAN interface index
    struct ifreq ifr;
    strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get interface index for %s: %s", 
                   can_interface_.c_str(), strerror(errno));
      close(can_socket_);
      return false;
    }

    // Bind the socket to the CAN interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket to %s: %s", 
                   can_interface_.c_str(), strerror(errno));
      close(can_socket_);
      return false;
    }

    // Set a CAN filter that blocks all incoming frames.
    // We only write to the bus — unread incoming frames (SDO responses, heartbeats)
    // will fill the kernel receive buffer and eventually cause write() to fail with ENOBUFS.
    struct can_filter rfilter;
    rfilter.can_id   = 0xFFFFFFFF;  // impossible ID
    rfilter.can_mask = CAN_EFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
    if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to set CAN filter (non-fatal): %s", strerror(errno));
    }

    RCLCPP_INFO(this->get_logger(), "Successfully bound to CAN interface %s", can_interface_.c_str());
    return true;
  }

  void send_nmt_command(uint8_t command)
  {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x000;  
    frame.can_dlc = 2;     
    
    frame.data[0] = command;  
    frame.data[1] = node_id_;
    
    ssize_t nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send NMT command: %s", strerror(errno));
    } else {
      const char* cmd_str = "";
      if (command == 0x01) cmd_str = "Start (Operational)";
      else if (command == 0x02) cmd_str = "Stop";
      else if (command == 0x80) cmd_str = "Enter Pre-Operational";
      else if (command == 0x81) cmd_str = "Reset Application";
      else if (command == 0x82) cmd_str = "Reset Communication";
      
      RCLCPP_INFO(this->get_logger(), "Sent NMT command to node %u: %s (0x%02X)",
                  node_id_, cmd_str, command);
    }
  }

  /**
   * @brief Send an SDO expedited write with a specified command specifier byte.
   * @param cmd_byte  SDO command specifier: 0x2F=1-byte, 0x2B=2-byte, 0x23=4-byte
   */
  void send_sdo_write_raw(uint8_t cmd_byte, uint16_t index, uint8_t subindex, uint32_t value)
  {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.can_id = 0x600 + node_id_;
    frame.can_dlc = 8;  
    
    frame.data[0] = cmd_byte;
    
    // Bytes 1-2: this holds the index of the object to write to (see page 7 of the manual)
    frame.data[1] = (index & 0xFF);
    frame.data[2] = ((index >> 8) & 0xFF);
    
    // Byte 3: subindex (see page 7)
    frame.data[3] = subindex & 0xFF;
    
    // Bytes 4-7: data (see page 7)
    frame.data[4] = (value & 0xFF);
    frame.data[5] = ((value >> 8) & 0xFF);
    frame.data[6] = ((value >> 16) & 0xFF);
    frame.data[7] = ((value >> 24) & 0xFF);
    
    // Send the frame
    ssize_t nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame: %s", strerror(errno));
    } else {
      RCLCPP_INFO(this->get_logger(), 
        "Sent SDO write: cmd=0x%02X index=0x%04X subindex=0x%02X value=0x%08X to node %u",
        cmd_byte, index, subindex, value, node_id_);

        logCanFrame(frame); //show what the structure of the frame looks like
    }
  }

  /// SDO write 4 bytes (command specifier 0x23)
  void send_sdo_write(uint16_t index, uint8_t subindex, uint32_t value)
  {
    send_sdo_write_raw(0x23, index, subindex, value);
  }

  /// SDO write 2 bytes (command specifier 0x2B)
  void send_sdo_write_2byte(uint16_t index, uint8_t subindex, uint16_t value)
  {
    send_sdo_write_raw(0x2B, index, subindex, static_cast<uint32_t>(value));
  }

  /// SDO write 1 byte (command specifier 0x2F)
  void send_sdo_write_1byte(uint16_t index, uint8_t subindex, uint8_t value)
  {
    send_sdo_write_raw(0x2F, index, subindex, static_cast<uint32_t>(value));
  }

  /**
   * @brief Full motor initialization sequence for CiA 402.
   *
   * Supports Profile Position Mode (mode 1) and Profile Velocity Mode (mode 3).
   *
   * Step 1: Network reset and initialization (NMT)
   * Step 2: Set operating mode
   * Step 3: Enable motor via CiA 402 state machine
   * Step 4: Configure motion parameters (accel, decel, velocity / position)
   */
  void initialize_motor()
  {
    const auto delay = std::chrono::milliseconds(100);

    const bool is_position = (operating_mode_ == "position");
    const uint8_t cia402_mode = is_position ? 0x01 : 0x03;  // 1 = Profile Position, 3 = Profile Velocity

    RCLCPP_INFO(this->get_logger(), "=== Motor Initialization Start (node %u, mode=%s) ===",
                node_id_, operating_mode_.c_str());

    // ── Step 1: Network Reset and Initialization ──────────────────────
    RCLCPP_INFO(this->get_logger(), "Step 1: Network reset and initialization");

    // Reset the node  (NMT command 0x81)
    send_nmt_command(0x81);
    std::this_thread::sleep_for(delay);

    // Reset communications  (NMT command 0x82)
    send_nmt_command(0x82);
    std::this_thread::sleep_for(delay);

    // Set to Operational  (NMT command 0x01)
    send_nmt_command(0x01);
    std::this_thread::sleep_for(delay);

    // ── Step 2: Set Operating Mode ────────────────────────────────────
    RCLCPP_INFO(this->get_logger(), "Step 2: Set operating mode to %s (CiA 402 mode %u)",
                operating_mode_.c_str(), cia402_mode);

    // Object 0x6060:00 = Modes of Operation
    // 1-byte SDO write (command specifier 0x2F)
    send_sdo_write_1byte(0x6060, 0x00, cia402_mode);
    std::this_thread::sleep_for(delay);

    // ── Step 3: Enable the Motor (CiA 402 State Machine) ─────────────
    RCLCPP_INFO(this->get_logger(), "Step 3: CiA 402 state machine enable sequence");

    // Controlword (0x6040:00) — all are 2-byte SDO writes (0x2B)
    // Transition 2 – Shutdown
    send_sdo_write_2byte(0x6040, 0x00, 0x0006);
    std::this_thread::sleep_for(delay);

    // Transition 3 – Switch On
    send_sdo_write_2byte(0x6040, 0x00, 0x0007);
    std::this_thread::sleep_for(delay);

    // Transition 4 – Enable Operation
    send_sdo_write_2byte(0x6040, 0x00, 0x000F);
    std::this_thread::sleep_for(delay);

    // ── Step 4: Configure Motion Parameters ──────────────────────────
    RCLCPP_INFO(this->get_logger(), "Step 4: Configure motion parameters");

    // Profile Acceleration (0x6083:00)
    send_sdo_write(0x6083, 0x00, static_cast<uint32_t>(profile_acceleration_));
    std::this_thread::sleep_for(delay);

    // Profile Deceleration (0x6084:00)
    send_sdo_write(0x6084, 0x00, static_cast<uint32_t>(profile_deceleration_));
    std::this_thread::sleep_for(delay);

    if (is_position) {
      // Profile Velocity (0x6081:00) — the velocity used during position moves
      send_sdo_write(0x6081, 0x00, static_cast<uint32_t>(profile_velocity_));
      std::this_thread::sleep_for(delay);

      // Set initial Target Position (0x607A:00) to 0
      send_sdo_write(0x607A, 0x00, 0);
      std::this_thread::sleep_for(delay);
    } else {
      // Target Velocity (0x60FF:00) — initial velocity
      send_sdo_write(0x60FF, 0x00, static_cast<uint32_t>(profile_velocity_));
      std::this_thread::sleep_for(delay);
    }

    RCLCPP_INFO(this->get_logger(), "=== Motor Initialization Complete ===");
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if ((int)msg->axes.size() <= joy_axis_) {
      RCLCPP_WARN(this->get_logger(), "Joy message has no axis %d", joy_axis_);
      return;
    }

    double axis = msg->axes[joy_axis_];

    if (operating_mode_ == "position") {
      // Map axis (-1..1) to target position in pulses
      int32_t position = static_cast<int32_t>(axis * static_cast<double>(max_position_));

      RCLCPP_INFO(this->get_logger(), "Axis %d: %.3f -> position %d", joy_axis_, axis, position);

      // Write Target Position (0x607A:00)
      send_sdo_write(0x607A, 0x00, static_cast<uint32_t>(position));

      // Trigger the move: set controlword bit 4 (new setpoint) + bits 0-3 (enable)
      // 0x001F = enable operation (0x000F) | new setpoint (bit 4)
      send_sdo_write_2byte(0x6040, 0x00, 0x001F);

      // Clear the new-setpoint bit so the next write is accepted
      // (some drives require the rising edge of bit 4)
      send_sdo_write_2byte(0x6040, 0x00, 0x000F);

    } else {
      // Velocity mode: map axis to velocity
      int32_t velocity = static_cast<int32_t>(axis * static_cast<double>(max_velocity_));

      RCLCPP_INFO(this->get_logger(), "Axis %d: %.3f -> velocity %d", joy_axis_, axis, velocity);

      // Write Target Velocity (0x60FF:00)
      send_sdo_write(target_velocity_index_, target_velocity_subindex_,
                     static_cast<uint32_t>(velocity));
    }
  }

  void logCanFrame(struct can_frame frame)
  {
    std::ostringstream msg;
    for (size_t i = 0; i < frame.can_dlc; ++i) {
      msg << std::setw(2) << std::setfill('0') << std::hex << (int)frame.data[i];
    }
    RCLCPP_DEBUG(this->get_logger(), "CAN FRAME: %s", msg.str().c_str());
                 
  }



  // Members
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  std::string can_interface_ = "can0";
  int can_socket_ = -1;
  bool shutdown_complete_ = false;
  uint8_t node_id_ = 1;
  int max_velocity_ = 1000;       // max velocity in pulses/s (velocity mode joystick scaling)
  int max_position_ = 100000;     // max position in pulses   (position mode joystick scaling)
  int joy_axis_ = 1;
  std::string operating_mode_ = "position";  // "position" or "velocity"
  int profile_velocity_ = 10000;    // pulses/s  (velocity during position moves, or initial target in velocity mode)
  int profile_acceleration_ = 1000000;  // pulses/s²
  int profile_deceleration_ = 1000000;  // pulses/s²
  uint16_t target_velocity_index_ = 0x60FF;
  uint8_t target_velocity_subindex_ = 0x00;
};

// Global weak_ptr so the signal handler can trigger shutdown without preventing destruction
std::weak_ptr<JoyToPDONode> g_node;

void signal_handler(int /*signum*/)
{
  // Try to stop the motor even if rclcpp shutdown doesn't run the destructor in time
  auto node = g_node.lock();
  if (node) {
    node->shutdown_motor();
  }
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<JoyToPDONode>();
  g_node = node;

  // Install signal handlers so Ctrl+C triggers our shutdown_motor before exit
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  rclcpp::spin(node);

  // Ensure motor is stopped even if signals were not caught
  node->shutdown_motor();

  node.reset();
  rclcpp::shutdown();
  return 0;
}
