
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "odrive/Constants.hpp"
#include <thread>
#include <atomic>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;
using namespace ODriveCommands;

class ODriveCanNode : public rclcpp::Node {
public:
  ODriveCanNode()
  : Node("odrive_can_node")
  {
    this->declare_parameter<std::string>("can_interface", "can2");
    this->declare_parameter<int>("node_id", 0);
    this->declare_parameter<int>("joy_axis", 1);
    this->declare_parameter<double>("joy_scale", 1.0); // max is 1 rev/s

    this->get_parameter("can_interface", can_if_);
    this->get_parameter("node_id", node_id_);
    this->get_parameter("joy_axis", joy_axis_);
    this->get_parameter("joy_scale", joy_scale_);

    RCLCPP_INFO(this->get_logger(), "Opening CAN interface: %s", can_if_.c_str());

    sock_ = open_can_socket(can_if_);
    if (sock_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CAN socket: %s", strerror(errno));
      throw std::runtime_error("Failed to open CAN socket");
    }

    // Put axis into CLOSED_LOOP_CONTROL (AxisState = 8)
    bool ok = send_set_axis_state(sock_, node_id_, ODriveCommands::CLOSED_LOOP_CONTROL_STATE);
    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send Set_Axis_State");
    } else {
      RCLCPP_INFO(this->get_logger(), "Sent Set_Axis_State(CLOSED_LOOP_CONTROL). Waiting for heartbeat...");
      // wait for heartbeat message for 5s indicating axis state change, if it doesn't arrive, still continue but warn
      if (wait_for_axis_state(sock_, node_id_, ODriveCommands::CLOSED_LOOP_CONTROL_STATE, 5000)) {
        RCLCPP_INFO(this->get_logger(), "Axis entered CLOSED_LOOP_CONTROL");
      } else {
        RCLCPP_WARN(this->get_logger(), "Timeout waiting for axis state change; continuing anyway");
      }
    }

    // Subscribe to joystick topic (joy_linux publishes sensor_msgs::msg::Joy by default)
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10,
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (static_cast<int>(msg->axes.size()) <= joy_axis_) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Joy message has no axis %d", joy_axis_);
          return;
        }
        float axis = static_cast<float>(msg->axes[joy_axis_]);
        float vel = axis * static_cast<float>(joy_scale_);
        if (!send_set_input_vel(sock_, node_id_, vel, 0.0f)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to send Set_Input_Vel from joy");
        } else {
          RCLCPP_DEBUG(this->get_logger(), "Joy -> velocity %.6f (axis %d = %.3f)", vel, joy_axis_, axis);
        }
      }
    );

    // Publisher for encoder estimates (position, velocity)
    enc_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("encoder_estimates", 10);

    // Start CAN reader thread
    running_.store(true);
    reader_thread_ = std::thread([this]() { this->can_read_loop(); });
  }

  ~ODriveCanNode() override
  {
    // stop reader thread first
    running_.store(false);
    if (reader_thread_.joinable()) reader_thread_.join();
    if (sock_ >= 0) close(sock_);
  }

private:
  /**
   * @brief Opens and binds a SocketCAN socket to the specified CAN interface.
   * @param ifname Name of the CAN network interface (e.g., "can0")
   * @return File descriptor for the CAN socket on success, -1 on failure
   */
  int open_can_socket(const std::string &ifname)
  {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return -1;

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
      close(s);
      return -1;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      close(s);
      return -1;
    }

    return s;
  }

  /**
   * @brief Constructs a CANSimple message ID from node_id and command ID.
   * @details According to ODrive CANSimple protocol, the message ID is formed by shifting
   * node_id left by 5 bits and ORing with the command ID. This ensures a valid 11-bit standard CAN ID.
   * @param node_id The ODrive node ID (0-63)
   * @param cmd_id The command ID (0-31)
   * @return The constructed 11-bit CAN message ID
   */
  static uint32_t make_can_id(uint8_t node_id, uint8_t cmd_id)
  {
    return ((static_cast<uint32_t>(node_id) << 5) | static_cast<uint32_t>(cmd_id)) & CAN_SFF_MASK;
  }

  /**
   * @brief Sends a Set_Axis_State command over CAN (cmd 0x07).
   * @details Encodes the requested axis state as a 32-bit little-endian value into a CAN frame
   * and transmits it over the socket. Used to request axis state transitions such as CLOSED_LOOP_CONTROL.
   * @param s File descriptor for the CAN socket
   * @param node_id Target ODrive node ID
   * @param requested_state The desired axis state (e.g., 8 for CLOSED_LOOP_CONTROL)
   * @return true if the frame was successfully written, false otherwise
   */
  bool send_set_axis_state(int s, uint8_t node_id, uint32_t requested_state)
  {
    struct can_frame frame{};
    frame.can_id = make_can_id(node_id, ODriveCommands::AXIS_STATE_CMD_ID);
    frame.can_dlc = 4;
    uint32_t v = requested_state;

    std::memcpy(frame.data, &v, 4);

    ssize_t n = write(s, &frame, sizeof(frame));
    return (n == sizeof(frame));
  }

  /**
   * @brief Sends a Set_Input_Vel command over CAN (cmd 0x0d).
   * @details Encodes velocity and optional torque feedforward as IEEE 754 floats in little-endian
   * format and transmits over CAN. Velocity is in turns/s, torque feedforward in Nm.
   * @param s File descriptor for the CAN socket
   * @param node_id Target ODrive node ID
   * @param vel Desired velocity in turns/s (IEEE 754 float at bytes 0-3)
   * @param torque_ff Torque feedforward in Nm (IEEE 754 float at bytes 4-7)
   * @return true if the frame was successfully written, false otherwise
   */
  bool send_set_input_vel(int s, uint8_t node_id, float vel, float torque_ff)
  {
    struct can_frame frame{};
    frame.can_id = make_can_id(node_id, ODriveCommands::SET_INPUT_VEL);
    frame.can_dlc = 8;

    std::memcpy(&frame.data[0], &vel, sizeof(float));
    std::memcpy(&frame.data[4], &torque_ff, sizeof(float));

    ssize_t n = write(s, &frame, sizeof(frame));
    return (n == sizeof(frame));
  }

  /**
   * @brief Background thread loop that reads incoming CAN frames and publishes encoder estimates.
   * @details Runs in a separate thread and continuously polls the CAN socket using select()
   * with a 200ms timeout. Processes incoming frames and exits cleanly when running_ is set to false.
   * Assumes the CAN interface bitrate is configured externally via ip link
   */
  void can_read_loop()
  {
    while (running_.load()) {
      struct can_frame frame{};
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(sock_, &readfds);
      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 200000; // 200ms
      int rv = select(sock_ + 1, &readfds, nullptr, nullptr, &tv);
      if (rv > 0 && FD_ISSET(sock_, &readfds)) {
        ssize_t n = read(sock_, &frame, sizeof(frame));
        if (n > 0) {
          process_can_frame(frame);
        }
      } else if (rv < 0) {
        if (errno == EINTR) continue;
        RCLCPP_WARN(this->get_logger(), "select() on CAN socket failed: %s", strerror(errno));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  /**
   * @brief Processes a received CAN frame and publishes relevant data.
   * @details Checks the frame command ID. If it is Get_Encoder_Estimates (0x09),
   * extracts position and velocity floats and publishes as a JointState message.
   * Other frames are logged at DEBUG level.
   * @param frame The CAN frame to process
   */
  void process_can_frame(const struct can_frame &frame)
  {
    uint32_t id = frame.can_id & CAN_SFF_MASK;
    if (id == make_can_id(static_cast<uint8_t>(node_id_), ODriveCommands::ENCODER_ESTIMATES)) {

      if (frame.can_dlc >= 8) {
        float pos = 0.0f, vel = 0.0f;
        std::memcpy(&pos, &frame.data[0], sizeof(float));
        std::memcpy(&vel, &frame.data[4], sizeof(float));
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = {"axis0"};
        msg.position = {static_cast<double>(pos)};
        msg.velocity = {static_cast<double>(vel)};
        enc_pub_->publish(msg);
      }

    }
    else{
      uint8_t cmd_id = id & 0x1F; // lower 5 bits for command ID

      RCLCPP_DEBUG(this->get_logger(), "Received CAN frame with ID 0x%03x, With total frame length: %d", cmd_id, frame.can_dlc); // note: dlc includes node_id
    }
  }

  /**
   * @brief Waits for a heartbeat message indicating the axis has reached a desired state.
   * @details Polls the CAN socket using select() with 100ms timeouts until either the
   * heartbeat message indicates the desired axis state or the timeout expires.
   * @param s File descriptor for the CAN socket
   * @param node_id Target ODrive node ID
   * @param desired_state The axis state value to wait for (e.g., 8 for CLOSED_LOOP_CONTROL)
   * @param timeout_ms Total timeout in milliseconds
   * @return true if the desired state is confirmed within the timeout, false if timeout expires
   */
  bool wait_for_axis_state(int s, uint8_t node_id, uint8_t desired_state, int timeout_ms)
  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    uint32_t hb_id = make_can_id(node_id, ODriveCommands::HEARTBEAT_CMD_ID);
    while (std::chrono::steady_clock::now() < deadline) {
      struct can_frame frame{};
      // Use a short timeout on read via poll/select instead of blocking forever
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(s, &readfds);
      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 100000; // 100ms
      int rv = select(s+1, &readfds, nullptr, nullptr, &tv);
      if (rv > 0 && FD_ISSET(s, &readfds)) {
        ssize_t n = read(s, &frame, sizeof(frame));
        if (n >= 0) {
          if ((frame.can_id & CAN_SFF_MASK) == hb_id && frame.can_dlc >= 5) {
            // Axis_State is at offset 4
            uint8_t state = frame.data[4];
            if (state == desired_state) return true;
          }
        }
      }
    }
    return false;
  }

  std::string can_if_ = "can2";
  int node_id_ = 0;
  int sock_ = -1;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr enc_pub_;
  std::thread reader_thread_;
  std::atomic<bool> running_{false};
  std::string joy_topic_ = "joy";
  int joy_axis_ = 1;
  double joy_scale_ = 1.0;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ODriveCanNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
