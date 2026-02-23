
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
    
    can_interface_ = this->get_parameter("can_interface").as_string();
    node_id_ = static_cast<uint8_t>(this->get_parameter("canopen_node_id").as_int());
    max_velocity_ = this->get_parameter("max_velocity").as_int();
    joy_axis_ = this->get_parameter("joy_axis").as_int();

    // Initialize socketCAN
    if (!init_can_socket()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN socket on interface %s", 
                   can_interface_.c_str());
      throw std::runtime_error("CAN socket initialization failed");
    }

    // Subscribe to joystick topic
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyToPDONode::joy_callback, this, _1));

    // Send NMT command to transition node to operational
    send_nmt_command(0x01);  // 0x01 = Start (operational)
    
    RCLCPP_INFO(this->get_logger(), 
      "JoyToPDONode initialized on %s for node %u (target velocity: 0x%04X:%02X)",
      can_interface_.c_str(), node_id_, target_velocity_index_, target_velocity_subindex_);
  }

  ~JoyToPDONode()
  {
    // Send NMT command to stop the node
    send_nmt_command(0x02);  // 0x02 = Stop
    
    if (can_socket_ >= 0) {
      close(can_socket_);
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

    RCLCPP_INFO(this->get_logger(), "Successfully bound to CAN interface %s", can_interface_.c_str());
    return true;
  }

  void send_nmt_command(uint8_t command)
  {
    struct can_frame frame;
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

  void send_sdo_write(uint16_t index, uint8_t subindex, uint32_t value)
  {
    struct can_frame frame;
    
    frame.can_id = 0x600 + node_id_;
    frame.can_dlc = 8;  
    
    frame.data[0] = 0x23;  // writes 4 bytes of data
    
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
      RCLCPP_DEBUG(this->get_logger(), 
        "Sent SDO write: index=0x%04X subindex=0x%02X value=0x%08X to node %u",
        index, subindex, value, node_id_);

        logCanFrame(frame); //show what the structure of the frame looks like
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if ((int)msg->axes.size() <= joy_axis_) {
      RCLCPP_WARN(this->get_logger(), "Joy message has no axis %d", joy_axis_);
      return;
    }

    // Read axis value (-1..1) and map to integer velocity
    double axis = msg->axes[joy_axis_];
    int32_t velocity = static_cast<int32_t>(axis * static_cast<double>(max_velocity_));

    RCLCPP_INFO(this->get_logger(), "Axis %d: %.3f -> velocity %d", joy_axis_, axis, velocity);

    // Send velocity command 
    send_sdo_write(target_velocity_index_, target_velocity_subindex_, 
                   static_cast<uint32_t>(velocity));

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
  uint8_t node_id_ = 1;
  int max_velocity_ = 1000; //TODO: confirm from EDS what max velocity should be (should be in pulses per second)
  int joy_axis_ = 1;
  uint16_t target_velocity_index_ = 0x60FF;
  uint8_t target_velocity_subindex_ = 0x00;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyToPDONode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
