#include "odrive/odrive_can_utils.hpp"
#include "odrive/Constants.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <chrono>

using namespace ODriveCommands;

namespace odrive {

int ODriveCAN::open_can_socket(const std::string &ifname)
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

  if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    close(s);
    return -1;
  }

  return s;
}

uint32_t ODriveCAN::make_can_id(uint8_t node_id, uint8_t cmd_id)
{
  return ((static_cast<uint32_t>(node_id) << 5) | static_cast<uint32_t>(cmd_id)) & CAN_SFF_MASK;
}

bool ODriveCAN::send_set_axis_state(int sock, uint8_t node_id, uint32_t requested_state)
{
  struct can_frame frame{};
  frame.can_id = make_can_id(node_id, AXIS_STATE_CMD_ID);
  frame.can_dlc = 4;
  uint32_t v = requested_state;
  std::memcpy(frame.data, &v, 4);
  
  ssize_t n = ::write(sock, &frame, sizeof(frame));
  return (n == sizeof(frame));
}

bool ODriveCAN::send_set_input_vel(int sock, uint8_t node_id, float vel, float torque_ff)
{
  struct can_frame frame{};
  frame.can_id = make_can_id(node_id, SET_INPUT_VEL);
  frame.can_dlc = 8;
  
  std::memcpy(&frame.data[0], &vel, sizeof(float));
  std::memcpy(&frame.data[4], &torque_ff, sizeof(float));
  
  ssize_t n = ::write(sock, &frame, sizeof(frame));
  return (n == sizeof(frame));
}

bool ODriveCAN::wait_for_axis_state(int sock, uint8_t node_id, uint8_t desired_state, int timeout_ms)
{
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  uint32_t hb_id = make_can_id(node_id, HEARTBEAT_CMD_ID);

  while (std::chrono::steady_clock::now() < deadline) {
    struct can_frame frame{};
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sock, &readfds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms
    
    int rv = select(sock + 1, &readfds, nullptr, nullptr, &tv);
    if (rv > 0 && FD_ISSET(sock, &readfds)) {
      ssize_t n = ::read(sock, &frame, sizeof(frame));
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

float ODriveCAN::meters_per_sec_to_turns_per_sec(float mps, float wheel_radius)
{
  return mps / (wheel_radius * 2.0f * static_cast<float>(M_PI));
}

float ODriveCAN::turns_to_radians(float turns)
{
  return turns * 2.0f * static_cast<float>(M_PI);
}

float ODriveCAN::turns_per_sec_to_rad_per_sec(float tps)
{
  return tps * 2.0f * static_cast<float>(M_PI);
}

float ODriveCAN::rad_per_sec_to_turns_per_sec(float rad_per_sec)
{
  return rad_per_sec / (2.0f * static_cast<float>(M_PI));
}

}  // namespace odrive
