#include "odesc/CanUtils.hpp"

#include <cerrno>
#include <cstring>

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace odesc {
namespace can {

int open_can_socket(const std::string &ifname, const rclcpp::Logger &logger)
{
  int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd < 0) {
    RCLCPP_ERROR(logger, "Failed to open CAN socket: %s", strerror(errno));
    return -1;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(logger, "Failed to open CAN socket: %s", strerror(errno));
    close(socket_fd);
    return -1;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(logger, "Failed to open CAN socket: %s", strerror(errno));
    close(socket_fd);
    return -1;
  }

  return socket_fd;
}

void close_can_socket(int &socket_fd)
{
  if (socket_fd >= 0) {
    close(socket_fd);
    socket_fd = -1;
  }
}

uint32_t make_can_id(uint8_t node_id, uint8_t cmd_id)
{
  return ((static_cast<uint32_t>(node_id) << 5) | static_cast<uint32_t>(cmd_id)) & CAN_SFF_MASK;
}

bool send_set_axis_state(int socket_fd, uint8_t node_id, uint32_t requested_state)
{
  struct can_frame frame{};
  frame.can_id = make_can_id(node_id, CMD_SET_AXIS_STATE);
  frame.can_dlc = 4;
  std::memcpy(frame.data, &requested_state, sizeof(uint32_t));
  ssize_t n = write(socket_fd, &frame, sizeof(frame));
  return (n == sizeof(frame));
}

bool send_set_input_vel(int socket_fd, uint8_t node_id, float velocity, float torque_ff)
{
  struct can_frame frame{};
  frame.can_id = make_can_id(node_id, CMD_SET_INPUT_VEL);
  frame.can_dlc = 8;
  std::memcpy(&frame.data[0], &velocity, sizeof(float));
  std::memcpy(&frame.data[4], &torque_ff, sizeof(float));
  ssize_t n = write(socket_fd, &frame, sizeof(frame));
  return (n == sizeof(frame));
}

bool read_can_frame(int socket_fd, struct can_frame &frame, int timeout_ms, const rclcpp::Logger &logger)
{
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(socket_fd, &readfds);

  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int rv = select(socket_fd + 1, &readfds, nullptr, nullptr, &tv);
  if (rv <= 0) {
    if (rv < 0) {
      RCLCPP_WARN(logger, "select() on CAN socket failed: %s", strerror(errno));
    }
    return false;
  }

  if (!FD_ISSET(socket_fd, &readfds)) {
    return false;
  }

  ssize_t n = read(socket_fd, &frame, sizeof(frame));
  return (n > 0);
}

bool parse_encoder_estimates(const struct can_frame &frame, uint8_t node_id, float &position, float &velocity)
{
  uint32_t id = frame.can_id & CAN_SFF_MASK;
  if (id != make_can_id(node_id, CMD_GET_ENCODER_ESTIMATES)) {
    return false;
  }

  if (frame.can_dlc < 8) {
    return false;
  }

  std::memcpy(&position, &frame.data[0], sizeof(float));
  std::memcpy(&velocity, &frame.data[4], sizeof(float));
  return true;
}

}  // namespace can
}  // namespace odesc
