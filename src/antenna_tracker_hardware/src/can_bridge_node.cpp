#include "antenna_tracker_hardware/can_bridge_node.hpp"
#include <cstring>

namespace antenna_tracker_hardware
{

CanBridgeNode::CanBridgeNode(const rclcpp::NodeOptions & options)
: Node("can_bridge_node", options)
{
  declare_parameter<std::string>("can_interface", "can0");
  declare_parameter<int>("can_bitrate", 500000);

  pub_target_gps_ = create_publisher<antenna_tracker_msgs::msg::TargetGPS>(
    "/antenna/target_gps", rclcpp::SensorDataQoS());

  std::string can_iface = get_parameter("can_interface").as_string();

  /* Open SocketCAN */
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open CAN socket: %s", strerror(errno));
    return;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, can_iface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(get_logger(), "CAN interface '%s' not found", can_iface.c_str());
    close(can_socket_);
    can_socket_ = -1;
    return;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to bind CAN socket");
    close(can_socket_);
    can_socket_ = -1;
    return;
  }

  /* Set receive filter for target messages only */
  struct can_filter rfilter[2];
  rfilter[0].can_id = CAN_ID_TARGET_GPS;
  rfilter[0].can_mask = CAN_SFF_MASK;
  rfilter[1].can_id = CAN_ID_TARGET_STATUS;
  rfilter[1].can_mask = CAN_SFF_MASK;
  setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  RCLCPP_INFO(get_logger(), "CAN bridge listening on %s", can_iface.c_str());

  running_ = true;
  rx_thread_ = std::thread(&CanBridgeNode::rx_thread_func, this);
}

CanBridgeNode::~CanBridgeNode()
{
  running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  if (can_socket_ >= 0) {
    close(can_socket_);
  }
}

void CanBridgeNode::rx_thread_func()
{
  struct can_frame frame;

  while (running_) {
    /* Use select with timeout for graceful shutdown */
    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(can_socket_, &rdfs);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; /* 100ms timeout */

    int ret = select(can_socket_ + 1, &rdfs, nullptr, nullptr, &tv);
    if (ret <= 0) {
      continue;
    }

    ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
      continue;
    }

    switch (frame.can_id & CAN_SFF_MASK) {
      case CAN_ID_TARGET_GPS:
        process_target_gps(frame);
        break;
      case CAN_ID_TARGET_STATUS:
        process_target_status(frame);
        break;
      default:
        break;
    }
  }
}

void CanBridgeNode::process_target_gps(const struct can_frame & frame)
{
  if (frame.can_dlc < 8) {
    return;
  }

  /* Protocol: lat * 1e7 (int32) + lon * 1e7 (int32) */
  int32_t lat_raw, lon_raw;
  std::memcpy(&lat_raw, &frame.data[0], 4);
  std::memcpy(&lon_raw, &frame.data[4], 4);

  auto msg = antenna_tracker_msgs::msg::TargetGPS();
  msg.header.stamp = now();
  msg.header.frame_id = "target";
  msg.latitude = lat_raw / 1e7;
  msg.longitude = lon_raw / 1e7;
  msg.altitude_m = 0.0; /* Updated from status message */
  msg.rssi_dbm = rssi_dbm_;
  msg.link_quality = link_quality_;

  pub_target_gps_->publish(msg);
}

void CanBridgeNode::process_target_status(const struct can_frame & frame)
{
  if (frame.can_dlc < 5) {
    return;
  }

  /* Protocol: altitude (int16) + rssi (int16) + status (uint8) */
  int16_t altitude, rssi;
  std::memcpy(&altitude, &frame.data[0], 2);
  std::memcpy(&rssi, &frame.data[2], 2);

  rssi_dbm_ = static_cast<float>(rssi);
  link_quality_ = frame.data[4];
}

}  // namespace antenna_tracker_hardware

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antenna_tracker_hardware::CanBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
