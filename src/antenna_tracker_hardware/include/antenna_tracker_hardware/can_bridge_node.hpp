#ifndef ANTENNA_TRACKER_HARDWARE__CAN_BRIDGE_NODE_HPP_
#define ANTENNA_TRACKER_HARDWARE__CAN_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <antenna_tracker_msgs/msg/target_gps.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <string>
#include <thread>

namespace antenna_tracker_hardware
{

class CanBridgeNode : public rclcpp::Node
{
public:
  explicit CanBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CanBridgeNode() override;

private:
  void rx_thread_func();
  void process_target_gps(const struct can_frame & frame);
  void process_target_status(const struct can_frame & frame);

  /* CAN message IDs (from protocol.h) */
  static constexpr uint32_t CAN_ID_TARGET_GPS    = 0x100;
  static constexpr uint32_t CAN_ID_TARGET_STATUS = 0x101;

  rclcpp::Publisher<antenna_tracker_msgs::msg::TargetGPS>::SharedPtr pub_target_gps_;

  int can_socket_{-1};
  std::thread rx_thread_;
  std::atomic<bool> running_{false};

  /* Latest target status fields */
  float rssi_dbm_{0.0f};
  uint8_t link_quality_{0};
};

}  // namespace antenna_tracker_hardware

#endif  // ANTENNA_TRACKER_HARDWARE__CAN_BRIDGE_NODE_HPP_
