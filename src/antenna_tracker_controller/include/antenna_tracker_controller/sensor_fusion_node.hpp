#ifndef ANTENNA_TRACKER_CONTROLLER__SENSOR_FUSION_NODE_HPP_
#define ANTENNA_TRACKER_CONTROLLER__SENSOR_FUSION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <antenna_tracker_msgs/msg/imu_fusion.hpp>
#include "antenna_tracker_controller/complementary_filter.hpp"
#include "antenna_tracker_controller/kalman_filter.hpp"

namespace antenna_tracker_controller
{

class SensorFusionNode : public rclcpp::Node
{
public:
  explicit SensorFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
  void timer_callback();

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_mag_;
  rclcpp::Publisher<antenna_tracker_msgs::msg::ImuFusion>::SharedPtr pub_fusion_;
  rclcpp::TimerBase::SharedPtr timer_;

  ComplementaryFilter comp_filter_;
  KalmanFilterAzEl kalman_filter_;

  sensor_msgs::msg::Imu::SharedPtr latest_imu_;
  sensor_msgs::msg::MagneticField::SharedPtr latest_mag_;

  bool imu_received_{false};
  bool mag_received_{false};
  bool kalman_initialized_{false};
};

}  // namespace antenna_tracker_controller

#endif
