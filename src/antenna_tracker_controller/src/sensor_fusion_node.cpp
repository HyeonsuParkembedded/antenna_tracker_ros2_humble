#include "antenna_tracker_controller/sensor_fusion_node.hpp"

namespace antenna_tracker_controller
{

SensorFusionNode::SensorFusionNode(const rclcpp::NodeOptions & options)
: Node("sensor_fusion_node", options)
{
  declare_parameter<double>("complementary_alpha", 0.98);
  declare_parameter<double>("mag_declination_deg", -8.0);
  declare_parameter<double>("kalman_q_process", 0.001);
  declare_parameter<double>("kalman_r_measurement", 2.0);
  declare_parameter<double>("loop_rate_hz", 100.0);

  double alpha = get_parameter("complementary_alpha").as_double();
  double declination = get_parameter("mag_declination_deg").as_double();
  double kf_q = get_parameter("kalman_q_process").as_double();
  double kf_r = get_parameter("kalman_r_measurement").as_double();
  double loop_rate = get_parameter("loop_rate_hz").as_double();

  comp_filter_.set_alpha(alpha);
  comp_filter_.set_declination(declination);

  double dt = 1.0 / loop_rate;
  kalman_filter_.init(dt, kf_q, kf_r);

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/raw", rclcpp::SensorDataQoS(),
    std::bind(&SensorFusionNode::imu_callback, this, std::placeholders::_1));

  sub_mag_ = create_subscription<sensor_msgs::msg::MagneticField>(
    "/magnetic_field", rclcpp::SensorDataQoS(),
    std::bind(&SensorFusionNode::mag_callback, this, std::placeholders::_1));

  pub_fusion_ = create_publisher<antenna_tracker_msgs::msg::ImuFusion>(
    "/antenna/imu_fusion", rclcpp::SensorDataQoS());

  auto period = std::chrono::duration<double>(dt);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SensorFusionNode::timer_callback, this));

  RCLCPP_INFO(get_logger(), "SensorFusionNode initialized (%.0f Hz, alpha=%.2f, decl=%.1f)",
              loop_rate, alpha, declination);
}

void SensorFusionNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  latest_imu_ = msg;
  imu_received_ = true;
}

void SensorFusionNode::mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
  latest_mag_ = msg;
  mag_received_ = true;
}

void SensorFusionNode::timer_callback()
{
  if (!imu_received_ || !mag_received_) {
    return;
  }

  double timestamp = latest_imu_->header.stamp.sec +
                     latest_imu_->header.stamp.nanosec * 1e-9;

  /* IMU angular velocity: rad/s -> deg/s for complementary filter */
  double gyro_x = latest_imu_->angular_velocity.x * 57.29577951;
  double gyro_y = latest_imu_->angular_velocity.y * 57.29577951;
  double gyro_z = latest_imu_->angular_velocity.z * 57.29577951;

  /* Magnetometer: T -> uT for filter */
  double mag_x = latest_mag_->magnetic_field.x * 1e6;
  double mag_y = latest_mag_->magnetic_field.y * 1e6;
  double mag_z = latest_mag_->magnetic_field.z * 1e6;

  comp_filter_.update(
    latest_imu_->linear_acceleration.x,
    latest_imu_->linear_acceleration.y,
    latest_imu_->linear_acceleration.z,
    gyro_x, gyro_y, gyro_z,
    mag_x, mag_y, mag_z,
    timestamp);

  const auto & orient = comp_filter_.orientation();

  /* Kalman filter for smoothed azimuth/elevation + velocity estimation */
  kalman_filter_.update(orient.azimuth, orient.elevation);
  kalman_initialized_ = true;

  auto msg = antenna_tracker_msgs::msg::ImuFusion();
  msg.header.stamp = now();
  msg.header.frame_id = "imu_link";
  msg.roll = orient.roll;
  msg.pitch = orient.pitch;
  msg.yaw = orient.yaw;
  msg.azimuth = kalman_filter_.azimuth();
  msg.elevation = kalman_filter_.elevation();
  msg.az_velocity = kalman_filter_.az_velocity();
  msg.el_velocity = kalman_filter_.el_velocity();
  msg.kalman_valid = kalman_initialized_;

  pub_fusion_->publish(msg);
}

}  // namespace antenna_tracker_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antenna_tracker_controller::SensorFusionNode>());
  rclcpp::shutdown();
  return 0;
}
