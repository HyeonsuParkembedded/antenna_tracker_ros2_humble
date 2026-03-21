#include "antenna_tracker_controller/complementary_filter.hpp"
#include <cmath>

namespace antenna_tracker_controller
{

static constexpr double RAD_TO_DEG = 180.0 / M_PI;
static constexpr double DEG_TO_RAD = M_PI / 180.0;

ComplementaryFilter::ComplementaryFilter()
: alpha_(0.98),
  declination_(0.0),
  last_timestamp_sec_(0.0),
  initialized_(false)
{
}

void ComplementaryFilter::set_alpha(double alpha)
{
  alpha_ = alpha;
}

void ComplementaryFilter::set_declination(double declination_deg)
{
  declination_ = declination_deg;
}

void ComplementaryFilter::update(
  double accel_x, double accel_y, double accel_z,
  double gyro_x, double gyro_y, double gyro_z,
  double mag_x, double mag_y, double mag_z,
  double timestamp_sec)
{
  double dt = 0.01;
  if (initialized_ && timestamp_sec > last_timestamp_sec_) {
    dt = timestamp_sec - last_timestamp_sec_;
  }
  last_timestamp_sec_ = timestamp_sec;
  initialized_ = true;

  /* Roll & Pitch from accelerometer */
  double accel_roll = std::atan2(accel_y, accel_z) * RAD_TO_DEG;
  double accel_pitch = std::atan2(
    -accel_x, std::sqrt(accel_y * accel_y + accel_z * accel_z)) * RAD_TO_DEG;

  /* Gyro integration (gyro in deg/s) */
  double gyro_roll = orientation_.roll + gyro_x * dt;
  double gyro_pitch = orientation_.pitch + gyro_y * dt;
  double gyro_yaw = orientation_.yaw + gyro_z * dt;

  /* Complementary filter: 98% gyro, 2% accel */
  orientation_.roll = alpha_ * gyro_roll + (1.0 - alpha_) * accel_roll;
  orientation_.pitch = alpha_ * gyro_pitch + (1.0 - alpha_) * accel_pitch;
  orientation_.yaw = gyro_yaw;

  /* Tilt-compensated magnetometer heading */
  double roll_rad = orientation_.roll * DEG_TO_RAD;
  double mx = mag_x;
  double my = mag_y * std::cos(roll_rad) - mag_z * std::sin(roll_rad);

  double heading = std::atan2(my, mx) * RAD_TO_DEG;
  heading += declination_;

  if (heading < 0.0) {
    heading += 360.0;
  } else if (heading >= 360.0) {
    heading -= 360.0;
  }

  orientation_.azimuth = heading;
  orientation_.elevation = orientation_.pitch;
}

}  // namespace antenna_tracker_controller
