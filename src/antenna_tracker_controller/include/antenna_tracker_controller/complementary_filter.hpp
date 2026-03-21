#ifndef ANTENNA_TRACKER_CONTROLLER__COMPLEMENTARY_FILTER_HPP_
#define ANTENNA_TRACKER_CONTROLLER__COMPLEMENTARY_FILTER_HPP_

#include <cstdint>

namespace antenna_tracker_controller
{

struct Orientation {
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  double azimuth{0.0};
  double elevation{0.0};
};

class ComplementaryFilter
{
public:
  ComplementaryFilter();

  void set_alpha(double alpha);
  void set_declination(double declination_deg);

  void update(
    double accel_x, double accel_y, double accel_z,
    double gyro_x, double gyro_y, double gyro_z,
    double mag_x, double mag_y, double mag_z,
    double timestamp_sec);

  const Orientation & orientation() const { return orientation_; }

private:
  double alpha_;
  double declination_;
  Orientation orientation_;
  double last_timestamp_sec_;
  bool initialized_;
};

}  // namespace antenna_tracker_controller

#endif  // ANTENNA_TRACKER_CONTROLLER__COMPLEMENTARY_FILTER_HPP_
