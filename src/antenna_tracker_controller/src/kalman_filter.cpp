#include "antenna_tracker_controller/kalman_filter.hpp"
#include <cstring>

namespace antenna_tracker_controller
{

KalmanFilterAzEl::KalmanFilterAzEl()
: Q_(0.001), R_(2.0), dt_(0.01)
{
  state_.fill(0.0);
  for (auto & row : P_) {
    row.fill(0.0);
  }
  for (int i = 0; i < 4; i++) {
    P_[i][i] = 1.0;
  }
}

void KalmanFilterAzEl::init(double dt, double q_process, double r_measurement)
{
  dt_ = dt;
  Q_ = q_process;
  R_ = r_measurement;

  state_.fill(0.0);
  for (auto & row : P_) {
    row.fill(0.0);
  }
  for (int i = 0; i < 4; i++) {
    P_[i][i] = 1.0;
  }
}

void KalmanFilterAzEl::update(double az_meas, double el_meas)
{
  /* Predict: state transition [az, az_vel, el, el_vel] */
  std::array<double, 4> pred;
  pred[0] = state_[0] + state_[1] * dt_;  /* az = az + az_vel * dt */
  pred[1] = state_[1];                    /* az_vel constant */
  pred[2] = state_[2] + state_[3] * dt_;  /* el = el + el_vel * dt */
  pred[3] = state_[3];                    /* el_vel constant */

  /* Covariance prediction (simplified diagonal) */
  for (int i = 0; i < 4; i++) {
    P_[i][i] += Q_;
  }

  /* Update: azimuth measurement */
  double K_az = P_[0][0] / (P_[0][0] + R_);
  pred[0] = pred[0] + K_az * (az_meas - pred[0]);
  P_[0][0] = (1.0 - K_az) * P_[0][0];

  /* Update: elevation measurement */
  double K_el = P_[2][2] / (P_[2][2] + R_);
  pred[2] = pred[2] + K_el * (el_meas - pred[2]);
  P_[2][2] = (1.0 - K_el) * P_[2][2];

  state_ = pred;
}

}  // namespace antenna_tracker_controller
