#ifndef ANTENNA_TRACKER_CONTROLLER__MPC_CONTROLLER_HPP_
#define ANTENNA_TRACKER_CONTROLLER__MPC_CONTROLLER_HPP_

#include "acados_solver_antenna_tracker.h"

namespace antenna_tracker_controller
{

class MpcController
{
public:
  MpcController();
  ~MpcController();

  void init();
  
  void compute(
    double az_target, double az_current, double az_vel,
    double el_target, double el_current, double el_vel,
    double & az_output, double & el_output);

private:
  antenna_tracker_solver_capsule * acados_capsule_;
};

}  // namespace antenna_tracker_controller

#endif  // ANTENNA_TRACKER_CONTROLLER__MPC_CONTROLLER_HPP_
