#include "antenna_tracker_controller/mpc_controller.hpp"
#include <iostream>
#include <cmath>
#include "acados_c/ocp_nlp_interface.h"

namespace antenna_tracker_controller
{

MpcController::MpcController() : acados_capsule_(nullptr)
{
}

MpcController::~MpcController()
{
  if (acados_capsule_) {
    antenna_tracker_acados_free(acados_capsule_);
    antenna_tracker_acados_free_capsule(acados_capsule_);
  }
}

void MpcController::init()
{
  acados_capsule_ = antenna_tracker_acados_create_capsule();
  int status = antenna_tracker_acados_create(acados_capsule_);
  if (status) {
    std::cerr << "antenna_tracker_acados_create() returned status " << status << std::endl;
  }
}

void MpcController::compute(
  double az_target, double az_current, double az_vel,
  double el_target, double el_current, double el_vel,
  double & az_output, double & el_output)
{
  if (!acados_capsule_) {
    az_output = 0.0;
    el_output = 0.0;
    return;
  }

  // State: [az, az_vel, el, el_vel]
  double x0[ANTENNA_TRACKER_NX];
  x0[0] = az_current * M_PI / 180.0;
  x0[1] = az_vel * M_PI / 180.0;
  x0[2] = el_current * M_PI / 180.0;
  x0[3] = el_vel * M_PI / 180.0;

  ocp_nlp_in * nlp_in = antenna_tracker_acados_get_nlp_in(acados_capsule_);
  ocp_nlp_config * nlp_config = antenna_tracker_acados_get_nlp_config(acados_capsule_);
  ocp_nlp_dims * nlp_dims = antenna_tracker_acados_get_nlp_dims(acados_capsule_);
  ocp_nlp_out * nlp_out = antenna_tracker_acados_get_nlp_out(acados_capsule_);

  // Set initial condition
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);

  // Set reference
  double yref[ANTENNA_TRACKER_NY] = {0};
  yref[0] = az_target * M_PI / 180.0;
  yref[1] = 0.0;
  yref[2] = el_target * M_PI / 180.0;
  yref[3] = 0.0;
  yref[4] = 0.0; // u_az ref
  yref[5] = 0.0; // u_el ref

  double yref_e[ANTENNA_TRACKER_NYN] = {0};
  yref_e[0] = az_target * M_PI / 180.0;
  yref_e[1] = 0.0;
  yref_e[2] = el_target * M_PI / 180.0;
  yref_e[3] = 0.0;

  for (int i = 0; i < ANTENNA_TRACKER_N; ++i) {
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
  }
  ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ANTENNA_TRACKER_N, "yref", yref_e);

  int status = antenna_tracker_acados_solve(acados_capsule_);
  if (status != 0) {
    std::cerr << "antenna_tracker_acados_solve() failed with status " << status << std::endl;
  }

  double u_out[ANTENNA_TRACKER_NU];
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_out);

  az_output = u_out[0];
  el_output = u_out[1];
}

}  // namespace antenna_tracker_controller
