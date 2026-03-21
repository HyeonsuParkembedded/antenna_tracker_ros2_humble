#include <gtest/gtest.h>
#include "antenna_tracker_controller/mpc_controller.hpp"

using namespace antenna_tracker_controller;

TEST(MpcControllerTest, BasicExecution) {
  MpcController mpc;
  mpc.init();
  
  double az_out = 0.0;
  double el_out = 0.0;

  mpc.compute(90.0, 0.0, 0.0, 45.0, 0.0, 0.0, az_out, el_out);

  // outputs should try to move towards target
  EXPECT_NE(az_out, 0.0);
  EXPECT_NE(el_out, 0.0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
