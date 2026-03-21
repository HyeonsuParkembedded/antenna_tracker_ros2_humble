#include <gtest/gtest.h>
#include "antenna_tracker_controller/pid_controller.hpp"

using namespace antenna_tracker_controller;

TEST(CascadePidTest, ClampLogic) {
  PidGains outer{1.0, 0.0, 0.0};
  PidGains inner{1.0, 0.0, 0.0};
  
  CascadePid pid;
  pid.init(outer, inner, 0.01, -10.0, 10.0);
  
  double out = pid.compute(100.0, 0.0, 0.0);
  EXPECT_LE(out, 10.0);
}

TEST(DualAxisCascadePidTest, Initialization) {
  DualAxisCascadePid pid;
  pid.init(0.01);
  
  double az_out = 0.0, el_out = 0.0;
  pid.compute(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, az_out, el_out);
  EXPECT_DOUBLE_EQ(az_out, 0.0);
  EXPECT_DOUBLE_EQ(el_out, 50.0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
