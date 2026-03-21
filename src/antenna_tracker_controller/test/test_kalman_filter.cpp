#include <gtest/gtest.h>
#include "antenna_tracker_controller/kalman_filter.hpp"

using namespace antenna_tracker_controller;

TEST(KalmanFilterTest, Initialization) {
  KalmanFilterAzEl filter;
  filter.init(0.01, 0.001, 2.0);
  EXPECT_DOUBLE_EQ(filter.azimuth(), 0.0);
  EXPECT_DOUBLE_EQ(filter.az_velocity(), 0.0);
}

TEST(KalmanFilterTest, UpdateConvergence) {
  KalmanFilterAzEl filter;
  filter.init(0.01, 0.001, 2.0);
  
  for (int i = 0; i < 100; ++i) {
    filter.update(90.0, 45.0);
  }
  
  EXPECT_NEAR(filter.azimuth(), 90.0, 2.0);
  EXPECT_NEAR(filter.elevation(), 45.0, 2.0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
