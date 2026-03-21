#include <gtest/gtest.h>
#include "antenna_tracker_controller/complementary_filter.hpp"

using namespace antenna_tracker_controller;

TEST(ComplementaryFilterTest, BasicInitialization) {
  ComplementaryFilter filter;
  filter.set_alpha(0.98);
  filter.set_declination(-8.0);
  
  EXPECT_DOUBLE_EQ(filter.orientation().roll, 0.0);
  EXPECT_DOUBLE_EQ(filter.orientation().pitch, 0.0);
  EXPECT_DOUBLE_EQ(filter.orientation().yaw, 0.0);
}

TEST(ComplementaryFilterTest, UpdateLogic) {
  ComplementaryFilter filter;
  filter.set_alpha(0.98);
  filter.set_declination(-8.0);
  
  filter.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 30.0, 0.0, 30.0, 1.0);
  filter.update(1.0, 0.0, 9.81, 0.1, 0.0, 0.0, 30.0, 0.0, 30.0, 1.01);
  
  EXPECT_NE(filter.orientation().pitch, 0.0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
