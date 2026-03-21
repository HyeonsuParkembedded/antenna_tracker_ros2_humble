#include <gtest/gtest.h>
#include "antenna_tracker_controller/navigation_node.hpp"
#include <cmath>

using namespace antenna_tracker_controller;

TEST(NavigationMathTest, HaversineDistance) {
  double lat1 = 37.5665;
  double lon1 = 126.9780;
  double lat2 = 35.1796;
  double lon2 = 129.0756;
  
  double dist = NavigationNode::haversine_distance(lat1, lon1, lat2, lon2);
  EXPECT_NEAR(dist, 325000.0, 10000.0);
}

TEST(NavigationMathTest, ElevationAngle) {
  double el = NavigationNode::elevation_angle(37.0, 127.0, 100.0, 37.0, 127.0, 200.0);
  EXPECT_NEAR(el, 90.0, 1e-6);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
