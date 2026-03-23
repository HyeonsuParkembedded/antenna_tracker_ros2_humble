#include <gtest/gtest.h>
#include "antenna_tracker_controller/navigation_node.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

using namespace antenna_tracker_controller;

namespace
{

using namespace std::chrono_literals;

template<typename PredicateT>
bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor,
  PredicateT predicate,
  std::chrono::milliseconds timeout = 500ms)
{
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  executor.spin_some();
  return predicate();
}

class NavigationNodeRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    nav_node_ = std::make_shared<NavigationNode>();
    io_node_ = std::make_shared<rclcpp::Node>("navigation_node_test_io");

    gps_pub_ = io_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", rclcpp::SensorDataQoS());
    target_pub_ = io_node_->create_publisher<antenna_tracker_msgs::msg::TargetGPS>(
      "/antenna/target_gps", rclcpp::SensorDataQoS());
    mode_pub_ = io_node_->create_publisher<std_msgs::msg::UInt8>("/antenna/mode", 10);

    az_sub_ = io_node_->create_subscription<std_msgs::msg::Float64>(
      "/antenna/target_azimuth", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        last_azimuth_ = msg->data;
        azimuth_received_ = true;
      });
    el_sub_ = io_node_->create_subscription<std_msgs::msg::Float64>(
      "/antenna/target_elevation", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        last_elevation_ = msg->data;
        elevation_received_ = true;
      });

    executor_.add_node(nav_node_);
    executor_.add_node(io_node_);
    spin_for(150ms);
  }

  void TearDown() override
  {
    if (nav_node_) {
      executor_.remove_node(nav_node_);
      nav_node_.reset();
    }
    if (io_node_) {
      executor_.remove_node(io_node_);
      io_node_.reset();
    }
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  template<typename MessageT>
  void publish_reliably(const typename rclcpp::Publisher<MessageT>::SharedPtr & publisher,
                        const MessageT & message)
  {
    for (int i = 0; i < 3; ++i) {
      publisher->publish(message);
      spin_for(40ms);
    }
  }

  void publish_ground_fix(double latitude, double longitude, double altitude, int8_t status)
  {
    sensor_msgs::msg::NavSatFix msg;
    msg.status.status = status;
    msg.latitude = latitude;
    msg.longitude = longitude;
    msg.altitude = altitude;
    publish_reliably<sensor_msgs::msg::NavSatFix>(gps_pub_, msg);
  }

  void publish_target(double latitude, double longitude, double altitude_m)
  {
    antenna_tracker_msgs::msg::TargetGPS msg;
    msg.latitude = latitude;
    msg.longitude = longitude;
    msg.altitude_m = altitude_m;
    publish_reliably<antenna_tracker_msgs::msg::TargetGPS>(target_pub_, msg);
  }

  void publish_mode(uint8_t mode)
  {
    std_msgs::msg::UInt8 msg;
    msg.data = mode;
    publish_reliably<std_msgs::msg::UInt8>(mode_pub_, msg);
  }

  bool wait_for_targets(std::chrono::milliseconds timeout = 500ms)
  {
    return spin_until(
      executor_,
      [this]() { return azimuth_received_ && elevation_received_; },
      timeout);
  }

  std::shared_ptr<NavigationNode> nav_node_;
  std::shared_ptr<rclcpp::Node> io_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<antenna_tracker_msgs::msg::TargetGPS>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr az_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr el_sub_;

  bool azimuth_received_{false};
  bool elevation_received_{false};
  double last_azimuth_{0.0};
  double last_elevation_{0.0};
};

}  // namespace

/* === haversine_distance ================================================= */

TEST(NavigationMathTest, HaversineDistanceSamePoint) {
  double dist = NavigationNode::haversine_distance(37.5665, 126.9780, 37.5665, 126.9780);
  EXPECT_DOUBLE_EQ(dist, 0.0);
}

TEST(NavigationMathTest, HaversineDistance) {
  // Seoul → Busan ≈ 325 km
  double dist = NavigationNode::haversine_distance(37.5665, 126.9780, 35.1796, 129.0756);
  EXPECT_NEAR(dist, 325000.0, 10000.0);
}

TEST(NavigationMathTest, HaversineDistanceOneDegreeLatitude) {
  // 1° latitude ≈ 111.195 km
  double dist = NavigationNode::haversine_distance(0.0, 0.0, 1.0, 0.0);
  EXPECT_NEAR(dist, 111195.0, 1000.0);
}

TEST(NavigationMathTest, HaversineDistanceSymmetric) {
  double d1 = NavigationNode::haversine_distance(10.0, 20.0, 15.0, 25.0);
  double d2 = NavigationNode::haversine_distance(15.0, 25.0, 10.0, 20.0);
  EXPECT_NEAR(d1, d2, 1e-6);
}

/* === haversine_bearing ================================================== */

TEST(NavigationMathTest, BearingNorth) {
  // Same longitude, target north → bearing ≈ 0°
  double b = NavigationNode::haversine_bearing(0.0, 0.0, 10.0, 0.0);
  EXPECT_NEAR(b, 0.0, 1.0);
}

TEST(NavigationMathTest, BearingSouth) {
  // Same longitude, target south → bearing ≈ 180°
  double b = NavigationNode::haversine_bearing(10.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(b, 180.0, 1.0);
}

TEST(NavigationMathTest, BearingEast) {
  // Same latitude at equator, target east → bearing ≈ 90°
  double b = NavigationNode::haversine_bearing(0.0, 0.0, 0.0, 10.0);
  EXPECT_NEAR(b, 90.0, 1.0);
}

TEST(NavigationMathTest, BearingWest) {
  // Same latitude at equator, target west → bearing ≈ 270°
  double b = NavigationNode::haversine_bearing(0.0, 0.0, 0.0, -10.0);
  EXPECT_NEAR(b, 270.0, 1.0);
}

TEST(NavigationMathTest, BearingInRange) {
  // Any bearing result must be in [0, 360)
  double b = NavigationNode::haversine_bearing(37.5, 127.0, 35.2, 129.1);
  EXPECT_GE(b, 0.0);
  EXPECT_LT(b, 360.0);
}

TEST(NavigationMathTest, BearingNegativeWrapsPositive) {
  // atan2 returns negative for westward-southward direction → bearing += 360
  double b = NavigationNode::haversine_bearing(0.0, 10.0, 0.0, 0.0);
  EXPECT_GE(b, 0.0);
  EXPECT_LT(b, 360.0);
}

/* === elevation_angle (static methods only — avoids ROS node context) === */

TEST(NavigationMathTest, ElevationAngleSamePoint_AbovePositive) {
  double el = NavigationNode::elevation_angle(37.0, 127.0, 100.0, 37.0, 127.0, 200.0);
  EXPECT_NEAR(el, 90.0, 1e-6);
}

TEST(NavigationMathTest, ElevationAngleSamePoint_Below) {
  double el = NavigationNode::elevation_angle(37.0, 127.0, 200.0, 37.0, 127.0, 100.0);
  EXPECT_NEAR(el, -90.0, 1e-6);
}

TEST(NavigationMathTest, ElevationAngleSamePoint_SameAlt) {
  double el = NavigationNode::elevation_angle(37.0, 127.0, 100.0, 37.0, 127.0, 100.0);
  EXPECT_NEAR(el, 0.0, 1e-6);
}

TEST(NavigationMathTest, ElevationAngle) {
  // Same point, target 100m higher → 90°
  double el = NavigationNode::elevation_angle(37.0, 127.0, 100.0, 37.0, 127.0, 200.0);
  EXPECT_NEAR(el, 90.0, 1e-6);
}

TEST(NavigationMathTest, ElevationAngleTargetAbove) {
  // ~11km away (0.1° lat), 1000m higher → positive elevation < 90°
  double el = NavigationNode::elevation_angle(37.0, 127.0, 0.0, 37.1, 127.0, 1000.0);
  EXPECT_GT(el, 0.0);
  EXPECT_LT(el, 90.0);
}

TEST(NavigationMathTest, ElevationAngleTargetBelow) {
  // ~11km away, 1000m lower → negative elevation > -90°
  double el = NavigationNode::elevation_angle(37.0, 127.0, 1000.0, 37.1, 127.0, 0.0);
  EXPECT_LT(el, 0.0);
  EXPECT_GE(el, -90.0);
}

TEST(NavigationMathTest, ElevationAngleNearLevel) {
  // Far target at same altitude → ~0° elevation
  double el = NavigationNode::elevation_angle(0.0, 0.0, 100.0, 1.0, 0.0, 100.0);
  EXPECT_NEAR(el, 0.0, 5.0);
}

TEST(NavigationMathTest, ElevationClampedAboveNeg90) {
  // Steep downward case is clamped to -90
  double el = NavigationNode::elevation_angle(0.0, 0.0, 100000.0, 10.0, 0.0, 0.0);
  EXPECT_GE(el, -90.0);
}

TEST(NavigationMathTest, ElevationClampedBelow90) {
  // Steep upward case is clamped to +90
  double el = NavigationNode::elevation_angle(0.0, 0.0, 0.0, 0.01, 0.0, 100000.0);
  EXPECT_LE(el, 90.0);
}

TEST_F(NavigationNodeRosTest, AutoModePublishesTargetsForValidInputs) {
  publish_ground_fix(0.0, 0.0, 0.0, 0);
  publish_target(1.0, 0.0, 1000.0);

  ASSERT_TRUE(wait_for_targets());
  EXPECT_NEAR(last_azimuth_, 0.0, 1.0);
  EXPECT_GT(last_elevation_, 0.0);
}

TEST_F(NavigationNodeRosTest, InvalidGpsFixPreventsPublish) {
  publish_ground_fix(37.0, 127.0, 100.0, -1);
  publish_target(37.1, 127.0, 200.0);

  spin_for(300ms);
  EXPECT_FALSE(azimuth_received_);
  EXPECT_FALSE(elevation_received_);
}

TEST_F(NavigationNodeRosTest, InvalidTargetIsIgnored) {
  publish_ground_fix(37.0, 127.0, 100.0, 0);
  publish_target(std::numeric_limits<double>::quiet_NaN(), 127.1, 200.0);

  spin_for(300ms);
  EXPECT_FALSE(azimuth_received_);
  EXPECT_FALSE(elevation_received_);
}

TEST_F(NavigationNodeRosTest, ManualModeSuppressesPublish) {
  publish_ground_fix(37.0, 127.0, 100.0, 0);
  publish_mode(1);
  publish_target(37.1, 127.0, 200.0);

  spin_for(300ms);
  EXPECT_FALSE(azimuth_received_);
  EXPECT_FALSE(elevation_received_);
}

TEST_F(NavigationNodeRosTest, BelowHorizonElevationIsClampedToZero) {
  publish_ground_fix(37.0, 127.0, 1000.0, 0);
  publish_target(37.1, 127.0, 0.0);

  ASSERT_TRUE(wait_for_targets());
  EXPECT_GE(last_elevation_, 0.0);
  EXPECT_NEAR(last_elevation_, 0.0, 1e-6);
}

TEST_F(NavigationNodeRosTest, AutoModePublishesAgainAfterReturningFromManual) {
  publish_ground_fix(37.0, 127.0, 100.0, 0);
  publish_mode(1);
  publish_target(37.1, 127.0, 200.0);

  spin_for(200ms);
  EXPECT_FALSE(azimuth_received_);
  EXPECT_FALSE(elevation_received_);

  azimuth_received_ = false;
  elevation_received_ = false;
  publish_mode(0);
  publish_target(37.1, 127.0, 200.0);

  ASSERT_TRUE(wait_for_targets());
  EXPECT_NEAR(last_azimuth_, 0.0, 1.0);
  EXPECT_GT(last_elevation_, 0.0);
}

TEST_F(NavigationNodeRosTest, VerticalTargetPublishesClampedHighElevation) {
  publish_ground_fix(37.0, 127.0, 0.0, 0);
  publish_target(37.0, 127.0, 1000.0);

  ASSERT_TRUE(wait_for_targets());
  EXPECT_NEAR(last_elevation_, 90.0, 1e-6);
  EXPECT_GE(last_azimuth_, 0.0);
  EXPECT_LT(last_azimuth_, 360.0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
