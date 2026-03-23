#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include <rclcpp_action/rclcpp_action.hpp>

#include "antenna_tracker_controller/controller_node.hpp"

using namespace antenna_tracker_controller;

namespace
{

using ClientGoalHandleTrackTarget = rclcpp_action::ClientGoalHandle<TrackTarget>;
using namespace std::chrono_literals;

template<typename PredicateT>
bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor,
  PredicateT predicate,
  std::chrono::milliseconds timeout = 500ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
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

class ControllerNodeRosTest : public ::testing::Test
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
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("loop_rate_hz", 50.0),
    });

    controller_node_ = std::make_shared<ControllerNode>(options);
    io_node_ = std::make_shared<rclcpp::Node>("controller_node_test_io");

    fusion_pub_ = io_node_->create_publisher<antenna_tracker_msgs::msg::ImuFusion>(
      "/antenna/imu_fusion", rclcpp::SensorDataQoS());
    encoder_pub_ = io_node_->create_publisher<antenna_tracker_msgs::msg::EncoderFeedback>(
      "/antenna/encoder_feedback", rclcpp::SensorDataQoS());
    mode_pub_ = io_node_->create_publisher<std_msgs::msg::UInt8>("/antenna/mode", 10);
    target_az_pub_ = io_node_->create_publisher<std_msgs::msg::Float64>("/antenna/target_azimuth", 10);
    target_el_pub_ = io_node_->create_publisher<std_msgs::msg::Float64>("/antenna/target_elevation", 10);

    motor_sub_ = io_node_->create_subscription<antenna_tracker_msgs::msg::MotorCommand>(
      "/antenna/motor_cmd", rclcpp::SensorDataQoS(),
      [this](const antenna_tracker_msgs::msg::MotorCommand::SharedPtr msg) {
        last_motor_ = *msg;
        motor_received_ = true;
      });
    state_sub_ = io_node_->create_subscription<antenna_tracker_msgs::msg::AntennaState>(
      "/antenna/state", 10,
      [this](const antenna_tracker_msgs::msg::AntennaState::SharedPtr msg) {
        last_state_ = *msg;
        state_received_ = true;
      });

    action_client_ = rclcpp_action::create_client<TrackTarget>(io_node_, "/antenna/track_target");

    executor_.add_node(controller_node_);
    executor_.add_node(io_node_);

    ASSERT_TRUE(wait_for_action_server());
  }

  void TearDown() override
  {
    if (controller_node_) {
      executor_.remove_node(controller_node_);
      controller_node_.reset();
    }
    if (io_node_) {
      executor_.remove_node(io_node_);
      io_node_.reset();
    }
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  template<typename FutureT>
  bool wait_for_future(FutureT & future, std::chrono::milliseconds timeout = 1s)
  {
    return spin_until(
      executor_,
      [&future]() {
        return future.wait_for(0ms) == std::future_status::ready;
      },
      timeout);
  }

  bool wait_for_action_server(std::chrono::milliseconds timeout = 1s)
  {
    return spin_until(
      executor_,
      [this]() {
        return action_client_->action_server_is_ready();
      },
      timeout);
  }

  template<typename MessageT>
  void publish_reliably(
    const typename rclcpp::Publisher<MessageT>::SharedPtr & publisher,
    const MessageT & message,
    int repeats = 3)
  {
    for (int i = 0; i < repeats; ++i) {
      publisher->publish(message);
      spin_for(30ms);
    }
  }

  void publish_mode(uint8_t mode)
  {
    std_msgs::msg::UInt8 msg;
    msg.data = mode;
    publish_reliably(mode_pub_, msg);
  }

  void publish_targets(double azimuth_deg, double elevation_deg)
  {
    std_msgs::msg::Float64 az_msg;
    az_msg.data = azimuth_deg;
    publish_reliably(target_az_pub_, az_msg);

    std_msgs::msg::Float64 el_msg;
    el_msg.data = elevation_deg;
    publish_reliably(target_el_pub_, el_msg);
  }

  void publish_fusion(double azimuth_deg, double elevation_deg, bool kalman_valid)
  {
    antenna_tracker_msgs::msg::ImuFusion msg;
    msg.azimuth = azimuth_deg;
    msg.elevation = elevation_deg;
    msg.kalman_valid = kalman_valid;
    publish_reliably(fusion_pub_, msg);
  }

  void publish_encoder(double azimuth_deg, double elevation_deg, bool valid = true)
  {
    antenna_tracker_msgs::msg::EncoderFeedback msg;
    msg.az_angle_deg = azimuth_deg;
    msg.el_angle_deg = elevation_deg;
    msg.az_valid = valid;
    msg.el_valid = valid;
    publish_reliably(encoder_pub_, msg);
  }

  bool wait_for_motor_and_state(std::chrono::milliseconds timeout = 700ms)
  {
    return spin_until(
      executor_,
      [this]() {
        return motor_received_ && state_received_;
      },
      timeout);
  }

  void reset_observers()
  {
    motor_received_ = false;
    state_received_ = false;
  }

  std::shared_ptr<ControllerNode> controller_node_;
  std::shared_ptr<rclcpp::Node> io_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  rclcpp::Publisher<antenna_tracker_msgs::msg::ImuFusion>::SharedPtr fusion_pub_;
  rclcpp::Publisher<antenna_tracker_msgs::msg::EncoderFeedback>::SharedPtr encoder_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_az_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_el_pub_;

  rclcpp::Subscription<antenna_tracker_msgs::msg::MotorCommand>::SharedPtr motor_sub_;
  rclcpp::Subscription<antenna_tracker_msgs::msg::AntennaState>::SharedPtr state_sub_;

  rclcpp_action::Client<TrackTarget>::SharedPtr action_client_;

  bool motor_received_{false};
  bool state_received_{false};
  antenna_tracker_msgs::msg::MotorCommand last_motor_;
  antenna_tracker_msgs::msg::AntennaState last_state_;
};

}  // namespace

TEST_F(ControllerNodeRosTest, StandbyPublishesEmergencyStop)
{
  publish_encoder(12.0, 4.0);
  publish_targets(40.0, 15.0);
  publish_mode(2);

  ASSERT_TRUE(wait_for_motor_and_state());
  EXPECT_TRUE(last_motor_.emergency_stop);
  EXPECT_DOUBLE_EQ(last_motor_.az_frequency_hz, 0.0);
  EXPECT_DOUBLE_EQ(last_motor_.el_frequency_hz, 0.0);
  EXPECT_FALSE(last_state_.tracking_active);
  EXPECT_EQ(last_state_.mode, 2);
}

TEST_F(ControllerNodeRosTest, InvalidFusionInAutoSuppressesCommands)
{
  publish_fusion(0.0, 0.0, false);
  publish_targets(30.0, 12.0);
  publish_mode(0);

  ASSERT_TRUE(wait_for_motor_and_state());
  EXPECT_FALSE(last_motor_.emergency_stop);
  EXPECT_DOUBLE_EQ(last_motor_.az_frequency_hz, 0.0);
  EXPECT_DOUBLE_EQ(last_motor_.el_frequency_hz, 0.0);
  EXPECT_TRUE(last_state_.tracking_active);
}

TEST_F(ControllerNodeRosTest, ActionGoalSucceedsWhenTargetReached)
{
  publish_mode(0);
  publish_encoder(0.0, 0.0);

  bool feedback_received = false;
  rclcpp_action::Client<TrackTarget>::SendGoalOptions options;
  options.feedback_callback =
    [&feedback_received](
      ClientGoalHandleTrackTarget::SharedPtr,
      const std::shared_ptr<const TrackTarget::Feedback>) {
      feedback_received = true;
    };

  TrackTarget::Goal goal;
  goal.target_azimuth_deg = 18.0;
  goal.target_elevation_deg = 9.0;
  goal.tolerance_deg = 1.0;
  goal.timeout_sec = 1.5;

  auto goal_future = action_client_->async_send_goal(goal, options);
  ASSERT_TRUE(wait_for_future(goal_future));
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = action_client_->async_get_result(goal_handle);
  spin_for(150ms);
  publish_encoder(18.4, 8.7);

  ASSERT_TRUE(wait_for_future(result_future, 1500ms));
  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->success);
  EXPECT_TRUE(feedback_received);
}

TEST_F(ControllerNodeRosTest, ActionGoalTimesOutWhenTargetDoesNotMove)
{
  publish_mode(0);
  publish_encoder(0.0, 0.0);

  TrackTarget::Goal goal;
  goal.target_azimuth_deg = 90.0;
  goal.target_elevation_deg = 20.0;
  goal.tolerance_deg = 0.5;
  goal.timeout_sec = 0.2;

  auto goal_future = action_client_->async_send_goal(goal);
  ASSERT_TRUE(wait_for_future(goal_future));
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_TRUE(wait_for_future(result_future, 1500ms));
  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_FALSE(wrapped_result.result->success);
}

TEST_F(ControllerNodeRosTest, CancelingActionReturnsCanceledResult)
{
  publish_mode(0);
  publish_encoder(0.0, 0.0);

  TrackTarget::Goal goal;
  goal.target_azimuth_deg = 120.0;
  goal.target_elevation_deg = 30.0;
  goal.tolerance_deg = 0.5;
  goal.timeout_sec = 5.0;

  auto goal_future = action_client_->async_send_goal(goal);
  ASSERT_TRUE(wait_for_future(goal_future));
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = action_client_->async_get_result(goal_handle);
  spin_for(150ms);
  auto cancel_future = action_client_->async_cancel_goal(goal_handle);
  ASSERT_TRUE(wait_for_future(cancel_future, 1s));
  ASSERT_NE(cancel_future.get(), nullptr);

  ASSERT_TRUE(wait_for_future(result_future, 1500ms));
  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_FALSE(wrapped_result.result->success);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
