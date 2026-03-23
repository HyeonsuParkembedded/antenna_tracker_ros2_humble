#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "antenna_tracker_controller/state_machine_node.hpp"

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

class StateMachineNodeRosTest : public ::testing::Test
{
protected:
  virtual rclcpp::NodeOptions make_node_options() const
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("imu_timeout_sec", 1.0),
      rclcpp::Parameter("lora_timeout_sec", 0.5),
      rclcpp::Parameter("diagnostics_rate_hz", 50.0),
    });
    return options;
  }

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
    auto options = make_node_options();
    sm_node_ = std::make_shared<StateMachineNode>(options);
    io_node_ = std::make_shared<rclcpp::Node>("state_machine_test_io");

    state_pub_ = io_node_->create_publisher<antenna_tracker_msgs::msg::AntennaState>(
      "/antenna/state", 10);
    fusion_pub_ = io_node_->create_publisher<antenna_tracker_msgs::msg::ImuFusion>(
      "/antenna/imu_fusion", rclcpp::SensorDataQoS());
    encoder_pub_ = io_node_->create_publisher<antenna_tracker_msgs::msg::EncoderFeedback>(
      "/antenna/encoder_feedback", rclcpp::SensorDataQoS());
    target_pub_ = io_node_->create_publisher<antenna_tracker_msgs::msg::TargetGPS>(
      "/antenna/target_gps", rclcpp::SensorDataQoS());
    heartbeat_pub_ = io_node_->create_publisher<antenna_tracker_msgs::msg::Heartbeat>(
      "/antenna/heartbeat", 10);

    mode_sub_ = io_node_->create_subscription<std_msgs::msg::UInt8>(
      "/antenna/mode", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        last_mode_ = msg->data;
        mode_received_ = true;
      });
    target_az_sub_ = io_node_->create_subscription<std_msgs::msg::Float64>(
      "/antenna/target_azimuth", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        last_target_az_ = msg->data;
        target_az_received_ = true;
      });
    target_el_sub_ = io_node_->create_subscription<std_msgs::msg::Float64>(
      "/antenna/target_elevation", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        last_target_el_ = msg->data;
        target_el_received_ = true;
      });
    diagnostics_sub_ = io_node_->create_subscription<antenna_tracker_msgs::msg::TrackerDiagnostics>(
      "/antenna/diagnostics", 10,
      [this](const antenna_tracker_msgs::msg::TrackerDiagnostics::SharedPtr msg) {
        last_diagnostics_ = *msg;
        diagnostics_received_ = true;
      });

    set_mode_client_ = io_node_->create_client<antenna_tracker_msgs::srv::SetMode>(
      "/antenna/set_mode");
    get_status_client_ = io_node_->create_client<antenna_tracker_msgs::srv::GetStatus>(
      "/antenna/get_status");
    set_manual_target_client_ =
      io_node_->create_client<antenna_tracker_msgs::srv::SetManualTarget>(
      "/antenna/set_manual_target");

    executor_.add_node(sm_node_);
    executor_.add_node(io_node_);

    ASSERT_TRUE(wait_for_service(set_mode_client_));
    ASSERT_TRUE(wait_for_service(get_status_client_));
    ASSERT_TRUE(wait_for_service(set_manual_target_client_));
  }

  void TearDown() override
  {
    if (sm_node_) {
      executor_.remove_node(sm_node_);
      sm_node_.reset();
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

  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr call_service(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const typename ServiceT::Request::SharedPtr & request,
    std::chrono::milliseconds timeout = 700ms)
  {
    auto future = client->async_send_request(request);
    const bool ready = spin_until(
      executor_,
      [&future]() {
        return future.wait_for(0ms) == std::future_status::ready;
      },
      timeout);

    if (!ready) {
      return nullptr;
    }
    return future.get();
  }

  bool wait_for_service(
    const rclcpp::ClientBase::SharedPtr & client,
    std::chrono::milliseconds timeout = 700ms)
  {
    return spin_until(
      executor_,
      [&client]() {
        return client->service_is_ready();
      },
      timeout);
  }

  bool wait_for_mode(uint8_t expected_mode, std::chrono::milliseconds timeout = 700ms)
  {
    return spin_until(
      executor_,
      [this, expected_mode]() {
        return mode_received_ && last_mode_ == expected_mode;
      },
      timeout);
  }

  bool wait_for_manual_targets(std::chrono::milliseconds timeout = 700ms)
  {
    return spin_until(
      executor_,
      [this]() {
        return target_az_received_ && target_el_received_;
      },
      timeout);
  }

  void publish_state(double azimuth_deg, double elevation_deg)
  {
    antenna_tracker_msgs::msg::AntennaState msg;
    msg.current_azimuth = azimuth_deg;
    msg.current_elevation = elevation_deg;
    publish_reliably(state_pub_, msg);
  }

  void publish_fusion()
  {
    antenna_tracker_msgs::msg::ImuFusion msg;
    msg.kalman_valid = true;
    publish_reliably(fusion_pub_, msg);
  }

  void publish_encoder()
  {
    antenna_tracker_msgs::msg::EncoderFeedback msg;
    msg.az_valid = true;
    msg.el_valid = true;
    publish_reliably(encoder_pub_, msg);
  }

  void publish_target_gps()
  {
    antenna_tracker_msgs::msg::TargetGPS msg;
    msg.latitude = 37.1;
    msg.longitude = 127.1;
    msg.altitude_m = 100.0;
    publish_reliably(target_pub_, msg);
  }

  void publish_heartbeat(uint8_t status = 1)
  {
    antenna_tracker_msgs::msg::Heartbeat msg;
    msg.status = status;
    publish_reliably(heartbeat_pub_, msg);
  }

  void reset_observers()
  {
    mode_received_ = false;
    target_az_received_ = false;
    target_el_received_ = false;
    diagnostics_received_ = false;
  }

  std::shared_ptr<StateMachineNode> sm_node_;
  std::shared_ptr<rclcpp::Node> io_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  rclcpp::Publisher<antenna_tracker_msgs::msg::AntennaState>::SharedPtr state_pub_;
  rclcpp::Publisher<antenna_tracker_msgs::msg::ImuFusion>::SharedPtr fusion_pub_;
  rclcpp::Publisher<antenna_tracker_msgs::msg::EncoderFeedback>::SharedPtr encoder_pub_;
  rclcpp::Publisher<antenna_tracker_msgs::msg::TargetGPS>::SharedPtr target_pub_;
  rclcpp::Publisher<antenna_tracker_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_az_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_el_sub_;
  rclcpp::Subscription<antenna_tracker_msgs::msg::TrackerDiagnostics>::SharedPtr diagnostics_sub_;

  rclcpp::Client<antenna_tracker_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<antenna_tracker_msgs::srv::GetStatus>::SharedPtr get_status_client_;
  rclcpp::Client<antenna_tracker_msgs::srv::SetManualTarget>::SharedPtr set_manual_target_client_;

  bool mode_received_{false};
  bool target_az_received_{false};
  bool target_el_received_{false};
  bool diagnostics_received_{false};
  uint8_t last_mode_{0};
  double last_target_az_{0.0};
  double last_target_el_{0.0};
  antenna_tracker_msgs::msg::TrackerDiagnostics last_diagnostics_;
};

class StateMachineNodeFastTimeoutRosTest : public StateMachineNodeRosTest
{
protected:
  rclcpp::NodeOptions make_node_options() const override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("imu_timeout_sec", 0.12),
      rclcpp::Parameter("lora_timeout_sec", 0.5),
      rclcpp::Parameter("diagnostics_rate_hz", 50.0),
    });
    return options;
  }
};

}  // namespace

TEST_F(StateMachineNodeRosTest, ModeTransitionsAndStatusAreConsistent)
{
  auto set_manual = std::make_shared<antenna_tracker_msgs::srv::SetMode::Request>();
  set_manual->mode = static_cast<uint8_t>(OperationMode::MANUAL);
  auto manual_response =
    call_service<antenna_tracker_msgs::srv::SetMode>(set_mode_client_, set_manual);
  ASSERT_NE(manual_response, nullptr);
  ASSERT_TRUE(manual_response->success);
  EXPECT_EQ(manual_response->previous_mode, static_cast<uint8_t>(OperationMode::STANDBY));
  ASSERT_TRUE(wait_for_mode(static_cast<uint8_t>(OperationMode::MANUAL)));

  auto get_status = std::make_shared<antenna_tracker_msgs::srv::GetStatus::Request>();
  auto status_response =
    call_service<antenna_tracker_msgs::srv::GetStatus>(get_status_client_, get_status);
  ASSERT_NE(status_response, nullptr);
  EXPECT_EQ(status_response->current_mode, static_cast<uint8_t>(OperationMode::MANUAL));
  EXPECT_FALSE(status_response->tracking_active);

  reset_observers();
  auto set_auto = std::make_shared<antenna_tracker_msgs::srv::SetMode::Request>();
  set_auto->mode = static_cast<uint8_t>(OperationMode::AUTO);
  auto auto_response = call_service<antenna_tracker_msgs::srv::SetMode>(set_mode_client_, set_auto);
  ASSERT_NE(auto_response, nullptr);
  ASSERT_TRUE(auto_response->success);
  ASSERT_TRUE(wait_for_mode(static_cast<uint8_t>(OperationMode::AUTO)));

  status_response =
    call_service<antenna_tracker_msgs::srv::GetStatus>(get_status_client_, get_status);
  ASSERT_NE(status_response, nullptr);
  EXPECT_EQ(status_response->current_mode, static_cast<uint8_t>(OperationMode::AUTO));
  EXPECT_TRUE(status_response->tracking_active);
}

TEST_F(StateMachineNodeRosTest, EmergencyRequiresStandbyToExit)
{
  auto set_emergency = std::make_shared<antenna_tracker_msgs::srv::SetMode::Request>();
  set_emergency->mode = static_cast<uint8_t>(OperationMode::EMERGENCY);
  auto emergency_response =
    call_service<antenna_tracker_msgs::srv::SetMode>(set_mode_client_, set_emergency);
  ASSERT_NE(emergency_response, nullptr);
  ASSERT_TRUE(emergency_response->success);
  ASSERT_TRUE(wait_for_mode(static_cast<uint8_t>(OperationMode::EMERGENCY)));

  auto set_auto = std::make_shared<antenna_tracker_msgs::srv::SetMode::Request>();
  set_auto->mode = static_cast<uint8_t>(OperationMode::AUTO);
  auto auto_response = call_service<antenna_tracker_msgs::srv::SetMode>(set_mode_client_, set_auto);
  ASSERT_NE(auto_response, nullptr);
  EXPECT_FALSE(auto_response->success);
  EXPECT_EQ(auto_response->previous_mode, static_cast<uint8_t>(OperationMode::EMERGENCY));
  EXPECT_NE(auto_response->message.find("STANDBY"), std::string::npos);

  reset_observers();
  auto set_standby = std::make_shared<antenna_tracker_msgs::srv::SetMode::Request>();
  set_standby->mode = static_cast<uint8_t>(OperationMode::STANDBY);
  auto standby_response =
    call_service<antenna_tracker_msgs::srv::SetMode>(set_mode_client_, set_standby);
  ASSERT_NE(standby_response, nullptr);
  ASSERT_TRUE(standby_response->success);
  ASSERT_TRUE(wait_for_mode(static_cast<uint8_t>(OperationMode::STANDBY)));
}

TEST_F(StateMachineNodeRosTest, ManualTargetServiceRequiresManualModeAndPublishesTargets)
{
  auto request = std::make_shared<antenna_tracker_msgs::srv::SetManualTarget::Request>();
  request->azimuth_deg = 359.0;
  request->elevation_deg = 90.0;

  auto denied_response = call_service<antenna_tracker_msgs::srv::SetManualTarget>(
    set_manual_target_client_, request);
  ASSERT_NE(denied_response, nullptr);
  EXPECT_FALSE(denied_response->success);
  EXPECT_NE(denied_response->message.find("MANUAL"), std::string::npos);

  auto set_manual = std::make_shared<antenna_tracker_msgs::srv::SetMode::Request>();
  set_manual->mode = static_cast<uint8_t>(OperationMode::MANUAL);
  auto manual_response =
    call_service<antenna_tracker_msgs::srv::SetMode>(set_mode_client_, set_manual);
  ASSERT_NE(manual_response, nullptr);
  ASSERT_TRUE(manual_response->success);

  reset_observers();
  auto publish_response = call_service<antenna_tracker_msgs::srv::SetManualTarget>(
    set_manual_target_client_, request);
  ASSERT_NE(publish_response, nullptr);
  ASSERT_TRUE(publish_response->success);
  ASSERT_TRUE(wait_for_manual_targets());
  EXPECT_NEAR(last_target_az_, 359.0, 1e-6);
  EXPECT_NEAR(last_target_el_, 90.0, 1e-6);

  auto out_of_range = std::make_shared<antenna_tracker_msgs::srv::SetManualTarget::Request>();
  out_of_range->azimuth_deg = 361.0;
  out_of_range->elevation_deg = 45.0;
  auto invalid_response = call_service<antenna_tracker_msgs::srv::SetManualTarget>(
    set_manual_target_client_, out_of_range);
  ASSERT_NE(invalid_response, nullptr);
  EXPECT_FALSE(invalid_response->success);
  EXPECT_NE(invalid_response->message.find("range"), std::string::npos);
}

TEST_F(StateMachineNodeRosTest, DiagnosticsReflectRecentInputs)
{
  publish_fusion();
  publish_encoder();
  publish_target_gps();
  publish_heartbeat();

  ASSERT_TRUE(spin_until(
    executor_,
    [this]() {
      return diagnostics_received_ && last_diagnostics_.imu_ok &&
             last_diagnostics_.encoder_ok && last_diagnostics_.can_ok;
    },
    700ms));
  EXPECT_TRUE(last_diagnostics_.imu_ok);
  EXPECT_TRUE(last_diagnostics_.encoder_ok);
  EXPECT_TRUE(last_diagnostics_.can_ok);
}

TEST_F(StateMachineNodeFastTimeoutRosTest, ImuTimeoutTriggersEmergencyMode)
{
  auto set_manual = std::make_shared<antenna_tracker_msgs::srv::SetMode::Request>();
  set_manual->mode = static_cast<uint8_t>(OperationMode::MANUAL);
  auto manual_response =
    call_service<antenna_tracker_msgs::srv::SetMode>(set_mode_client_, set_manual);
  ASSERT_NE(manual_response, nullptr);
  ASSERT_TRUE(manual_response->success);

  reset_observers();
  ASSERT_TRUE(spin_until(
    executor_,
    [this]() {
      return mode_received_ &&
             last_mode_ == static_cast<uint8_t>(OperationMode::EMERGENCY) &&
             diagnostics_received_ && !last_diagnostics_.imu_ok;
    },
    700ms));

  auto get_status = std::make_shared<antenna_tracker_msgs::srv::GetStatus::Request>();
  auto status_response =
    call_service<antenna_tracker_msgs::srv::GetStatus>(get_status_client_, get_status);
  ASSERT_NE(status_response, nullptr);
  EXPECT_EQ(status_response->current_mode, static_cast<uint8_t>(OperationMode::EMERGENCY));
  EXPECT_FALSE(status_response->tracking_active);
}

TEST_F(StateMachineNodeRosTest, StateFeedbackIsReturnedByStatusService)
{
  publish_state(123.4, 56.7);

  auto get_status = std::make_shared<antenna_tracker_msgs::srv::GetStatus::Request>();
  auto status_response =
    call_service<antenna_tracker_msgs::srv::GetStatus>(get_status_client_, get_status);
  ASSERT_NE(status_response, nullptr);
  EXPECT_NEAR(status_response->current_azimuth, 123.4, 1e-6);
  EXPECT_NEAR(status_response->current_elevation, 56.7, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
