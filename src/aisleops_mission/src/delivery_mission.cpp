#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;
using Navigate = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;
using Odometry = nav_msgs::msg::Odometry;
using Trigger = std_srvs::srv::Trigger;

volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
  stop_requested = 1;
}

struct Leg
{
  std::string name;
  std::vector<double> pose;
};

class DeliveryMission
{
public:
  explicit DeliveryMission(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    navigator_(rclcpp_action::create_client<Navigate>(node_, "navigate_to_pose")),
    lifecycle_(node_->create_client<Trigger>("lifecycle_manager_nav2/is_active")),
    status_(node_->create_publisher<std_msgs::msg::String>(
        "mission_status", rclcpp::QoS(10).transient_local())),
    guarded_(node_->declare_parameter<bool>("guard_localization", false))
  {
    odom_subscription_ = node_->create_subscription<Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](Odometry::ConstSharedPtr message) {odometry_ = message;});
    if (guarded_) {
      gate_subscription_ = node_->create_subscription<std_msgs::msg::String>(
        "localization/gate_state", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String & message) {observe_gate(message.data);});
    }
    executor_.add_node(node_);
  }

  bool run()
  {
    const std::array<Leg, 3> route{{
      load_leg("receive", {0.0, -3.5, 0.0}),
      load_leg("deliver", {9.0, 3.5, 0.0}),
      load_leg("return", {0.0, 0.0, 0.0}),
    }};
    const auto timeout = node_->declare_parameter<int>("leg_timeout_sec", 75);
    if (timeout <= 0) {
      throw std::invalid_argument("leg_timeout_sec must be positive");
    }
    if (!wait_for_nav2()) {
      if (hold_reason_) {
        return hold(route.front().name);
      }
      return fail("Nav2 or the localization gate did not become ready with a running clock");
    }
    gate_armed_ = guarded_;
    for (const auto & leg : route) {
      executor_.spin_some();
      if (interrupted() || !run_leg(leg, std::chrono::seconds(timeout))) {
        if (hold_reason_) {
          return hold(leg.name);
        }
        return fail("Mission halted at " + leg.name);
      }
    }
    announce("COMPLETED");
    return true;
  }

private:
  void observe_gate(const std::string & state)
  {
    gate_received_ = SteadyClock::now();
    gate_open_ = state == "OPEN";
    if (!hold_reason_) {
      if (state.rfind("HOLD ", 0) == 0) {
        hold_reason_ = state.substr(5);
      } else if (gate_armed_ && !gate_open_) {
        hold_reason_ = "gate_closed";
      }
    }
  }

  bool gate_ready() const
  {
    return !guarded_ || (gate_open_ && SteadyClock::now() - gate_received_ < 500ms);
  }

  bool interrupted()
  {
    if (gate_armed_ && !hold_reason_ && !gate_ready()) {
      hold_reason_ = "gate_status_timeout";
    }
    return stop_requested || hold_reason_.has_value();
  }

  bool hold(const std::string & leg)
  {
    announce("HOLD " + leg);
    RCLCPP_WARN(node_->get_logger(), "Pending leg %s held (%s)", leg.c_str(), hold_reason_->c_str());
    while (rclcpp::ok() && !stop_requested) {
      executor_.spin_some();
      std::this_thread::sleep_for(50ms);
    }
    return false;
  }

  Leg load_leg(const std::string & name, const std::vector<double> & defaults)
  {
    auto pose = node_->declare_parameter<std::vector<double>>(name + "_pose", defaults);
    if (pose.size() != 3 ||
      !std::all_of(pose.begin(), pose.end(), [](double value) {return std::isfinite(value);}))
    {
      throw std::invalid_argument(name + "_pose must contain three finite numbers [x, y, yaw]");
    }
    return {name, pose};
  }

  void announce(const std::string & event)
  {
    std_msgs::msg::String status;
    status.data = event;
    status_->publish(status);
    RCLCPP_INFO(node_->get_logger(), "%s", event.c_str());
  }

  bool fail(const std::string & reason)
  {
    announce("FAILED " + reason);
    return false;
  }

  template<typename Future>
  bool await(Future & future, SteadyClock::time_point deadline, bool allow_interrupt = true)
  {
    while (rclcpp::ok()) {
      if (future.wait_for(0s) == std::future_status::ready) {
        return true;
      }
      const auto remaining = deadline - SteadyClock::now();
      if (remaining <= 0s || (allow_interrupt && interrupted())) {
        return false;
      }
      const auto interval = std::min(remaining, SteadyClock::duration(100ms));
      if (executor_.spin_until_future_complete(future, interval) == rclcpp::FutureReturnCode::SUCCESS) {
        return true;
      }
    }
    return false;
  }

  bool wait_for_nav2()
  {
    const auto deadline = SteadyClock::now() + 35s;
    while (rclcpp::ok() && !interrupted() && SteadyClock::now() < deadline) {
      executor_.spin_some();
      if (!interrupted() && gate_ready() &&
        node_->get_clock()->ros_time_is_active() && node_->now().nanoseconds() > 0 &&
        odometry_ && navigator_->action_server_is_ready() && lifecycle_->service_is_ready())
      {
        auto response = lifecycle_->async_send_request(std::make_shared<Trigger::Request>());
        if (!await(response, deadline)) {
          lifecycle_->remove_pending_request(response);
          return false;
        }
        if (response.get()->success && gate_ready() && !interrupted()) {
          return true;
        }
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  bool run_leg(const Leg & leg, std::chrono::seconds timeout)
  {
    Navigate::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->now();
    goal.pose.pose.position.x = leg.pose[0];
    goal.pose.pose.position.y = leg.pose[1];
    goal.pose.pose.orientation.z = std::sin(leg.pose[2] / 2);
    goal.pose.pose.orientation.w = std::cos(leg.pose[2] / 2);

    auto accepted = navigator_->async_send_goal(goal);
    // Keep receiving the reply after an interrupt so an accepted goal can still be canceled.
    if (!await(accepted, SteadyClock::now() + 5s, false)) {
      RCLCPP_ERROR(node_->get_logger(), "No goal acknowledgement for %s", leg.name.c_str());
      return false;
    }
    const auto handle = accepted.get();
    if (!handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal rejected for %s", leg.name.c_str());
      return false;
    }
    announce("STARTED " + leg.name);
    auto result = navigator_->async_get_result(handle);
    if (interrupted() || !await(result, SteadyClock::now() + timeout)) {
      RCLCPP_WARN(
        node_->get_logger(), "%s at %s. Canceling the current goal.",
        hold_reason_ ? hold_reason_->c_str() : (stop_requested ? "Interrupted" : "Deadline expired"),
        leg.name.c_str());
      cancel(handle, result);
      return false;
    }
    const auto outcome = result.get();
    if (outcome.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !outcome.result || outcome.result->error_code != Navigate::Result::NONE)
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Nav2 failed at %s (result=%d, error=%d)",
        leg.name.c_str(), static_cast<int>(outcome.code),
        outcome.result ? outcome.result->error_code : -1);
      return false;
    }
    if (!wait_until_stopped() || interrupted()) {
      return false;
    }
    announce("REACHED " + leg.name);
    return true;
  }

  void cancel(const GoalHandle::SharedPtr & handle, std::shared_future<GoalHandle::WrappedResult> & result)
  {
    if (result.wait_for(0s) == std::future_status::ready) {
      return;
    }
    auto cancellation = navigator_->async_cancel_goal(handle);
    if (await(cancellation, SteadyClock::now() + 3s, false) &&
      !cancellation.get()->goals_canceling.empty())
    {
      RCLCPP_INFO(node_->get_logger(), "Nav2 acknowledged cancellation");
    }
    if (await(result, SteadyClock::now() + 4s, false)) {
      const auto outcome = result.get();
      RCLCPP_INFO(
        node_->get_logger(), "Goal terminal result after cancellation: %d",
        static_cast<int>(outcome.code));
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Nav2 did not report a terminal result");
    }
  }

  bool wait_until_stopped()
  {
    const auto after_result = node_->now();
    const auto deadline = SteadyClock::now() + 6s;
    std::optional<rclcpp::Time> stopped_since;
    while (rclcpp::ok() && !interrupted() && SteadyClock::now() < deadline) {
      executor_.spin_some();
      if (interrupted()) {
        return false;
      }
      if (odometry_) {
        const auto stamp = rclcpp::Time(odometry_->header.stamp);
        const auto age = (node_->now() - stamp).seconds();
        const auto & twist = odometry_->twist.twist;
        if (stamp >= after_result && age >= 0.0 && age < 0.25 &&
          std::hypot(twist.linear.x, twist.linear.y) < 0.02 &&
          std::abs(twist.angular.z) < 0.03)
        {
          if (!stopped_since) {
            stopped_since = stamp;
          }
          if ((stamp - *stopped_since).seconds() >= 0.30) {
            return true;
          }
        } else {
          stopped_since.reset();
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    RCLCPP_ERROR(node_->get_logger(), "No settled odometry after the goal result");
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp_action::Client<Navigate>::SharedPtr navigator_;
  rclcpp::Client<Trigger>::SharedPtr lifecycle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  const bool guarded_;
  bool gate_armed_{false};
  bool gate_open_{false};
  SteadyClock::time_point gate_received_{};
  std::optional<std::string> hold_reason_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gate_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_subscription_;
  Odometry::ConstSharedPtr odometry_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  const auto node = std::make_shared<rclcpp::Node>("delivery_mission");
  int code = 1;
  try {
    DeliveryMission mission(node);
    code = mission.run() ? 0 : 1;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
  }
  rclcpp::shutdown();
  return stop_requested ? 130 : code;
}
