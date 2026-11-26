#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;
using Velocity = geometry_msgs::msg::TwistStamped;
using Agreement = diagnostic_msgs::msg::DiagnosticArray;
using GateStatus = std_msgs::msg::String;

class LocalizationGate : public rclcpp::Node
{
public:
  LocalizationGate()
  : Node("localization_gate")
  {
    output_ = create_publisher<Velocity>("cmd_vel_guarded", 10);
    status_ = create_publisher<GateStatus>("localization/gate_state", rclcpp::QoS(1).transient_local());
    agreement_ = create_subscription<Agreement>(
      "localization/scan_agreement", 10, [this](const Agreement & report) {observe(report);});
    input_ = create_subscription<Velocity>(
      "cmd_vel_guard_in", 10, [this](const Velocity & command) {forward(command);});
    timer_ = create_wall_timer(50ms, [this] {check_and_publish();});
  }

private:
  enum class State {waiting, open, hold};

  void observe(const Agreement & report)
  {
    check_health();
    if (state_ == State::hold || report.status.size() != 1) {
      return;
    }
    std::string health;
    std::string latched;
    for (const auto & value : report.status.front().values) {
      if (value.key == "health_state") {
        health = value.value;
      } else if (value.key == "loss_latched") {
        latched = value.value;
      }
    }
    if (health == "lost" || latched == "true") {
      hold("localization_lost");
      return;
    }
    if (health == "unavailable" || health.empty() || latched != "false") {
      if (state_ == State::open) {
        hold("localization_unavailable");
      }
      return;
    }
    if (health != "tracking" && (state_ != State::open || health != "suspect")) {
      return;
    }
    if (!fresh(report.header.stamp, 0.5)) {
      return;
    }
    const auto stamp = rclcpp::Time(report.header.stamp).nanoseconds();
    if (last_health_stamp_ && stamp <= *last_health_stamp_) {
      return;
    }
    last_health_stamp_ = stamp;
    last_health_ = SteadyClock::now();
    if (state_ == State::waiting) {
      state_ = State::open;
      publish_state();
    }
  }

  bool fresh(const builtin_interfaces::msg::Time & timestamp, double max_age)
  {
    if (timestamp.sec < 0 || timestamp.nanosec >= 1'000'000'000) {
      return false;
    }
    const rclcpp::Time stamp(timestamp);
    const double age = (now() - stamp).seconds();
    return stamp.nanoseconds() > 0 && age >= -0.05 && age <= max_age;
  }

  void forward(const Velocity & command)
  {
    check_health();
    if (state_ != State::open || !fresh(command.header.stamp, 0.5)) {
      return;
    }
    const auto & linear = command.twist.linear;
    const auto & angular = command.twist.angular;
    if (!std::isfinite(linear.x) || !std::isfinite(linear.y) || !std::isfinite(linear.z) ||
      !std::isfinite(angular.x) || !std::isfinite(angular.y) || !std::isfinite(angular.z))
    {
      publish_zero();
      return;
    }
    last_command_ = SteadyClock::now();
    output_->publish(command);
  }

  void check_health()
  {
    if (state_ == State::open && SteadyClock::now() - last_health_ >= 750ms) {
      hold("localization_stale");
    }
  }

  void hold(const std::string & reason)
  {
    state_ = State::hold;
    reason_ = reason;
    publish_zero();
    publish_state();
    RCLCPP_WARN(get_logger(), "Velocity gate holding (%s)", reason_.c_str());
  }

  void check_and_publish()
  {
    check_health();
    if (state_ != State::open || SteadyClock::now() - last_command_ >= 500ms) {
      publish_zero();
    }
    publish_state();
  }

  void publish_zero()
  {
    Velocity command;
    command.header.stamp = now();
    command.header.frame_id = "base_footprint";
    output_->publish(command);
  }

  void publish_state()
  {
    GateStatus status;
    switch (state_) {
      case State::waiting: status.data = "WAITING"; break;
      case State::open: status.data = "OPEN"; break;
      case State::hold: status.data = "HOLD " + reason_; break;
    }
    status_->publish(status);
  }

  State state_{State::waiting};
  std::string reason_;
  std::optional<std::int64_t> last_health_stamp_;
  SteadyClock::time_point last_health_{};
  SteadyClock::time_point last_command_{};
  rclcpp::Publisher<Velocity>::SharedPtr output_;
  rclcpp::Publisher<GateStatus>::SharedPtr status_;
  rclcpp::Subscription<Agreement>::SharedPtr agreement_;
  rclcpp::Subscription<Velocity>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationGate>());
  rclcpp::shutdown();
}
