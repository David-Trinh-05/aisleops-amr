#ifndef AISLEOPS_MISSION__POSE_RECOVERY_HPP_
#define AISLEOPS_MISSION__POSE_RECOVERY_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/srv/set_initial_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "aisleops_localization/srv/search_pose.hpp"
#include "pose_evidence.hpp"

namespace aisleops
{
class PoseRecovery
{
  using Clock = std::chrono::steady_clock;
  using Search = aisleops_localization::srv::SearchPose;
  using SetPose = nav2_msgs::srv::SetInitialPose;
  using Empty = std_srvs::srv::Empty;
  using Estimate = geometry_msgs::msg::PoseWithCovarianceStamped;
  using SetClient = rclcpp::Client<SetPose>;
  using SearchClient = rclcpp::Client<Search>;
  using UpdateClient = rclcpp::Client<Empty>;
  enum class Phase {idle, searching, seeding, verifying, needs_help};

public:
  explicit PoseRecovery(rclcpp::Node & node)
  : node_(node), search_(node.create_client<Search>("localization/search_pose")),
    seed_(node.create_client<SetPose>("set_initial_pose")),
    update_(node.create_client<Empty>("request_nomotion_update"))
  {
    estimate_ = node.create_subscription<Estimate>(
      "amcl_pose", 10, [this](const Estimate & pose) {observe_estimate(pose);});
  }

  bool start()
  {
    using namespace std::chrono_literals;
    const auto stamp = node_.now().nanoseconds();
    if (phase_ != Phase::idle || stamp <= 0) {
      return false;
    }
    phase_ = Phase::searching;
    started_ = stamp;
    last_candidate_ = stamp;
    consensus_ = {};
    estimate_since_ = last_estimate_ = last_scan_ = 0;
    matches_ = 0;
    tracking_ = false;
    next_request_ = Clock::now();
    deadline_ = next_request_ + 12s;
    return true;
  }

  void tick()
  {
    using namespace std::chrono_literals;
    if (phase_ == Phase::idle || phase_ == Phase::needs_help) {
      return;
    }
    const auto time = Clock::now();
    if (time >= deadline_ || node_.now().nanoseconds() < started_) {
      fail(phase_ == Phase::searching ? "search_timeout" : "verification_timeout");
      return;
    }
    if (time < next_request_ || pending_) {
      return;
    }
    next_request_ = time + 140ms;
    switch (phase_) {
      case Phase::searching:
        if (search_->service_is_ready()) {
          pending_ = search_->async_send_request(
            std::make_shared<Search::Request>(),
            [this](SearchClient::SharedFuture result) {
              pending_.reset();
              observe_candidate(*result.get());
            }).request_id;
        }
        break;
      case Phase::seeding:
        if (seed_->service_is_ready()) {
          send_seed();
        }
        break;
      case Phase::verifying:
        if (update_->service_is_ready()) {
          pending_ = update_->async_send_request(
            std::make_shared<Empty::Request>(),
            [this](UpdateClient::SharedFuture) {pending_.reset();}).request_id;
        }
        break;
      default: break;
    }
  }

  void observe_scan(std::int64_t stamp, bool matching, bool tracking)
  {
    if (phase_ != Phase::verifying || estimate_since_ == 0 || stamp <= estimate_since_ ||
      stamp <= last_scan_)
    {
      return;
    }
    if (!matching || stamp - last_scan_ > 350'000'000) {
      matches_ = 0;
    }
    last_scan_ = stamp;
    scan_received_ = Clock::now();
    tracking_ = tracking;
    if (matching) {
      matches_ = std::min(matches_ + 1, 5);
    }
  }

  bool ready() const
  {
    using namespace std::chrono_literals;
    const auto time = Clock::now();
    const auto stamp = node_.now().nanoseconds();
    return phase_ == Phase::verifying && time < deadline_ && estimate_since_ > 0 &&
           tracking_ && matches_ >= 5 && time - scan_received_ < 400ms &&
           time - estimate_received_ < 500ms && stamp >= std::max(last_scan_, last_estimate_) &&
           stamp - last_scan_ < 400'000'000 && stamp - last_estimate_ < 500'000'000;
  }

  void finish()
  {
    clear_pending();
    phase_ = Phase::idle;
  }

  std::string status() const
  {
    switch (phase_) {
      case Phase::idle: return {};
      case Phase::searching: return "RECOVERING searching";
      case Phase::seeding: return "RECOVERING seeding";
      case Phase::verifying: return ready() ? "READY" : "RECOVERING verifying";
      case Phase::needs_help: return "NEEDS_HELP " + reason_;
    }
    return {};
  }

private:
  std::optional<std::int64_t> recent_stamp(const std_msgs::msg::Header & header) const
  {
    if (header.frame_id != "map" || header.stamp.sec < 0 ||
      header.stamp.nanosec >= 1'000'000'000)
    {
      return std::nullopt;
    }
    const auto stamp = rclcpp::Time(header.stamp).nanoseconds();
    const auto age = node_.now().nanoseconds() - stamp;
    return stamp > 0 && age >= 0 && age < 500'000'000 ? std::optional(stamp) : std::nullopt;
  }

  static std::optional<PlanarPose> planar(const geometry_msgs::msg::Pose & pose)
  {
    const auto & q = pose.orientation;
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !(std::abs(q.z * q.z + q.w * q.w - 1.0) < 1e-3) ||
      !(std::abs(q.x) < 1e-3 && std::abs(q.y) < 1e-3))
    {
      return std::nullopt;
    }
    return PlanarPose{pose.position.x, pose.position.y,
      std::atan2(2.0 * q.z * q.w, 1.0 - 2.0 * q.z * q.z)};
  }

  void observe_candidate(const Search::Response & result)
  {
    if (result.status == Search::Response::UNAVAILABLE) {
      return;
    }
    const auto stamp = recent_stamp(result.candidates.header);
    if (!stamp || *stamp <= last_candidate_) {
      return;
    }
    last_candidate_ = *stamp;
    if (result.status == Search::Response::AMBIGUOUS) {
      fail("ambiguous_pose");
      return;
    }
    if (result.status == Search::Response::NO_MATCH) {
      fail("no_matching_pose");
      return;
    }
    if (result.status != Search::Response::CANDIDATE || result.candidates.poses.empty()) {
      fail("invalid_search_result");
      return;
    }
    const auto & candidate = result.candidates.poses.front();
    const auto pose = planar(candidate);
    if (!pose) {
      fail("invalid_candidate");
      return;
    }
    switch (consensus_.observe(*stamp, *pose)) {
      case PoseConsensus::Result::inconsistent: fail("inconsistent_candidates"); break;
      case PoseConsensus::Result::collecting: break;
      case PoseConsensus::Result::ready:
        candidate_ = candidate;
        target_ = *pose;
        phase_ = Phase::seeding;
        break;
    }
  }

  void send_seed()
  {
    seed_stamp_ = node_.now().nanoseconds();
    if (seed_stamp_ < last_candidate_ || seed_stamp_ - last_candidate_ >= 500'000'000) {
      fail("candidate_expired");
      return;
    }
    auto request = std::make_shared<SetPose::Request>();
    request->pose.header.stamp = rclcpp::Time(last_candidate_, RCL_ROS_TIME);
    request->pose.header.frame_id = "map";
    request->pose.pose.pose = candidate_;
    request->pose.pose.covariance[0] = request->pose.pose.covariance[7] = 0.01;
    request->pose.pose.covariance[35] = 0.0064;
    pending_ = seed_->async_send_request(
      request, [this](SetClient::SharedFuture) {
        pending_.reset();
        phase_ = Phase::verifying;
      }).request_id;
  }

  void observe_estimate(const Estimate & estimate)
  {
    if (phase_ != Phase::verifying) {
      return;
    }
    const auto stamp = recent_stamp(estimate.header);
    if (!stamp || *stamp <= std::max(seed_stamp_, last_estimate_)) {
      return;
    }
    last_estimate_ = *stamp;
    const auto pose = planar(estimate.pose.pose);
    const auto & c = estimate.pose.covariance;
    if (!pose || !target_.near(*pose, 0.25, 0.20) ||
      !(c[0] >= 0 && c[0] <= 0.09 && c[7] >= 0 && c[7] <= 0.09 && c[35] >= 0 && c[35] <= 0.0625))
    {
      estimate_since_ = 0;
      matches_ = 0;
      tracking_ = false;
      return;
    }
    if (estimate_since_ == 0) {
      estimate_since_ = *stamp;
    }
    estimate_received_ = Clock::now();
  }

  void clear_pending()
  {
    if (!pending_) {
      return;
    }
    switch (phase_) {
      case Phase::searching: search_->remove_pending_request(*pending_); break;
      case Phase::seeding: seed_->remove_pending_request(*pending_); break;
      case Phase::verifying: update_->remove_pending_request(*pending_); break;
      default: break;
    }
    pending_.reset();
  }

  void fail(const std::string & reason)
  {
    clear_pending();
    phase_ = Phase::needs_help;
    reason_ = reason;
    RCLCPP_WARN(node_.get_logger(), "Pose recovery needs help (%s)", reason.c_str());
  }

  rclcpp::Node & node_;
  SearchClient::SharedPtr search_;
  SetClient::SharedPtr seed_;
  UpdateClient::SharedPtr update_;
  rclcpp::Subscription<Estimate>::SharedPtr estimate_;
  PoseConsensus consensus_;
  geometry_msgs::msg::Pose candidate_;
  PlanarPose target_{};
  Phase phase_{Phase::idle};
  std::optional<std::int64_t> pending_;
  Clock::time_point next_request_{}, deadline_{}, scan_received_{}, estimate_received_{};
  std::int64_t started_{0}, last_candidate_{0}, seed_stamp_{0}, estimate_since_{0};
  std::int64_t last_estimate_{0}, last_scan_{0};
  int matches_{0};
  bool tracking_{false};
  std::string reason_;
};
}  // namespace aisleops

#endif  // AISLEOPS_MISSION__POSE_RECOVERY_HPP_
