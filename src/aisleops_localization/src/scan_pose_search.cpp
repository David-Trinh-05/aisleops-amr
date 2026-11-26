#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "aisleops_localization/srv/search_pose.hpp"
#include "pose_search.hpp"
#include "scan_geometry.hpp"

namespace
{
using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;
using Grid = nav_msgs::msg::OccupancyGrid;
using Scan = sensor_msgs::msg::LaserScan;
using Odometry = nav_msgs::msg::Odometry;
using Search = aisleops_localization::srv::SearchPose;

class ScanPoseSearch : public rclcpp::Node
{
public:
  ScanPoseSearch()
  : Node("scan_pose_search"), buffer_(get_clock()), listener_(buffer_, this, false)
  {
    map_subscription_ = create_subscription<Grid>(
      "map", rclcpp::QoS(1).transient_local(),
      [this](const Grid & map) {update_map(map);});
    scan_subscription_ = create_subscription<Scan>(
      "scan", rclcpp::SensorDataQoS(), [this](Scan::ConstSharedPtr scan) {
        scan_ = std::move(scan);
        scan_received_ = SteadyClock::now();
      });
    odom_subscription_ = create_subscription<Odometry>(
      "odom", rclcpp::SensorDataQoS(), [this](const Odometry & odom) {observe_motion(odom);});
    service_ = create_service<Search>(
      "localization/search_pose",
      [this](const std::shared_ptr<Search::Request>, std::shared_ptr<Search::Response> response) {
        search(*response);
      });
  }

private:
  void update_map(const Grid & map)
  {
    const auto & info = map.info;
    if (map.data.size() > 500'000 || !aisleops::valid_grid(map, "map") ||
      info.resolution < 0.02f || info.resolution > 0.20f ||
      info.width * info.resolution > 30.0f || info.height * info.resolution > 30.0f)
    {
      field_.reset();
      RCLCPP_WARN(get_logger(), "Pose search requires a valid warehouse-scale map");
      return;
    }
    tf2::fromMsg(info.origin, map_from_grid_);
    field_ = std::make_unique<aisleops::SearchMap>(
      info.width, info.height, info.resolution, map.data);
  }

  void observe_motion(const Odometry & odom)
  {
    const auto & stamp = odom.header.stamp;
    if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000) {
      stopped_since_.reset();
      return;
    }
    const rclcpp::Time time(stamp);
    const auto & twist = odom.twist.twist;
    if (odom.child_frame_id == "base_footprint" &&
      std::hypot(twist.linear.x, twist.linear.y) < 0.02 && std::abs(twist.angular.z) < 0.03)
    {
      const double gap = (time - last_odom_).seconds();
      if (!stopped_since_ || gap < 0.0 || gap > 0.25) {
        stopped_since_ = time;
      }
    } else {
      stopped_since_.reset();
    }
    last_odom_ = time;
    odom_received_ = SteadyClock::now();
  }

  void search(Search::Response & response)
  {
    response.status = Search::Response::UNAVAILABLE;
    response.candidates.header.frame_id = "map";
    const auto received = SteadyClock::now();
    if (!field_ || !scan_ || !aisleops::valid_scan(*scan_) || received - scan_received_ > 500ms) {
      response.detail = "A valid map and recent scan are required";
      return;
    }
    const rclcpp::Time stamp(scan_->header.stamp);
    const double age = (now() - stamp).seconds();
    if (age < -0.05 || age > 0.5 || !stopped_since_ || received - odom_received_ > 500ms ||
      (stamp - *stopped_since_).seconds() < 0.25 || std::abs((last_odom_ - stamp).seconds()) > 0.25)
    {
      response.detail = "A recent scan taken after the robot has settled is required";
      return;
    }

    tf2::Transform base_from_scan;
    try {
      tf2::fromMsg(
        buffer_.lookupTransform("base_footprint", scan_->header.frame_id, stamp).transform,
        base_from_scan);
    } catch (const tf2::TransformException &) {
      response.detail = "Missing transform from scan to base_footprint";
      return;
    }

    std::vector<aisleops::Point> hits;
    constexpr std::size_t max_endpoints = 90;
    const auto count = scan_->ranges.size();
    const auto stride = std::max<std::size_t>(1, (count + max_endpoints - 1) / max_endpoints);
    hits.reserve(max_endpoints);
    for (std::size_t i = 0; i < scan_->ranges.size(); i += stride) {
      const double range = scan_->ranges[i];
      if (aisleops::valid_return(*scan_, scan_->ranges[i])) {
        const double angle = scan_->angle_min + static_cast<double>(i) * scan_->angle_increment;
        const auto point = base_from_scan *
          tf2::Vector3(range * std::cos(angle), range * std::sin(angle), 0);
        hits.push_back({point.x(), point.y()});
      }
    }
    const auto candidates = field_->search(hits);
    response.candidates.header.stamp = stamp;
    for (const auto & candidate : candidates) {
      tf2::Quaternion rotation;
      rotation.setRPY(0, 0, candidate.yaw);
      const auto pose = map_from_grid_ *
        tf2::Transform(rotation, tf2::Vector3(candidate.x, candidate.y, 0));
      auto & message = response.candidates.poses.emplace_back();
      message.position.x = pose.getOrigin().x();
      message.position.y = pose.getOrigin().y();
      message.position.z = pose.getOrigin().z();
      message.orientation = tf2::toMsg(pose.getRotation());
      response.scores.push_back(candidate.score);
    }
    if (candidates.empty() || candidates.front().score < aisleops::minimum_candidate_score) {
      response.status = Search::Response::NO_MATCH;
      response.detail = "Insufficient map agreement";
    } else if (candidates.size() > 1 &&
      candidates[0].score - candidates[1].score < aisleops::ambiguity_margin)
    {
      response.status = Search::Response::AMBIGUOUS;
      response.detail = "Distinct poses fit the scan within the score margin";
    } else {
      response.status = Search::Response::CANDIDATE;
      response.detail = "One ranked pose exceeds the agreement and separation thresholds";
    }
  }

  std::unique_ptr<aisleops::SearchMap> field_;
  tf2::Transform map_from_grid_;
  Scan::ConstSharedPtr scan_;
  SteadyClock::time_point scan_received_{};
  SteadyClock::time_point odom_received_{};
  std::optional<rclcpp::Time> stopped_since_;
  rclcpp::Time last_odom_{0, 0, RCL_ROS_TIME};
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<Grid>::SharedPtr map_subscription_;
  rclcpp::Subscription<Scan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_subscription_;
  rclcpp::Service<Search>::SharedPtr service_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanPoseSearch>());
  rclcpp::shutdown();
}
