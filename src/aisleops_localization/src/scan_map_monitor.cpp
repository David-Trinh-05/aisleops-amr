#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "scan_health.hpp"
#include "scan_geometry.hpp"

namespace
{
using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;
using Scan = sensor_msgs::msg::LaserScan;
using Grid = nav_msgs::msg::OccupancyGrid;
using Report = diagnostic_msgs::msg::DiagnosticArray;
using Status = diagnostic_msgs::msg::DiagnosticStatus;

const auto startup_only = rcl_interfaces::msg::ParameterDescriptor().set__read_only(true);

enum class Endpoint {unsupported, mismatch, match};

struct Counts
{
  std::size_t returns{0};
  std::size_t comparable{0};
  std::size_t matched{0};

  double known_fraction() const
  {
    return returns == 0 ? 0.0 : static_cast<double>(comparable) / returns;
  }

  double match_fraction() const
  {
    return comparable == 0 ? 0.0 : static_cast<double>(matched) / comparable;
  }
};

bool unit_interval(double value)
{
  return value > 0.0 && value <= 1.0;
}

Endpoint classify(const Grid & map, double x, double y, double radius, int occupied_threshold)
{
  const double resolution = map.info.resolution;
  const double width = map.info.width;
  const double height = map.info.height;
  if (!std::isfinite(x) || !std::isfinite(y) ||
    x < -radius || y < -radius || x >= width * resolution + radius ||
    y >= height * resolution + radius)
  {
    return Endpoint::unsupported;
  }

  const auto cell = [](double position, double scale, double cells) {
      return static_cast<std::size_t>(std::clamp(std::floor(position / scale), 0.0, cells - 1));
    };
  const auto value = [&map](std::size_t col, std::size_t row) {
      return map.data[row * map.info.width + col];
    };
  const double radius_squared = radius * radius;
  const auto end_col = cell(x + radius, resolution, width);
  const auto end_row = cell(y + radius, resolution, height);
  for (auto row = cell(y - radius, resolution, height); row <= end_row; ++row) {
    const double dy = (static_cast<double>(row) + 0.5) * resolution - y;
    for (auto col = cell(x - radius, resolution, width); col <= end_col; ++col) {
      if (value(col, row) >= occupied_threshold) {
        const double dx = (static_cast<double>(col) + 0.5) * resolution - x;
        if (dx * dx + dy * dy <= radius_squared) {
          return Endpoint::match;
        }
      }
    }
  }

  if (x >= 0.0 && y >= 0.0 && x < width * resolution && y < height * resolution &&
    value(cell(x, resolution, width), cell(y, resolution, height)) >= 0)
  {
    return Endpoint::mismatch;
  }
  return Endpoint::unsupported;
}

class ScanMapMonitor : public rclcpp::Node
{
public:
  ScanMapMonitor()
  : Node("scan_map_monitor"),
    map_frame_(declare_parameter<std::string>("map_frame", "map", startup_only)),
    radius_(declare_parameter<double>("match_radius_m", 0.18, startup_only)),
    minimum_returns_(declare_parameter<int>("minimum_returns", 30, startup_only)),
    minimum_known_(declare_parameter<double>("minimum_known_fraction", 0.55, startup_only)),
    minimum_match_(declare_parameter<double>("minimum_match_fraction", 0.65, startup_only)),
    occupied_threshold_(declare_parameter<int>("occupied_threshold", 65, startup_only)),
    buffer_(get_clock()),
    listener_(buffer_, this, false)
  {
    if (map_frame_.empty() || map_frame_.front() == '/' ||
      !std::isfinite(radius_) || radius_ <= 0.0 || minimum_returns_ <= 0 ||
      !unit_interval(minimum_known_) || !unit_interval(minimum_match_) ||
      occupied_threshold_ < 1 || occupied_threshold_ > 100)
    {
      throw std::invalid_argument("invalid scan agreement parameters");
    }
    buffer_.setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface()));
    filter_ = std::make_unique<tf2_ros::MessageFilter<Scan>>(
      buffer_, map_frame_, 10, get_node_logging_interface(), get_node_clock_interface(), 250ms);
    filter_->registerCallback(
      [this](const Scan::ConstSharedPtr & message) {evaluate(*message);});
    publisher_ = create_publisher<Report>("localization/scan_agreement", 10);
    map_subscription_ = create_subscription<Grid>(
      "map", rclcpp::QoS(1).reliable().transient_local(),
      [this](Grid::ConstSharedPtr message) {update_map(std::move(message));});
    scan_subscription_ = create_subscription<Scan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](Scan::ConstSharedPtr message) {receive_scan(std::move(message));});
    watchdog_ = create_wall_timer(250ms, [this] {check_streams();});
  }

private:
  void update_map(Grid::ConstSharedPtr map)
  {
    health_.unavailable();
    const auto & info = map->info;
    if (!aisleops::valid_grid(*map, map_frame_)) {
      map_.reset();
      map_status_ = "invalid_map";
      report(Status::STALE, map_status_, now());
      return;
    }
    tf2::Transform map_from_grid;
    tf2::fromMsg(info.origin, map_from_grid);
    grid_from_map_ = map_from_grid.inverse();
    map_ = std::move(map);
  }

  void receive_scan(Scan::ConstSharedPtr scan)
  {
    last_received_ = SteadyClock::now();
    if (!map_) {
      return;
    }
    if (!aisleops::valid_scan(*scan)) {
      last_evaluated_ = last_received_;
      report_unavailable(Status::STALE, "invalid_scan", now());
      return;
    }
    filter_->add(scan);
  }

  void evaluate(const Scan & scan)
  {
    if (!map_) {
      return;
    }
    const rclcpp::Time stamp(scan.header.stamp);
    const double age = (sample_clock() - stamp).seconds();
    if (age > 0.75 || age < -0.05) {
      last_evaluated_ = SteadyClock::now();
      report_unavailable(Status::STALE, "stale_scan_stamp", stamp);
      return;
    }
    tf2::Transform map_from_scan;
    try {
      tf2::fromMsg(
        buffer_.lookupTransform(map_frame_, scan.header.frame_id, stamp).transform,
        map_from_scan);
    } catch (const tf2::TransformException &) {
      return;
    }

    const auto grid_from_scan = grid_from_map_ * map_from_scan;
    Counts counts;
    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
      const double range = scan.ranges[index];
      if (!aisleops::valid_return(scan, scan.ranges[index])) {
        continue;
      }
      ++counts.returns;
      const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
      const auto point = grid_from_scan *
        tf2::Vector3(range * std::cos(angle), range * std::sin(angle), 0.0);
      const auto result = classify(*map_, point.x(), point.y(), radius_, occupied_threshold_);
      counts.comparable += result != Endpoint::unsupported;
      counts.matched += result == Endpoint::match;
    }
    last_evaluated_ = SteadyClock::now();
    if (counts.comparable < static_cast<std::size_t>(minimum_returns_) ||
      counts.known_fraction() < minimum_known_)
    {
      report_unavailable(Status::WARN, "insufficient_overlap", stamp, &counts);
    } else if (counts.match_fraction() < minimum_match_) {
      health_.observe(false, stamp.nanoseconds());
      report(Status::WARN, "low_agreement", stamp, &counts);
    } else {
      health_.observe(true, stamp.nanoseconds());
      report(Status::OK, "matching", stamp, &counts);
    }
  }

  void check_streams()
  {
    const auto time = sample_clock();
    if (!map_) {
      report_unavailable(Status::STALE, map_status_, time);
    } else if (SteadyClock::now() - last_received_ >= 750ms) {
      report_unavailable(Status::STALE, "no_recent_scan", time);
    } else if (SteadyClock::now() - last_evaluated_ >= 750ms) {
      report_unavailable(Status::STALE, "transform_unavailable", time);
    }
  }

  rclcpp::Time sample_clock()
  {
    const auto current = now();
    if (current.nanoseconds() < last_clock_ns_) {
      health_.reset_time();
    }
    last_clock_ns_ = current.nanoseconds();
    return current;
  }

  void report_unavailable(std::uint8_t level, const std::string & reason, const rclcpp::Time & time,
    const Counts * counts = nullptr)
  {
    health_.unavailable();
    report(level, reason, time, counts);
  }

  void report(std::uint8_t level, const std::string & message, const rclcpp::Time & time,
    const Counts * counts = nullptr)
  {
    Report report;
    report.header.stamp = time;
    report.header.frame_id = map_frame_;
    auto & status = report.status.emplace_back();
    status.name = get_fully_qualified_name();
    status.hardware_id = "warehouse_lidar";
    status.level = level;
    status.message = message;
    const auto add = [&status](const char * key, const std::string & value) {
        auto & entry = status.values.emplace_back();
        entry.key = key;
        entry.value = value;
      };
    add("health_state", health_.label());
    add("loss_latched", health_.loss_latched() ? "true" : "false");
    add("consecutive_low", std::to_string(health_.low_count()));
    add("consecutive_matching", std::to_string(health_.matching_count()));
    if (counts) {
      add("finite_returns", std::to_string(counts->returns));
      add("comparable_returns", std::to_string(counts->comparable));
      add("matched_returns", std::to_string(counts->matched));
      add("known_fraction", std::to_string(counts->known_fraction()));
      add("match_fraction", std::to_string(counts->match_fraction()));
    }
    publisher_->publish(report);
  }

  const std::string map_frame_;
  const double radius_;
  const int minimum_returns_;
  const double minimum_known_;
  const double minimum_match_;
  const int occupied_threshold_;
  aisleops::ScanHealth health_;
  std::int64_t last_clock_ns_{0};
  Grid::ConstSharedPtr map_;
  std::string map_status_{"waiting_for_map"};
  tf2::Transform grid_from_map_;
  SteadyClock::time_point last_received_{};
  SteadyClock::time_point last_evaluated_{};
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::unique_ptr<tf2_ros::MessageFilter<Scan>> filter_;
  rclcpp::Publisher<Report>::SharedPtr publisher_;
  rclcpp::Subscription<Grid>::SharedPtr map_subscription_;
  rclcpp::Subscription<Scan>::SharedPtr scan_subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<ScanMapMonitor>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("scan_map_monitor"), "%s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
