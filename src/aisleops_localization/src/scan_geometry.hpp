#ifndef AISLEOPS_LOCALIZATION__SCAN_GEOMETRY_HPP_
#define AISLEOPS_LOCALIZATION__SCAN_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace aisleops
{
inline bool valid_grid(const nav_msgs::msg::OccupancyGrid & grid, const std::string & map_frame)
{
  const auto & info = grid.info;
  const auto & p = info.origin.position;
  const auto & q = info.origin.orientation;
  const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return grid.header.frame_id == map_frame && info.width > 0 && info.height > 0 &&
         std::isfinite(info.resolution) && info.resolution > 0 &&
         grid.data.size() == static_cast<std::size_t>(info.width) * info.height &&
         std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::abs(norm2 - 1.0) < 1e-3 &&
         std::all_of(grid.data.begin(), grid.data.end(), [](int value) {
           return value >= -1 && value <= 100;
         });
}

inline bool valid_scan(const sensor_msgs::msg::LaserScan & scan)
{
  const auto & stamp = scan.header.stamp;
  return !scan.header.frame_id.empty() && stamp.sec >= 0 && stamp.nanosec < 1'000'000'000 &&
         (stamp.sec > 0 || stamp.nanosec > 0) &&
         std::isfinite(scan.angle_min) && std::isfinite(scan.angle_increment) &&
         scan.angle_increment != 0.0f && std::isfinite(scan.range_min) &&
         std::isfinite(scan.range_max) && scan.range_min >= 0.0f &&
         scan.range_max > scan.range_min;
}

inline bool valid_return(const sensor_msgs::msg::LaserScan & scan, float range)
{
  return std::isfinite(range) && range >= scan.range_min && range < scan.range_max;
}
}  // namespace aisleops

#endif  // AISLEOPS_LOCALIZATION__SCAN_GEOMETRY_HPP_
