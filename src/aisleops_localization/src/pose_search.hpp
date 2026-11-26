#ifndef AISLEOPS_LOCALIZATION__POSE_SEARCH_HPP_
#define AISLEOPS_LOCALIZATION__POSE_SEARCH_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace aisleops
{
struct Point
{
  double x, y;
};

struct Candidate
{
  double x, y, yaw, score;
};

constexpr double pi = 3.14159265358979323846;
constexpr double minimum_candidate_score = 0.60;
constexpr double ambiguity_margin = 0.06;

class SearchMap
{
public:
  SearchMap(int width, int height, double resolution, std::vector<std::int8_t> occupancy)
  : width_(width), height_(height), resolution_(resolution), occupancy_(std::move(occupancy)),
    squared_distance_(occupancy_.size()), weights_(occupancy_.size())
  {
    constexpr double radius = 0.64;
    constexpr double sigma = 0.20;
    const auto radius2 = static_cast<float>(radius * radius);
    const int extent = static_cast<int>(std::floor(radius / resolution_));
    std::vector<float> horizontal(occupancy_.size(), radius2);
    const auto squared_metres = [this](int cells) {
        const auto metres = cells * resolution_;
        return static_cast<float>(metres * metres);
      };
    // Find the closest occupied cell along each row, then search nearby rows.
    for (int y = 0; y < height_; ++y) {
      int last = -extent - 1;
      for (int x = 0; x < width_; ++x) {
        if (occupancy_[index(x, y)] >= 65) {
          last = x;
        }
        horizontal[index(x, y)] = std::min(radius2, squared_metres(x - last));
      }
      last = width_ + extent;
      for (int x = width_ - 1; x >= 0; --x) {
        auto & distance = horizontal[index(x, y)];
        if (occupancy_[index(x, y)] >= 65) {
          last = x;
        }
        distance = std::min(distance, squared_metres(last - x));
      }
    }
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        auto best = radius2;
        for (int row = std::max(0, y - extent); row <= std::min(height_ - 1, y + extent); ++row) {
          best = std::min(best, horizontal[index(x, row)] + squared_metres(y - row));
        }
        squared_distance_[index(x, y)] = best;
        weights_[index(x, y)] = best < radius2 ? std::exp(-0.5 * best / (sigma * sigma)) : 0.0;
      }
    }
  }

  std::vector<Candidate> search(const std::vector<Point> & endpoints) const
  {
    if (endpoints.size() < 24) {
      return {};
    }
    std::vector<Point> origins;
    const int stride = std::max(1, static_cast<int>(std::lround(0.30 / resolution_)));
    for (int y = 0; y < height_; y += stride) {
      for (int x = 0; x < width_; x += stride) {
        const Point origin{(x + 0.5) * resolution_, (y + 0.5) * resolution_};
        if (free_origin(origin.x, origin.y)) {
          origins.push_back(origin);
        }
      }
    }

    std::vector<Candidate> coarse;
    coarse.reserve(origins.size() * 24);
    for (int step = 0; step < 24; ++step) {
      const double angle = -pi + step * pi / 12.0;
      const auto rotated = rotate(endpoints, angle);
      for (const auto origin : origins) {
        coarse.push_back({origin.x, origin.y, angle, score(rotated, origin.x, origin.y)});
      }
    }
    auto seeds = distinct_best(coarse, 16);
    for (auto & seed : seeds) {
      const auto coarse_pose = seed;
      for (int dt = -6; dt <= 6; ++dt) {
        const double angle = coarse_pose.yaw + dt * pi / 72.0;
        const auto rotated = rotate(endpoints, angle);
        for (int dy = -4; dy <= 4; ++dy) {
          for (int dx = -4; dx <= 4; ++dx) {
            const double x = coarse_pose.x + dx * 0.05;
            const double y = coarse_pose.y + dy * 0.05;
            if (free_origin(x, y)) {
              const auto fit = score(rotated, x, y);
              if (fit > seed.score) {
                seed = {x, y, std::remainder(angle, 2.0 * pi), fit};
              }
            }
          }
        }
      }
    }
    return distinct_best(seeds, 5);
  }

private:
  std::size_t index(int x, int y) const {return static_cast<std::size_t>(y) * width_ + x;}

  bool inside(double x, double y) const
  {
    return x >= 0 && y >= 0 && x < width_ * resolution_ && y < height_ * resolution_;
  }

  bool free_origin(double x, double y) const
  {
    if (!inside(x, y)) {
      return false;
    }
    const auto cell = index(static_cast<int>(x / resolution_), static_cast<int>(y / resolution_));
    return occupancy_[cell] == 0 && squared_distance_[cell] >= 0.28 * 0.28;
  }

  double weight(double x, double y) const
  {
    if (!inside(x, y)) {
      return 0;
    }
    const double u = std::clamp(x / resolution_ - 0.5, 0.0, width_ - 1.0);
    const double v = std::clamp(y / resolution_ - 0.5, 0.0, height_ - 1.0);
    const int x0 = static_cast<int>(u);
    const int y0 = static_cast<int>(v);
    const int x1 = std::min(x0 + 1, width_ - 1);
    const int y1 = std::min(y0 + 1, height_ - 1);
    const double tx = u - x0;
    const double ty = v - y0;
    const double bottom = (1 - tx) * weights_[index(x0, y0)] + tx * weights_[index(x1, y0)];
    const double top = (1 - tx) * weights_[index(x0, y1)] + tx * weights_[index(x1, y1)];
    return (1 - ty) * bottom + ty * top;
  }

  double score(const std::vector<Point> & rotated, double x, double y) const
  {
    double sum = 0.0;
    for (const auto point : rotated) {
      sum += weight(x + point.x, y + point.y);
    }
    return sum / rotated.size();
  }

  static std::vector<Point> rotate(const std::vector<Point> & endpoints, double angle)
  {
    const auto c = std::cos(angle);
    const auto s = std::sin(angle);
    std::vector<Point> result;
    result.reserve(endpoints.size());
    for (const auto point : endpoints) {
      result.push_back({c * point.x - s * point.y, s * point.x + c * point.y});
    }
    return result;
  }

  static std::vector<Candidate> distinct_best(std::vector<Candidate> & candidates, std::size_t limit)
  {
    std::stable_sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
      return a.score > b.score;
    });
    std::vector<Candidate> selected;
    for (const auto & candidate : candidates) {
      if (std::none_of(selected.begin(), selected.end(), [&candidate](const auto & other) {
        return std::hypot(candidate.x - other.x, candidate.y - other.y) < 0.65 &&
               std::abs(std::remainder(candidate.yaw - other.yaw, 2.0 * pi)) < pi / 6;
      })) {
        selected.push_back(candidate);
        if (selected.size() == limit) {
          break;
        }
      }
    }
    return selected;
  }

  int width_, height_;
  double resolution_;
  std::vector<std::int8_t> occupancy_;
  std::vector<float> squared_distance_;
  std::vector<float> weights_;
};
}  // namespace aisleops

#endif  // AISLEOPS_LOCALIZATION__POSE_SEARCH_HPP_
