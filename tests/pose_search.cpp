#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "pose_search.hpp"

int main()
{
  constexpr int width = 160;
  constexpr int height = 120;
  constexpr double cell = 0.05;
  std::vector<std::int8_t> grid(width * height, 0);
  auto box = [&grid](int x, int y, int w, int h) {
      for (int j = y; j < y + h; ++j) {
        for (int i = x; i < x + w; ++i) {
          grid[j * width + i] = 100;
        }
      }
    };
  box(0, 0, width, 1);
  box(0, height - 1, width, 1);
  box(0, 0, 1, height);
  box(width - 1, 0, 1, height);
  box(18, 45, 12, 45);
  box(50, 24, 40, 8);
  box(111, 61, 12, 31);
  box(119, 34, 27, 7);

  constexpr double x = 2.07;
  constexpr double y = 2.73;
  constexpr double yaw = 0.41;
  using aisleops::pi;
  std::vector<aisleops::Point> hits;
  for (int beam = 0; beam < 90; ++beam) {
    const double angle = -pi + beam * 2.0 * pi / 90;
    const double c = std::cos(angle + yaw);
    const double s = std::sin(angle + yaw);
    for (double range = 0.0; range < 12.0; range += 0.01) {
      const auto i = static_cast<int>((x + range * c) / cell);
      const auto j = static_cast<int>((y + range * s) / cell);
      if (i < 0 || j < 0 || i >= width || j >= height) {
        break;
      }
      if (grid[j * width + i] == 100) {
        hits.push_back({range * std::cos(angle), range * std::sin(angle)});
        break;
      }
    }
  }

  const aisleops::SearchMap map(width, height, cell, grid);
  assert(map.search({}).empty());
  const auto result = map.search(hits);
  assert(result.size() >= 2);
  const auto & best = result.front();
  const double xy_error = std::hypot(best.x - x, best.y - y);
  const double yaw_error = std::abs(std::remainder(best.yaw - yaw, 2.0 * pi));
  const double margin = best.score - result[1].score;
  std::cout << "Asymmetric grid: xy " << xy_error << " m, yaw " << yaw_error
            << " rad, score " << best.score << ", margin " << margin << '\n';
  assert(xy_error < 0.08 && yaw_error < 0.05);
  assert(best.score >= aisleops::minimum_candidate_score && margin >= aisleops::ambiguity_margin);
}
