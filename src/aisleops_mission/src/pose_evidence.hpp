#ifndef AISLEOPS_MISSION__POSE_EVIDENCE_HPP_
#define AISLEOPS_MISSION__POSE_EVIDENCE_HPP_

#include <cmath>
#include <cstdint>

namespace aisleops
{
struct PlanarPose
{
  double x, y, yaw;

  bool near(const PlanarPose & other, double metres, double radians) const
  {
    constexpr double turn = 6.283185307179586;
    return std::hypot(x - other.x, y - other.y) <= metres &&
           std::abs(std::remainder(yaw - other.yaw, turn)) <= radians;
  }
};

class PoseConsensus
{
public:
  enum class Result {collecting, ready, inconsistent};

  Result observe(std::int64_t stamp, const PlanarPose & pose)
  {
    if (stamp <= last_) {
      return Result::collecting;
    }
    if (stamp - last_ > 500'000'000) {
      count_ = 0;
    }
    last_ = stamp;
    if (count_ == 0) {
      first_ = pose;
    } else if (!first_.near(pose, 0.20, 0.15)) {
      return Result::inconsistent;
    }
    return ++count_ >= 3 ? Result::ready : Result::collecting;
  }

private:
  std::int64_t last_{0};
  PlanarPose first_{};
  int count_{0};
};
}  // namespace aisleops

#endif  // AISLEOPS_MISSION__POSE_EVIDENCE_HPP_
