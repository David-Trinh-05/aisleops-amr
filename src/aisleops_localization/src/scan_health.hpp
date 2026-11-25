#ifndef AISLEOPS_LOCALIZATION__SCAN_HEALTH_HPP_
#define AISLEOPS_LOCALIZATION__SCAN_HEALTH_HPP_

#include <algorithm>
#include <cstdint>
#include <optional>

namespace aisleops
{
class ScanHealth
{
public:
  enum class State {initializing, tracking, suspect, lost, unavailable};
  static constexpr unsigned lost_after = 3;
  static constexpr unsigned tracking_after = 5;
  static constexpr std::int64_t maximum_gap_ns = 350'000'000;

  void observe(bool matching, std::int64_t stamp_ns)
  {
    if (last_stamp_) {
      const auto gap = stamp_ns - *last_stamp_;
      if (gap <= 0) {
        return;
      }
      if (gap > maximum_gap_ns) {
        unavailable();
      }
    }
    last_stamp_ = stamp_ns;

    if (!matching) {
      matching_ = 0;
      low_ = std::min(low_ + 1, lost_after);
      lost_ = lost_ || low_ == lost_after;
      state_ = lost_ ? State::lost : State::suspect;
      return;
    }
    low_ = 0;
    matching_ = std::min(matching_ + 1, tracking_after);
    if (matching_ == tracking_after) {
      lost_ = false;
      state_ = State::tracking;
    } else if (lost_) {
      state_ = State::lost;
    } else if (state_ == State::unavailable) {
      state_ = State::initializing;
    }
  }

  void unavailable()
  {
    low_ = matching_ = 0;
    state_ = State::unavailable;
  }

  void reset_time()
  {
    last_stamp_.reset();
    unavailable();
  }

  const char * label() const
  {
    switch (state_) {
      case State::initializing: return "initializing";
      case State::tracking: return "tracking";
      case State::suspect: return "suspect";
      case State::lost: return "lost";
      case State::unavailable: return "unavailable";
    }
    return "unavailable";
  }

  unsigned low_count() const {return low_;}
  unsigned matching_count() const {return matching_;}
  bool loss_latched() const {return lost_;}

private:
  State state_{State::initializing};
  bool lost_{false};
  unsigned low_{0};
  unsigned matching_{0};
  std::optional<std::int64_t> last_stamp_;
};
}  // namespace aisleops

#endif  // AISLEOPS_LOCALIZATION__SCAN_HEALTH_HPP_
