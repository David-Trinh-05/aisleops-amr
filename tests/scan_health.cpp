#include <cassert>
#include <string_view>

#include "scan_health.hpp"

int main()
{
  aisleops::ScanHealth health;
  constexpr auto scan = 100'000'000LL;
  auto label = [&health] {return std::string_view(health.label());};
  for (int i = 1; i <= 5; ++i) {
    health.observe(true, i * scan);
  }
  assert(label() == "tracking");

  health.observe(false, 6 * scan);
  health.observe(false, 6 * scan);
  health.observe(false, 5 * scan);
  health.observe(false, scan);
  assert(label() == "suspect" && health.low_count() == 1);
  health.observe(true, 7 * scan);
  assert(label() == "suspect" && health.low_count() == 0);
  for (int i = 8; i <= 11; ++i) {
    health.observe(true, i * scan);
  }
  assert(label() == "tracking");

  health.observe(false, 12 * scan);
  health.observe(false, 17 * scan);
  assert(label() == "suspect" && health.low_count() == 1);
  health.observe(false, 18 * scan);
  health.observe(false, 19 * scan);
  assert(label() == "lost" && health.loss_latched());

  health.unavailable();
  assert(label() == "unavailable" && health.loss_latched());
  for (int i = 20; i <= 23; ++i) {
    health.observe(true, i * scan);
    assert(label() == "lost");
  }
  health.observe(true, 24 * scan);
  assert(label() == "tracking" && !health.loss_latched());
  health.reset_time();
  health.observe(true, scan);
  assert(label() == "initializing" && health.matching_count() == 1);
}
