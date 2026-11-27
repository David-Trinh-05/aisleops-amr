#include <cassert>
#include <iostream>

#include "pose_evidence.hpp"

int main()
{
  using aisleops::PoseConsensus;
  using Result = PoseConsensus::Result;
  PoseConsensus wrapped;
  assert(wrapped.observe(1'000'000'000, {2.0, 1.0, 3.13}) == Result::collecting);
  assert(wrapped.observe(1'000'000'000, {2.0, 1.0, 3.13}) == Result::collecting);
  assert(wrapped.observe(900'000'000, {4.0, 1.0, 0.0}) == Result::collecting);
  assert(wrapped.observe(1'100'000'000, {2.02, 1.01, -3.13}) == Result::collecting);
  assert(wrapped.observe(1'200'000'000, {2.02, 1.01, -3.13}) == Result::ready);

  PoseConsensus drift;
  assert(drift.observe(1'000'000'000, {2.0, 1.0, 0.0}) == Result::collecting);
  assert(drift.observe(1'100'000'000, {2.15, 1.0, 0.0}) == Result::collecting);
  assert(drift.observe(1'200'000'000, {2.30, 1.0, 0.0}) == Result::inconsistent);

  PoseConsensus gap;
  assert(gap.observe(1'000'000'000, {2.0, 1.0, 0.0}) == Result::collecting);
  assert(gap.observe(1'100'000'000, {2.0, 1.0, 0.0}) == Result::collecting);
  assert(gap.observe(1'800'000'000, {4.0, 1.0, 0.0}) == Result::collecting);
  assert(gap.observe(1'900'000'000, {4.0, 1.0, 0.0}) == Result::collecting);
  assert(gap.observe(2'000'000'000, {4.0, 1.0, 0.0}) == Result::ready);
  std::cout << "PASS  duplicates, out-of-order stamps, yaw wrapping, drift and scan gaps.\n";
}
