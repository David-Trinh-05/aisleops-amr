import math
import unittest

from trajectory import Sample, interpolate


class InterpolationTests(unittest.TestCase):
    def test_yaw_interpolates_across_the_pi_boundary(self):
        start = Sample(1_000_000_000, 0.0, 2.0, math.radians(179))
        end = Sample(1_020_000_000, 2.0, 4.0, math.radians(-179))
        middle = interpolate([start, end], [start.stamp_ns, end.stamp_ns], 1_010_000_000)
        self.assertEqual((middle.x, middle.y), (1.0, 3.0))
        self.assertAlmostEqual(abs(middle.yaw), math.pi)

    def test_missing_data_is_not_extrapolated_or_interpolated_across_a_dropout(self):
        start = Sample(1_000_000_000, 0.0, 0.0, 0.0)
        end = Sample(1_200_000_000, 1.0, 0.0, 0.0)
        truth, stamps = [start, end], [start.stamp_ns, end.stamp_ns]
        for timestamp in (999_999_999, 1_100_000_000, 1_200_000_001):
            with self.subTest(timestamp=timestamp):
                self.assertIsNone(interpolate(truth, stamps, timestamp))
        self.assertEqual(interpolate(truth, stamps, end.stamp_ns), end)


if __name__ == "__main__":
    unittest.main()
