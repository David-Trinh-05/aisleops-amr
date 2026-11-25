"""Evaluate wheel odometry on a timed L-shaped route in the warehouse."""

from bisect import bisect_left, bisect_right
import csv
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import NamedTuple

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

SPAWN_X = -4.5  # Published launch pose, with world and odom axes aligned.
MAX_TRUTH_GAP_NS = 100_000_000
ROUTE = (
    ("east", 0.25, 0.0, 4_000_000_000),
    ("turn_left", 0.0, 0.5, round(math.pi * 1e9)),
    ("north", 0.25, 0.0, 4_000_000_000),
)


class Sample(NamedTuple):
    stamp_ns: int
    x: float
    y: float
    yaw: float


class Comparison(NamedTuple):
    phase: str
    estimate: Sample
    actual: Sample
    xy_error: float
    yaw_error: float


def yaw(quaternion):
    q = quaternion
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def sample(header, pose):
    stamp_ns = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
    result = Sample(stamp_ns, pose.position.x, pose.position.y, yaw(pose.orientation))
    q = pose.orientation
    if (
        stamp_ns <= 0 or not all(math.isfinite(value) for value in result[1:])
        or not math.isclose(math.hypot(q.x, q.y, q.z, q.w), 1.0, abs_tol=1e-3)
    ):
        raise RuntimeError(f"invalid stamped pose in {header.frame_id!r}")
    return result


def wrap(angle):
    return math.remainder(angle, math.tau)


def interpolate(truth, stamps, stamp_ns):
    right = bisect_left(stamps, stamp_ns)
    if right < len(stamps) and stamps[right] == stamp_ns:
        return truth[right]
    if right == 0 or right == len(stamps):
        return None
    a, b = truth[right - 1], truth[right]
    gap = b.stamp_ns - a.stamp_ns
    if gap > MAX_TRUTH_GAP_NS:
        return None
    fraction = (stamp_ns - a.stamp_ns) / gap
    return Sample(
        stamp_ns,
        a.x + fraction * (b.x - a.x),
        a.y + fraction * (b.y - a.y),
        wrap(a.yaw + fraction * wrap(b.yaw - a.yaw)),
    )


def score(odometry, truth, phases, end_ns, output):
    phase_starts = [start for start, _ in phases]
    start_ns = phase_starts[0]
    odometry = [item for item in odometry if start_ns <= item.stamp_ns <= end_ns]
    stamps = [item.stamp_ns for item in truth]
    rows = []
    for odom in odometry:
        actual = interpolate(truth, stamps, odom.stamp_ns)
        if actual is None:
            continue
        estimate = Sample(odom.stamp_ns, SPAWN_X + odom.x, odom.y, odom.yaw)
        xy_error = math.hypot(estimate.x - actual.x, estimate.y - actual.y)
        yaw_error = abs(wrap(estimate.yaw - actual.yaw))
        phase = phases[bisect_right(phase_starts, odom.stamp_ns) - 1][1]
        rows.append(Comparison(phase, estimate, actual, xy_error, yaw_error))
    paired_stamps = [start_ns, *(row.actual.stamp_ns for row in rows), end_ns]
    if (
        len(rows) < 100 or len(rows) < 0.95 * len(odometry)
        or max(b - a for a, b in zip(paired_stamps, paired_stamps[1:])) > 200_000_000
    ):
        raise RuntimeError(f"insufficient time-aligned odometry and truth ({len(rows)}/{len(odometry)})")
    final = rows[-1]
    if (
        math.hypot(final.actual.x - (SPAWN_X + 1), final.actual.y - 1) > 0.30
        or abs(wrap(final.actual.yaw - math.pi / 2)) > 0.35
    ):
        raise RuntimeError(f"model finished outside the expected L-route endpoint: {final.actual}")
    rmse = math.sqrt(math.fsum(row.xy_error ** 2 for row in rows) / len(rows))
    max_xy = max(row.xy_error for row in rows)
    max_yaw = max(row.yaw_error for row in rows)

    with TemporaryDirectory(prefix=".aisleops-csv-", dir=output.parent) as folder:
        staged = Path(folder) / "trajectory.csv"
        with staged.open("w", encoding="utf-8", newline="") as report:
            writer = csv.writer(report)
            writer.writerow((
                "sim_time_ns", "phase", "odom_world_x_m", "odom_world_y_m", "odom_yaw_rad",
                "truth_x_m", "truth_y_m", "truth_yaw_rad", "xy_error_m", "yaw_error_rad",
            ))
            writer.writerows(
                (
                    row.actual.stamp_ns, row.phase, *row.estimate[1:], *row.actual[1:],
                    row.xy_error, row.yaw_error,
                )
                for row in rows
            )
        output.hardlink_to(staged)
    print(
        f"RESULT  matched {len(rows)}/{len(odometry)} samples, "
        f"XY RMSE {rmse:.6f} m, max XY error {max_xy:.6f} m, max yaw error {max_yaw:.6f} rad",
        flush=True,
    )
    print(
        f"RESULT  Gazebo endpoint ({final.actual.x:.3f}, {final.actual.y:.3f}) m, "
        f"yaw {final.actual.yaw:.3f} rad; final XY error {final.xy_error:.6f} m",
        flush=True,
    )
    print(f"CSV   {output}", flush=True)


def run_trajectory(node, wait_for, drive, stop_robot, output):
    odometry = []
    truth = []

    def read_odom(message):
        if message.header.frame_id != "odom" or message.child_frame_id != "base_footprint":
            raise RuntimeError("trajectory requires odom -> base_footprint")
        item = sample(message.header, message.pose.pose)
        if odometry and item.stamp_ns <= odometry[-1].stamp_ns:
            raise RuntimeError("odometry timestamps do not increase")
        odometry.append(item)

    def read_truth(message):
        if message.header.frame_id != "warehouse":
            raise RuntimeError(f"expected Gazebo warehouse frame, got {message.header.frame_id!r}")
        item = sample(message.header, message.pose)
        if truth and item.stamp_ns <= truth[-1].stamp_ns:
            raise RuntimeError("ground truth timestamps do not increase")
        truth.append(item)

    node.create_subscription(Odometry, "odom", read_odom, qos_profile_sensor_data)
    node.create_subscription(
        PoseStamped, "evaluation/ground_truth", read_truth, qos_profile_sensor_data,
    )
    clock = node.get_clock()
    wait_for("stationary odometry and Gazebo model pose", lambda: bool(odometry and truth))
    initial = truth[-1]
    if math.hypot(initial.x - SPAWN_X, initial.y) > 0.05 or abs(initial.yaw) > 0.05:
        raise RuntimeError(f"model is not at the configured warehouse spawn: {initial}")

    phases = []
    for phase, linear, angular, duration_ns in ROUTE:
        start_ns = clock.now().nanoseconds
        phases.append((start_ns, phase))
        finish_ns = start_ns + duration_ns
        drive(linear, angular)
        wait_for(f"end of the {phase} leg", lambda: clock.now().nanoseconds >= finish_ns)
        stop_robot()
    end_ns = odometry[-1].stamp_ns
    wait_for("ground truth through the final odometry sample", lambda: truth[-1].stamp_ns >= end_ns)
    score(odometry, truth, phases, end_ns, output)
