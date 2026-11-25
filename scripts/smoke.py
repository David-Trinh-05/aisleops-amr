#!/usr/bin/env python3
"""Check the ROS–Gazebo data path with the reference robot or warehouse AMR."""

import argparse
from collections import deque
from contextlib import suppress
import math
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
from tempfile import TemporaryDirectory
from time import monotonic
from uuid import uuid4

from ament_index_python.packages import PackageNotFoundError, get_package_share_path
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener

from trajectory import run_trajectory, yaw


def forward_range(scan):
    if scan.angle_min <= 0 <= scan.angle_max and scan.angle_increment > 0:
        index = -scan.angle_min / scan.angle_increment
        if 0 <= index < len(scan.ranges) - 0.5:
            return scan.ranges[round(index)]
    return math.nan


def odom_yaw(odometry):
    return yaw(odometry.pose.pose.orientation)


def check_robot(processes, namespace, warehouse, trajectory_output):
    node = rclpy.create_node(
        "aisleops_smoke",
        namespace=namespace,
        cli_args=[
            "--ros-args", "-r", "/clock:=clock", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static",
        ],
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    transforms = Buffer(node=node)
    listener = TransformListener(transforms, node)
    messages = {}
    subscriptions = [("odom", Odometry), ("scan", LaserScan)]
    if warehouse:
        subscriptions.append(("imu", Imu))
    for topic, message_type in subscriptions:
        node.create_subscription(
            message_type, topic,
            lambda msg: messages.__setitem__(type(msg), msg),
            qos_profile_sensor_data,
        )
    publisher = node.create_publisher(TwistStamped, "cmd_vel", 10)
    speed = 0.0
    turn_rate = 0.0

    def publish_command():
        command = TwistStamped()
        command.header.stamp = node.get_clock().now().to_msg()
        command.header.frame_id = "base_footprint"
        command.twist.linear.x = speed
        command.twist.angular.z = turn_rate
        publisher.publish(command)

    timer = node.create_timer(0.05, publish_command)
    timer.cancel()
    deadline = monotonic() + (90 if trajectory_output else 45)

    def drive(linear, angular):
        nonlocal speed, turn_rate
        speed, turn_rate = linear, angular
        publish_command()
        if timer.is_canceled():
            timer.reset()

    def wait_for(label, condition):
        while monotonic() < deadline:
            for name, process in processes.items():
                status = process.poll()
                if status is not None and (name != "spawn" or status != 0):
                    raise RuntimeError(f"{name} exited with code {status}")
            rclpy.spin_once(node, timeout_sec=0.1)
            if condition():
                return
        odom = messages.get(Odometry)
        scan = messages.get(LaserScan)
        imu = messages.get(Imu)
        raise RuntimeError(
            f"timed out waiting for {label} "
            f"(sim time {node.get_clock().now().nanoseconds / 1e9:.3f}s, "
            f"odom frame {odom.header.frame_id if odom else None!r}, "
            f"scan frame {scan.header.frame_id if scan else None!r}, "
            f"imu frame {imu.header.frame_id if imu else None!r})"
        )

    def sensors_ready():
        odom = messages.get(Odometry)
        scan = messages.get(LaserScan)
        if odom is None or scan is None:
            return False
        pose = odom.pose.pose.position
        scan_time = Time.from_msg(scan.header.stamp)
        wall_distance = 11.25 if warehouse else 1.9
        return (
            odom.header.frame_id == "odom"
            and odom.child_frame_id == "base_footprint"
            and all(math.isfinite(value) for value in (pose.x, pose.y))
            and scan.header.frame_id == "base_scan"
            and len(scan.ranges) == 360
            and abs(forward_range(scan) - wall_distance) < 0.3
            and 0 < scan_time.nanoseconds <= node.get_clock().now().nanoseconds
            and transforms.can_transform("odom", "base_scan", scan_time)
        )

    def imu_ready():
        imu = messages.get(Imu)
        if imu is None or imu.header.frame_id != "imu_link":
            return False
        stamp = Time.from_msg(imu.header.stamp)
        return (
            0 < stamp.nanoseconds <= node.get_clock().now().nanoseconds
            and 9.0 < imu.linear_acceleration.z < 10.6
            and abs(imu.angular_velocity.z) < 0.05
            and transforms.can_transform("base_link", "imu_link", stamp)
        )

    def stop_robot():
        drive(0.0, 0.0)
        stop_time = node.get_clock().now().nanoseconds

        def stopped():
            odom = messages[Odometry]
            twist = odom.twist.twist
            return (
                Time.from_msg(odom.header.stamp).nanoseconds - stop_time >= 350_000_000
                and abs(twist.linear.x) < 0.02
                and abs(twist.angular.z) < 0.03
            )

        wait_for("the robot to stop after publishing zero /cmd_vel", stopped)
        return stop_time

    try:
        wait_for("the first /clock tick", lambda: node.get_clock().now().nanoseconds > 0)
        initial_time = node.get_clock().now().nanoseconds
        wait_for(
            "/clock to advance by 0.4 s",
            lambda: node.get_clock().now().nanoseconds - initial_time >= 400_000_000,
        )
        wait_for("valid /odom, /scan and odom -> base_scan at the scan stamp", sensors_ready)
        scan = messages[LaserScan]
        first_range = forward_range(scan)
        print(
            f"PASS  /clock advanced, /scan returned {len(scan.ranges)} rays "
            f"(front {first_range:.3f} m), TF available at scan timestamp",
            flush=True,
        )
        if warehouse:
            wait_for("a stationary /imu in imu_link", imu_ready)
            print(
                f"PASS  /imu is in imu_link, z acceleration "
                f"{messages[Imu].linear_acceleration.z:.3f} m/s²",
                flush=True,
            )

        if trajectory_output:
            run_trajectory(node, wait_for, drive, stop_robot, trajectory_output)
            return

        start = messages[Odometry].pose.pose.position.x
        speed = 0.12
        timer.reset()
        wait_for(
            "forward motion after publishing /cmd_vel",
            lambda: messages[Odometry].pose.pose.position.x - start >= 0.10,
        )
        stop_time = stop_robot()
        distance = messages[Odometry].pose.pose.position.x - start
        print(f"PASS  /cmd_vel moved the robot {distance:.3f} m in /odom, then stopped", flush=True)

        if warehouse:
            wait_for(
                "a fresh LiDAR scan after stopping",
                lambda: Time.from_msg(messages[LaserScan].header.stamp).nanoseconds
                >= stop_time + 350_000_000,
            )
            lidar_delta = first_range - forward_range(messages[LaserScan])
            if not (lidar_delta > 0.05 and abs(lidar_delta - distance) < 0.06):
                raise RuntimeError(
                    f"wall distance changed by {lidar_delta:.3f} m for {distance:.3f} m of odometry"
                )

            start_yaw = odom_yaw(messages[Odometry])
            start_turn = node.get_clock().now().nanoseconds
            turn_rate = 0.35

            def turning():
                imu = messages[Imu]
                yaw_delta = odom_yaw(messages[Odometry]) - start_yaw
                return (
                    0.25 <= yaw_delta < 0.8
                    and Time.from_msg(imu.header.stamp).nanoseconds > start_turn
                    and 0.15 < imu.angular_velocity.z < 0.7
                )

            wait_for("a positive yaw change in /odom and matching /imu angular velocity", turning)
            gyro = messages[Imu].angular_velocity.z
            stop_robot()
            yaw_delta = odom_yaw(messages[Odometry]) - start_yaw
            print(
                f"PASS  LiDAR wall distance decreased {lidar_delta:.3f} m; "
                f"robot rotated +{yaw_delta:.3f} rad with gyro +{gyro:.3f} rad/s, then stopped",
                flush=True,
            )
    finally:
        listener.unregister()
        node.destroy_node()


def turtlebot_commands(root, namespace):
    share = get_package_share_path("turtlebot3_gazebo")
    os.environ["GZ_SIM_RESOURCE_PATH"] = str(share / "models")
    return {
        "gazebo": [
            "gz", "sim", "-s", "-r", "--headless-rendering", str(root / "worlds/smoke.sdf"),
        ],
        "spawn": [
            "ros2", "run", "ros_gz_sim", "create", "-world", "smoke",
            "-name", "burger", "-z", "0.01",
            "-file", str(share / "models/turtlebot3_burger/model.sdf"),
        ],
        "bridge": [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "--ros-args", "-r", f"__ns:={namespace}",
            "-p", f"config_file:={share / 'params/turtlebot3_burger_bridge.yaml'}",
        ],
        "robot-state": [
            "ros2", "run", "robot_state_publisher", "robot_state_publisher",
            "--ros-args", "-r", f"__ns:={namespace}",
            "-r", "/clock:=clock", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static",
            "-p", "use_sim_time:=true",
            "-p", "robot_description:=" + (share / "urdf/turtlebot3_burger.urdf").read_text(),
        ],
    }


def run(log_dir, warehouse, trajectory_output):
    root = Path(__file__).resolve().parents[1]
    run_id = uuid4().hex
    namespace = f"/aisleops_{run_id}"
    os.environ.update(
        {
            "GZ_PARTITION": f"aisleops-{run_id}",
            "ROS_DOMAIN_ID": str(170 + secrets.randbelow(40)),
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
            "ROS_LOG_DIR": str(log_dir),
        }
    )
    os.environ.pop("ROS_STATIC_PEERS", None)
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    if warehouse:
        commands = {
            "warehouse": [
                "ros2", "launch", str(root / "launch/warehouse.launch.py"),
                f"namespace:={namespace}", f"gz_partition:={os.environ['GZ_PARTITION']}",
                f"ground_truth:={'true' if trajectory_output else 'false'}",
            ],
        }
    else:
        commands = turtlebot_commands(root, namespace)
    processes = {}
    try:
        for name, command in commands.items():
            with (log_dir / f"{name}.log").open("w") as log:
                processes[name] = subprocess.Popen(
                    command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                )
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        try:
            check_robot(processes, namespace, warehouse, trajectory_output)
        finally:
            rclpy.shutdown()
    finally:
        for process in processes.values():
            with suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGINT)
        for process in processes.values():
            with suppress(subprocess.TimeoutExpired):
                process.wait(timeout=3)
            with suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def interrupt(_signal_number, _frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--warehouse", action="store_true", help="check the custom AMR in the warehouse")
    mode.add_argument("--trajectory", type=Path, metavar="CSV", help="score the warehouse L-route")
    args = parser.parse_args()
    if args.trajectory and (args.trajectory.exists() or not args.trajectory.parent.is_dir()):
        parser.error("--trajectory requires a new CSV path in an existing directory")
    signal.signal(signal.SIGTERM, interrupt)
    with TemporaryDirectory(prefix="aisleops-smoke-") as directory:
        log_dir = Path(directory)
        try:
            run(log_dir, args.warehouse or bool(args.trajectory), args.trajectory)
        except KeyboardInterrupt:
            sys.exit(130)
        except (RuntimeError, OSError, PackageNotFoundError) as error:
            print(f"FAIL  {error}", file=sys.stderr)
            for path in sorted(log_dir.glob("*.log")):
                with path.open(errors="replace") as log:
                    tail = "".join(deque(log, maxlen=18))
                if tail:
                    print(f"--- {path.name} ---\n{tail}", file=sys.stderr, end="")
            sys.exit(1)
