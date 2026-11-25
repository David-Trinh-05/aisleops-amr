#!/usr/bin/env python3
"""Check the ROS–Gazebo data path with an isolated TurtleBot3 Burger."""

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
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def check_robot(processes, namespace):
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
    for topic, message_type in (("odom", Odometry), ("scan", LaserScan)):
        node.create_subscription(
            message_type, topic,
            lambda msg: messages.__setitem__(type(msg), msg),
            qos_profile_sensor_data,
        )
    publisher = node.create_publisher(TwistStamped, "cmd_vel", 10)
    speed = 0.0

    def publish_command():
        command = TwistStamped()
        command.header.stamp = node.get_clock().now().to_msg()
        command.header.frame_id = "base_footprint"
        command.twist.linear.x = speed
        publisher.publish(command)

    timer = node.create_timer(0.05, publish_command)
    timer.cancel()
    deadline = monotonic() + 45

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
        raise RuntimeError(
            f"timed out waiting for {label} "
            f"(sim time {node.get_clock().now().nanoseconds / 1e9:.3f}s, "
            f"odom frame {odom.header.frame_id if odom else None!r}, "
            f"scan frame {scan.header.frame_id if scan else None!r})"
        )

    def sensors_ready():
        odom = messages.get(Odometry)
        scan = messages.get(LaserScan)
        if odom is None or scan is None:
            return False
        pose = odom.pose.pose.position
        scan_time = Time.from_msg(scan.header.stamp)
        return (
            odom.header.frame_id == "odom"
            and odom.child_frame_id == "base_footprint"
            and all(math.isfinite(value) for value in (pose.x, pose.y))
            and scan.header.frame_id == "base_scan"
            and len(scan.ranges) == 360
            and 1.6 < scan.ranges[0] < 2.2  # Front ray hits the wall at x=1.9 m.
            and 0 < scan_time.nanoseconds <= node.get_clock().now().nanoseconds
            and transforms.can_transform("odom", "base_scan", scan_time)
        )

    try:
        wait_for("the first /clock tick", lambda: node.get_clock().now().nanoseconds > 0)
        initial_time = node.get_clock().now().nanoseconds
        wait_for(
            "/clock to advance by 0.4 s",
            lambda: node.get_clock().now().nanoseconds - initial_time >= 400_000_000,
        )
        wait_for("valid /odom, /scan and odom -> base_scan at the scan stamp", sensors_ready)
        scan = messages[LaserScan]
        print(
            f"PASS  /clock advanced, /scan returned {len(scan.ranges)} rays "
            f"(front {scan.ranges[0]:.3f} m), TF available at scan timestamp",
            flush=True,
        )

        start = messages[Odometry].pose.pose.position.x
        speed = 0.12
        timer.reset()
        wait_for(
            "forward motion after publishing /cmd_vel",
            lambda: messages[Odometry].pose.pose.position.x - start >= 0.10,
        )
        speed = 0.0
        publish_command()
        stop_time = node.get_clock().now().nanoseconds
        wait_for(
            "the robot to stop after publishing zero /cmd_vel",
            lambda: (
                Time.from_msg(messages[Odometry].header.stamp).nanoseconds - stop_time
                >= 350_000_000
                and abs(messages[Odometry].twist.twist.linear.x) < 0.02
            ),
        )
        distance = messages[Odometry].pose.pose.position.x - start
        print(f"PASS  /cmd_vel moved the robot {distance:.3f} m in /odom, then stopped")
    finally:
        listener.unregister()
        node.destroy_node()


def run(log_dir):
    share = get_package_share_path("turtlebot3_gazebo")
    world = Path(__file__).resolve().parents[1] / "worlds" / "smoke.sdf"
    run_id = uuid4().hex
    namespace = f"/aisleops_{run_id}"
    os.environ.update(
        {
            "GZ_SIM_RESOURCE_PATH": str(share / "models"),
            "GZ_PARTITION": f"aisleops-{run_id}",
            "ROS_DOMAIN_ID": str(170 + secrets.randbelow(40)),
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
            "ROS_LOG_DIR": str(log_dir),
        }
    )
    os.environ.pop("ROS_STATIC_PEERS", None)
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    commands = {
        "gazebo": ["gz", "sim", "-s", "-r", "--headless-rendering", str(world)],
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
            "-r", "/tf:=tf", "-r", "/tf_static:=tf_static",
            "-p", "use_sim_time:=true",
            "-p", "robot_description:=" + (share / "urdf/turtlebot3_burger.urdf").read_text(),
        ],
    }
    processes = {}
    try:
        for name, command in commands.items():
            with (log_dir / f"{name}.log").open("w") as log:
                processes[name] = subprocess.Popen(
                    command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                )
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        try:
            check_robot(processes, namespace)
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
    signal.signal(signal.SIGTERM, interrupt)
    with TemporaryDirectory(prefix="aisleops-smoke-") as directory:
        log_dir = Path(directory)
        try:
            run(log_dir)
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
