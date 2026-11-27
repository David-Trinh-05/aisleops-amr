"""Exercise the real Gazebo LiDAR watchdog during a delivery."""

from contextlib import ExitStack
import os
from pathlib import Path
from secrets import randbelow
import subprocess
import sys
from tempfile import TemporaryDirectory
from time import monotonic
from uuid import uuid4

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "benchmarks/recovery"))
from run import Recorder, observe, score, stamp_ns, start, wait_for_nav2


def set_rate(hz):
    response = subprocess.run(
        ["gz", "service", "-s", "/scan/set_rate", "--reqtype", "gz.msgs.Double",
         "--reptype", "gz.msgs.Empty", "--timeout", "3000", "--req", f"data: {hz}"],
        capture_output=True, text=True, timeout=5,
    )
    if response.returncode or response.stderr.strip():
        raise RuntimeError(f"Gazebo LiDAR rate request failed: {response.stderr.strip()}")


def verify(rec, simulator, mission):
    scans, diagnostics = [], []
    rec.node.create_subscription(LaserScan, "scan", lambda msg: scans.append(stamp_ns(msg.header)),
                                 qos_profile_sensor_data)
    rec.node.create_subscription(DiagnosticArray, "localization/scan_agreement",
                                 lambda msg: diagnostics.extend(s.message for s in msg.status), 10)

    def wait_for(predicate, description):
        end = monotonic() + 25
        while monotonic() < end:
            rec.spin()
            if predicate():
                return
            if simulator.poll() is not None:
                break
        raise RuntimeError(f"{description} missing. Mission={rec.events[-4:]} gate={rec.gate[-5:]}")

    wait_for(lambda: rec.active_leg == "receive" and scans and rec.odom
             and rec.odom[-1][1] > 0.20 and rec.gate and rec.gate[-1][1] == "OPEN", "Moving on the receiving leg")
    scan_count = len(scans)
    set_rate(0.5)
    try:
        wait_for(lambda: "no_recent_scan" in diagnostics, "LiDAR watchdog diagnostic")
        wait_for(lambda: any(state in ("HOLD localization_stale", "HOLD localization_unavailable")
                             for _, state in rec.gate), "Localization hold")
        held = next(t for t, state in rec.gate if state.startswith("HOLD "))
        wait_for(lambda: any(t >= held and speed == 0 for t, speed in rec.commands),
                 "Zero velocity command during HOLD")
        assert not any(t > held and state in ("READY", "OPEN") for t, state in rec.gate)
    finally:
        set_rate(10)

    terminal, sent = observe(rec, simulator, mission, [], True)
    gaps = [b - a for a, b in zip(scans[scan_count - 1:], scans[scan_count:])]
    assert gaps and max(gaps) >= 650_000_000, "Scan gap missing"
    openings = [t for t, state in rec.gate if t > held and state == "OPEN"]
    if openings:
        assert any(held < t <= openings[0] and state == "READY" for t, state in rec.gate)
        assert not any(held < t < openings[0] and speed > 0 for t, speed in rec.commands)
        assert [event for _, event in rec.events if event.startswith("RESUMED ")] == ["RESUMED receive"]
        assert score(rec, terminal, sent, [])["completed"], terminal
    else:
        assert terminal[1] in {"needs_help", "mission_failed"}, terminal
        assert not any(event.startswith("RESUMED ") for _, event in rec.events)
        assert not any(t > held and speed > 0 for t, speed in rec.commands)
    gap = max(gaps) / 1e9
    print(f"PASS LiDAR gap {gap:.3f} s, zero velocity during HOLD, "
          f"mission={terminal[1]} {terminal[2]}".rstrip())


def main():
    run_id = uuid4().hex
    partition = "aisleops-lidar-" + run_id
    namespace = "aisleops_lidar_" + run_id
    with TemporaryDirectory(prefix="aisleops-lidar-") as folder, ExitStack() as stack:
        os.environ.update(GZ_IP="127.0.0.1", GZ_PARTITION=partition,
                          ROS_DOMAIN_ID=str(170 + randbelow(40)),
                          ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", ROS_LOG_DIR=folder)
        simulator = stack.enter_context(start(
            ["ros2", "launch", "launch/warehouse.launch.py", f"namespace:={namespace}",
             f"gz_partition:={partition}", "map:=demo/landmarks.yaml", "world:=demo/landmarks.sdf",
             "sim_seed:=53", "ground_truth:=true", "recover_localization:=true"],
            Path(folder) / "warehouse.log", ROOT,
        ))
        rclpy.init()
        stack.callback(rclpy.try_shutdown)
        node = rclpy.create_node("verify_lidar", namespace=namespace, cli_args=["--ros-args", "-r", "/clock:=clock"],
                                 parameter_overrides=[Parameter("use_sim_time", value=True)])
        stack.callback(node.destroy_node)
        executor = SingleThreadedExecutor()
        stack.callback(executor.shutdown)
        executor.add_node(node)
        rec = Recorder(node, executor)
        wait_for_nav2(rec, simulator)
        mission = stack.enter_context(start(
            ["ros2", "run", "aisleops_mission", "delivery_mission", "--ros-args",
             "-r", f"__ns:=/{namespace}", "-r", f"/clock:=/{namespace}/clock",
             "-p", "use_sim_time:=true", "-p", "recover_localization:=true"],
            Path(folder) / "mission.log", ROOT,
        ))
        verify(rec, simulator, mission)


if __name__ == "__main__":
    main()
