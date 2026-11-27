"""Drive the gate with deterministic clients and test release evidence."""

from contextlib import ExitStack
import os
from pathlib import Path
import secrets
import sys
from tempfile import TemporaryDirectory
from time import monotonic
from uuid import uuid4

from aisleops_localization.srv import SearchPose
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.srv import SetInitialPose
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "benchmarks/recovery"))
from process import start


def verify(node, gate):
    clock = node.get_clock()
    states, velocities, seeds = [], [], []
    search_calls = 0
    repeated_candidate = None
    varying_candidates = False
    health = "tracking"
    frozen_report = None
    covariance = 0.5
    good_estimates = 0
    command_stamp = None
    send_commands = False

    reports = node.create_publisher(DiagnosticArray, "localization/scan_agreement", 10)
    estimates = node.create_publisher(PoseWithCovarianceStamped, "amcl_pose", 10)
    commands = node.create_publisher(TwistStamped, "cmd_vel_guard_in", 10)
    mission_status = node.create_publisher(
        String, "mission_status", QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(String, "localization/gate_state", lambda msg: states.append(msg.data), 10)
    node.create_subscription(
        TwistStamped, "cmd_vel_guarded", lambda msg: velocities.append(msg.twist.linear.x), 20,
    )

    def search(_, result):
        nonlocal search_calls, repeated_candidate
        search_calls += 1
        if repeated_candidate is None:
            repeated_candidate = clock.now().to_msg()
        result.status = SearchPose.Response.CANDIDATE
        result.candidates.header.frame_id = "map"
        result.candidates.header.stamp = clock.now().to_msg() if varying_candidates else repeated_candidate
        candidate = Pose()
        candidate.position.x = 2.0
        candidate.orientation.w = 1.0
        result.candidates.poses = [candidate]
        result.scores = [0.95]
        return result

    def seed(request, response):
        seeds.append((search_calls, request.pose))
        return response

    def update(_, response):
        nonlocal good_estimates
        estimate = PoseWithCovarianceStamped()
        estimate.header.frame_id = "map"
        estimate.header.stamp = clock.now().to_msg()
        estimate.pose.pose.position.x = 2.02
        estimate.pose.pose.orientation.w = 1.0
        estimate.pose.covariance[0] = estimate.pose.covariance[7] = covariance
        estimate.pose.covariance[35] = 0.01
        estimates.publish(estimate)
        if covariance < 0.09:
            good_estimates += 1
        return response

    node.create_service(SearchPose, "localization/search_pose", search)
    node.create_service(SetInitialPose, "set_initial_pose", seed)
    node.create_service(Empty, "request_nomotion_update", update)
    recover = node.create_client(Trigger, "localization/recover")
    resume = node.create_client(Trigger, "localization/resume")

    def publish_inputs():
        stamp = clock.now().to_msg()
        report = DiagnosticArray()
        report.header.frame_id = "map"
        report.header.stamp = frozen_report or stamp
        report.status = [DiagnosticStatus(
            message="matching" if health == "tracking" else "low_agreement",
            values=[KeyValue(key="health_state", value=health),
                    KeyValue(key="loss_latched", value=str(health == "lost").lower())],
        )]
        reports.publish(report)
        if send_commands:
            twist = TwistStamped()
            twist.header.stamp = command_stamp or stamp
            twist.twist.linear.x = 0.4
            commands.publish(twist)

    node.create_timer(0.04, publish_inputs)

    def wait(label, condition):
        deadline = monotonic() + 3.0
        while monotonic() < deadline:
            if gate.poll() is not None:
                raise RuntimeError("gate exited")
            rclpy.spin_once(node, timeout_sec=0.04)
            if condition():
                return
        raise RuntimeError(f"{label} timed out. Gate={states[-5:]}, search calls={search_calls}")

    def dwell(seconds):
        end = monotonic() + seconds
        wait("observation interval", lambda: monotonic() >= end)

    def call(client):
        wait("service discovery", client.service_is_ready)
        response = client.call_async(Trigger.Request())
        wait("trigger response", response.done)
        return response.result().success

    def require(condition, reason):
        if not condition:
            raise RuntimeError(reason)

    wait("OPEN before fault", lambda: states and states[-1] == "OPEN")
    require(not call(resume), "resume unexpectedly accepted at startup")
    health = "lost"
    wait("HOLD on loss", lambda: states[-1] == "HOLD localization_lost")
    require(call(recover), "recovery start refused")
    wait("repeat the same search stamp", lambda: search_calls >= 4)
    require(not seeds and not call(resume), "duplicated scan seeded AMCL or allowed resume")
    varying_candidates = True
    wait("three fresh scans and AMCL seed", lambda: len(seeds) == 1)
    health = "tracking"
    start_states = len(states)
    dwell(0.7)
    require("READY" not in states[start_states:] and not call(resume), "bad AMCL covariance unlocked motion")
    frozen_report = clock.now().to_msg()
    covariance = 0.01
    wait("first good AMCL estimate", lambda: good_estimates > 0)
    frozen_report = clock.now().to_msg()
    dwell(0.7)
    require(states[-1] == "RECOVERING verifying" and not call(resume), "a repeated scan unlocked motion")
    frozen_report = None
    wait("READY on post-AMCL scan streak", lambda: states[-1] == "READY")
    frozen_report = clock.now().to_msg()
    wait("READY expires when scans stop", lambda: states[-1] != "READY")
    require(not call(resume), "stale evidence unlocked motion")
    frozen_report = None
    wait("fresh proof renews READY", lambda: states[-1] == "READY")
    command_stamp = clock.now().to_msg()
    require(call(resume), "verified gate did not open")
    wait("released OPEN", lambda: states[-1] == "OPEN")
    start_velocity = len(velocities)
    send_commands = True
    dwell(0.25)
    require(not any(velocities[start_velocity:]), "gate forwarded a pre-release command")
    command_stamp = None
    wait("new command forwarded", lambda: velocities and velocities[-1] == 0.4)
    print("PASS  duplicate hypotheses, high covariance, duplicate scans, stale proof and queued velocity.")

    mission_status.publish(String(data="NEEDS_HELP deliver retry_limit"))
    wait("terminal mission hold", lambda: states[-1] == "NEEDS_HELP retry_limit")
    wait("zero velocity after terminal hold", lambda: velocities[-1] == 0.0)
    start_velocity = len(velocities)
    require(not call(recover) and not call(resume), "terminal gate accepted a recovery request")
    dwell(0.6)
    require(
        len(velocities[start_velocity:]) >= 5 and not any(velocities[start_velocity:])
        and states[-1] == "NEEDS_HELP retry_limit",
        "fresh matching scans opened a terminal hold",
    )
    print("PASS  terminal mission status closes an open gate and rejects both recovery services.")


def main():
    namespace = "aisleops_verify_" + uuid4().hex
    with TemporaryDirectory(prefix="aisleops-recovery-proof-") as folder, ExitStack() as stack:
        os.environ.update(
            ROS_DOMAIN_ID=str(170 + secrets.randbelow(40)),
            ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", ROS_LOG_DIR=folder,
        )
        os.environ.pop("ROS_STATIC_PEERS", None)
        os.environ.pop("ROS_LOCALHOST_ONLY", None)
        gate = stack.enter_context(start(
            ["ros2", "run", "aisleops_mission", "localization_gate", "--ros-args",
             "-r", f"__ns:=/{namespace}", "-p", "recover_localization:=true"],
            Path(folder) / "gate.log", ROOT,
        ))
        rclpy.init()
        try:
            node = rclpy.create_node("verify_recovery", namespace=namespace)
            try:
                verify(node, gate)
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
