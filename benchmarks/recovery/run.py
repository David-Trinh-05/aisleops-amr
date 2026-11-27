"""Run paired warehouse simulations and record both trial and fault metrics."""

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from contextlib import ExitStack
import csv
from hashlib import sha256
import json
import math
import os
from pathlib import Path
from random import Random
import re
import subprocess
import sys
from time import monotonic
from uuid import uuid4

from process import start

from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

BENCHMARK = Path(__file__).resolve().parent
ROOT = BENCHMARK.parents[1]
STATIONS = {"receive": (0.0, -3.5), "deliver": (9.0, 3.5), "return": (0.0, 0.0)}
DEFAULT_SEEDS = (17, 23, 41, 53, 67, 89)
sys.path.insert(0, str(ROOT / "scripts"))
from trajectory import SPAWN_X, interpolate, sample, wrap

SCENARIOS = {
    "nominal": ("landmarks", ()),
    "receive": ("landmarks", (("receive", 4.45, 0.25, -1.10, True),)),
    "deliver": ("landmarks", (("deliver", 4.45, 0.25, -1.10, True),)),
    "symmetric": ("symmetric", (("receive", 0.50, 0.25, -math.pi / 2, False),)),
    "repeat": ("landmarks", (("receive", 4.45, 0.25, -1.10, True),
                             ("receive", -3.65, -1.80, 0.75, False))),
}
BASE_SCENARIOS = tuple(SCENARIOS)
SCENARIOS.update({
    "receive-offset": ("landmarks", (("receive", 3.8, 0.48, -0.70, True),)),
    "deliver-offset": ("landmarks", (("deliver", 5.75, -0.95, 1.80, True),)),
})
MODES = ("hold", "recover")
SECOND = 1_000_000_000
XY_LIMIT = 0.25
YAW_LIMIT = 0.22


def stamp_ns(header):
    return header.stamp.sec * SECOND + header.stamp.nanosec


def fault_plan(scenario, seed):
    rng = Random(seed)
    return [dict(leg=leg, x=x + rng.uniform(-0.045, 0.045), y=y + rng.uniform(-0.04, 0.04),
                 yaw=yaw + rng.uniform(-0.025, 0.025), resumable=resumable,
                 delay_s=rng.uniform(4.0, 4.5)) for leg, x, y, yaw, resumable in SCENARIOS[scenario][1]]


class Recorder:
    def __init__(self, node, executor):
        self.node = node
        self.executor = executor
        self.truth, self.amcl, self.odom = [], [], []
        self.events, self.gate, self.health, self.commands = [], [], [], []
        self.goals = {}
        self.start_ns = 0
        self.active_leg = None
        self.leg_started_ns = 0
        durable = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(PoseStamped, "evaluation/ground_truth", self.record_truth, qos_profile_sensor_data)
        node.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.record_pose, 20)
        node.create_subscription(Odometry, "odom", self.record_odom, qos_profile_sensor_data)
        node.create_subscription(String, "mission_status", self.record_event, durable)
        node.create_subscription(String, "localization/gate_state", self.record_gate, durable)
        node.create_subscription(DiagnosticArray, "localization/scan_agreement", self.record_health, 20)
        node.create_subscription(TwistStamped, "cmd_vel_guarded", self.record_command, 30)
        node.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status", self.record_goals, durable)

    def now(self):
        return self.node.get_clock().now().nanoseconds

    def spin(self):
        self.executor.spin_once(timeout_sec=0.04)

    def record_truth(self, message):
        if message.header.frame_id != "warehouse":
            raise RuntimeError("Unexpected ground-truth frame")
        self.truth.append(sample(message.header, message.pose))

    def record_pose(self, message):
        if message.header.frame_id != "map":
            raise RuntimeError("Unexpected AMCL frame")
        self.amcl.append(sample(message.header, message.pose.pose))

    def record_odom(self, message):
        twist = message.twist.twist
        self.odom.append((stamp_ns(message.header), math.hypot(twist.linear.x, twist.linear.y), abs(twist.angular.z)))

    def record_event(self, message):
        now = self.now()
        self.events.append((now, message.data))
        label, _, part = message.data.partition(" ")
        if label == "STARTED":
            self.active_leg, self.leg_started_ns = part, now
            self.start_ns = self.start_ns or now
        elif label in {"HOLD", "REACHED", "NEEDS_HELP", "COMPLETED", "FAILED"}:
            self.active_leg = None

    def record_gate(self, message):
        if not self.gate or self.gate[-1][1] != message.data:
            self.gate.append((self.now(), message.data))

    def record_health(self, message):
        if len(message.status) == 1:
            values = {value.key: value.value for value in message.status[0].values}
            self.health.append((stamp_ns(message.header), values.get("health_state", "unavailable")))

    def record_command(self, message):
        twist = message.twist
        speed = max(math.hypot(twist.linear.x, twist.linear.y, twist.linear.z),
                    math.hypot(twist.angular.x, twist.angular.y, twist.angular.z))
        self.commands.append((stamp_ns(message.header), speed))

    def record_goals(self, message):
        for entry in message.status_list:
            self.goals[bytes(entry.goal_info.goal_id.uuid)] = entry.status


def teleport(rec, fault):
    yaw = fault["yaw"]
    payload = (f'name:"aisleops" position:{{x:{fault["x"]} y:{fault["y"]} z:0.005}} '
               f'orientation:{{z:{math.sin(yaw / 2)} w:{math.cos(yaw / 2)}}}')
    sent_ns = rec.now()
    with subprocess.Popen(
        ["gz", "service", "-s", "/world/warehouse/set_pose", "--reqtype", "gz.msgs.Pose",
         "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", payload],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    ) as proc:
        deadline = monotonic() + 6
        while proc.poll() is None and monotonic() < deadline:
            rec.spin()
        if proc.poll() is None:
            proc.kill()
        stdout, stderr = proc.communicate()
        if proc.returncode or not re.search(r"\bdata:\s*true\b", stdout):
            raise RuntimeError(f"Gazebo did not accept relocation ({stdout} {stderr})")
    return sent_ns


def wait_for_nav2(rec, simulator):
    client = rec.node.create_client(Trigger, "lifecycle_manager_nav2/is_active")
    deadline = monotonic() + 35
    probe = None
    sent = 0.0
    try:
        while monotonic() < deadline and simulator.poll() is None:
            rec.spin()
            now = monotonic()
            if probe is not None and probe.done():
                result = probe.result()
                if result and result.success:
                    return
                probe = None
            elif probe is not None and now - sent > 1.5:
                client.remove_pending_request(probe)
                probe = None
            if (probe is None and now - sent >= 0.25 and rec.truth and rec.now() > 0
                    and client.service_is_ready()):
                probe = client.call_async(Trigger.Request())
                sent = now
    finally:
        rec.node.destroy_client(client)
    raise RuntimeError("Nav2 lifecycle, simulation clock or ground truth was unavailable at startup")


def observe(rec, simulator, mission, plan, recovery):
    sent = []
    terminal = None
    process_exit_at = None
    wall_deadline = monotonic() + 180
    while monotonic() < wall_deadline:
        if simulator.poll() is not None:
            raise RuntimeError("Warehouse launch exited during trial")
        rec.spin()
        now = rec.now()
        if rec.events and terminal is None:
            stamp, event = rec.events[-1]
            if event == "COMPLETED":
                terminal = (stamp, "completed", "")
            elif event.startswith("NEEDS_HELP "):
                terminal = (stamp, "needs_help", event.split(" ", 2)[2])
            elif event.startswith("HOLD ") and not recovery:
                terminal = (stamp, "held", "hold_only")
            elif event.startswith("FAILED "):
                terminal = (stamp, "mission_failed", event[7:])
        if terminal:
            if now - terminal[0] >= 3 * SECOND:
                return terminal, sent
            continue
        if mission.poll() is not None:
            process_exit_at = process_exit_at or monotonic()
            if monotonic() - process_exit_at >= 1:
                return (now, "mission_failed", f"exit_{mission.returncode}"), sent
            continue
        if rec.start_ns and now - rec.start_ns > 125 * SECOND:
            return (now, "deadline", "mission_sim_time"), sent
        if len(sent) == len(plan):
            continue
        fault = plan[len(sent)]
        prior_request = sent[-1]["request_ns"] if sent else 0
        if (rec.active_leg == fault["leg"] and rec.leg_started_ns > prior_request
                and now - rec.leg_started_ns >= int(fault["delay_s"] * SECOND)
                and rec.odom and max(rec.odom[-1][1:]) > 0.20 and rec.health
                and rec.health[-1][1] == "tracking"):
            sent.append(dict(fault, request_ns=teleport(rec, fault)))
    if not rec.truth or not rec.start_ns:
        raise RuntimeError("Trial did not reach a running mission with ground truth")
    if terminal:
        raise RuntimeError("Simulator stopped advancing after a terminal mission event")
    return (rec.now(), "deadline", "wall_time"), sent


def first_settled(odom, start_ns, end_ns):
    first = None
    previous = 0
    for stamp, linear, angular in odom:
        if stamp < start_ns or stamp >= end_ns or stamp <= previous:
            continue
        if stamp - previous > 150_000_000:
            first = None
        previous = stamp
        if linear >= 0.02 or angular >= 0.03:
            first = None
            continue
        first = stamp if first is None else first
        if stamp - first >= 250_000_000:
            return first
    return None


def elapsed(end, start):
    return (end - start) / SECOND if end is not None and start is not None else None


def pose_error(estimate, actual):
    return (math.hypot(estimate.x + SPAWN_X - actual.x, estimate.y - actual.y),
            abs(wrap(estimate.yaw - actual.yaw)))


def score(rec, terminal, sent, plan):
    end, outcome, reason = terminal
    truth = sorted({pose.stamp_ns: pose for pose in rec.truth}.values())
    stamps = [pose.stamp_ns for pose in truth]
    comparisons = []
    for estimate in sorted({pose.stamp_ns: pose for pose in rec.amcl}.values()):
        if rec.start_ns <= estimate.stamp_ns <= end:
            actual = interpolate(truth, stamps, estimate.stamp_ns)
            if actual is not None:
                comparisons.append((estimate.stamp_ns, *pose_error(estimate, actual)))

    faults = []
    consumed_holds = set()
    for index, fault in enumerate(sent):
        next_request = sent[index + 1]["request_ns"] if index + 1 < len(sent) else rec.now() + 1
        jump = next((b for a, b in zip(truth, truth[1:])
                     if fault["request_ns"] <= b.stamp_ns < next_request
                     and math.hypot(b.x - a.x, b.y - a.y) >= 1.5), None)
        if jump is None:
            raise RuntimeError(f"Accepted relocation had no ground-truth jump for {fault['leg']}")
        stamp = jump.stamp_ns
        lost = next((t for t, state in rec.health if stamp <= t < next_request and state == "lost"), None)
        held = next((t for t, state in rec.gate
                     if stamp <= t <= stamp + 2 * SECOND and state.startswith("HOLD ")), None)
        if held is not None:
            consumed_holds.add(held)
        reopened = next((t for t, state in rec.gate
                         if held is not None and held < t < next_request and state == "OPEN"), None)
        zero = next((t for t, speed in rec.commands
                     if lost is not None and lost <= t < next_request and speed == 0.0), None)
        settled = first_settled(rec.odom, lost if lost is not None else stamp, reopened or next_request)
        before_open = next((row for row in reversed(comparisons)
                            if reopened is not None and max(stamp, reopened - SECOND // 2) <= row[0] <= reopened), None)
        xy, yaw = before_open[1:] if before_open else (None, None)
        bad_resume = reopened is not None and (not fault["resumable"] or
                     (xy is not None and (xy > XY_LIMIT or yaw > YAW_LIMIT)))
        faults.append(dict(fault, number=index + 1, jump_ns=stamp, lost_ns=lost, held_ns=held,
                           reopened_ns=reopened, detect_s=elapsed(lost, stamp), zero_s=elapsed(zero, lost),
                           settle_s=elapsed(settled, stamp), recover_s=elapsed(reopened, lost),
                           amcl_xy_at_open_m=xy, amcl_yaw_at_open_rad=yaw, false_resume=bad_resume,
                           unverified_resume=reopened is not None and before_open is None))

    arrivals = {}
    for timestamp, event in rec.events:
        if event.startswith("REACHED "):
            leg = event.split()[1]
            actual = interpolate(truth, stamps, timestamp)
            if actual is not None:
                x, y = STATIONS[leg]
                arrivals[leg] = (math.hypot(actual.x - SPAWN_X - x, actual.y - y), abs(wrap(actual.yaw)))

    def reached(leg):
        xy, yaw = arrivals.get(leg, (math.inf, math.inf))
        return xy <= XY_LIMIT and yaw <= YAW_LIMIT

    first_fault = faults[0]["jump_ns"] if faults else end
    before_fault = [row for row in comparisons if row[0] < first_fault]
    opened = next((t for t, state in rec.gate if state == "OPEN"), 0)
    false_stops = sum(opened <= t <= end and state.startswith("HOLD ") and t not in consumed_holds
                      for t, state in rec.gate)
    delivered = reached("deliver")
    completed = outcome == "completed" and all(reached(leg) for leg in STATIONS)
    return dict(outcome=outcome, reason=reason, duration_s=elapsed(end, rec.start_ns or None), delivered=delivered,
                completed=completed, injected=len(faults), planned=len(plan), canceled_goals=sum(
                    status == GoalStatus.STATUS_CANCELED for status in rec.goals.values()),
                false_stops=false_stops, false_resumes=sum(row["false_resume"] for row in faults),
                unverified_resumes=sum(row["unverified_resume"] for row in faults),
                matched_poses=len(comparisons), pre_fault_poses=len(before_fault),
                xy_rmse_m=math.sqrt(sum(row[1] ** 2 for row in comparisons) / len(comparisons)) if comparisons else None,
                yaw_rmse_rad=math.sqrt(sum(row[2] ** 2 for row in comparisons) / len(comparisons)) if comparisons else None,
                pre_fault_xy_rmse_m=math.sqrt(sum(row[1] ** 2 for row in before_fault) / len(before_fault)) if before_fault else None,
                arrivals=arrivals, events=rec.events, gate=rec.gate, faults=faults, fault_plan=plan)


def run_trial(repo, output, scenario, seed, mode, code_id):
    trial = output / scenario / str(seed) / mode
    trial.mkdir(parents=True, exist_ok=True)
    result_path = trial / "result.json"
    plan = fault_plan(scenario, seed)
    details = dict(scenario=scenario, seed=seed, mode=mode, code_id=code_id)
    if result_path.is_file():
        previous = json.loads(result_path.read_text())
        if any(previous.get(key) != value for key, value in details.items()):
            raise ValueError(f"Trial configuration changed for {trial}")
        if previous.get("fault_plan") != plan:
            raise ValueError(f"Fault plan changed for {trial}")
        if previous["outcome"] != "infrastructure_error":
            return previous
    log_dir = trial / f"attempt-{len(list(trial.glob('attempt-*'))) + 1:02d}"
    log_dir.mkdir()
    unique = uuid4().hex
    ns = "aisleops_bench_" + unique
    partition = "aisleops-bench-" + unique
    os.environ.update(GZ_PARTITION=partition, ROS_DOMAIN_ID=str(170 + int(unique[:4], 16) % 40),
                      ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", ROS_LOG_DIR=str(log_dir / "ros"))
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    for key in ("DISPLAY", "WAYLAND_DISPLAY", "ROS_STATIC_PEERS", "ROS_LOCALHOST_ONLY"):
        os.environ.pop(key, None)
    recover = str(mode == "recover").lower()
    fixture = []
    map_file = repo / "benchmarks/recovery/symmetric/warehouse.yaml"
    if SCENARIOS[scenario][0] == "landmarks":
        world_file = repo / "demo/landmarks.sdf"
        map_file = repo / "demo/landmarks.yaml"
        fixture = [f"world:={world_file}"]
    try:
        with ExitStack() as stack:
            simulator = stack.enter_context(start(
                ["ros2", "launch", "launch/warehouse.launch.py", f"namespace:={ns}", f"map:={map_file}",
                 f"gz_partition:={partition}", f"sim_seed:={seed}", "ground_truth:=true", "guard_localization:=true",
                 f"recover_localization:={recover}", *fixture], log_dir / "warehouse.log", repo,
            ))
            context = rclpy.Context()
            rclpy.init(context=context)
            stack.callback(rclpy.try_shutdown, context=context)
            node = rclpy.create_node(
                "benchmark", context=context, namespace=ns, cli_args=["--ros-args", "-r", "/clock:=clock"],
                parameter_overrides=[Parameter("use_sim_time", value=True)],
            )
            stack.callback(node.destroy_node)
            executor = SingleThreadedExecutor(context=context)
            stack.callback(executor.shutdown)
            executor.add_node(node)
            recorder = Recorder(node, executor)
            wait_for_nav2(recorder, simulator)
            mission = stack.enter_context(start(
                ["ros2", "run", "aisleops_mission", "delivery_mission", "--ros-args", "-r", f"__ns:=/{ns}",
                 "-r", f"/clock:=/{ns}/clock", "-p", "use_sim_time:=true", "-p", "guard_localization:=true",
                 "-p", f"recover_localization:={recover}"], log_dir / "mission.log", repo,
            ))
            terminal, sent = observe(recorder, simulator, mission, plan, mode == "recover")
            result = dict(details, **score(recorder, terminal, sent, plan))
    except RuntimeError as error:
        result = dict(details, outcome="infrastructure_error", reason=str(error), fault_plan=plan)
    temporary = result_path.with_suffix(".tmp")
    document = json.dumps(result, indent=2, allow_nan=False) + "\n"
    (log_dir / "result.json").write_text(document)
    temporary.write_text(document)
    temporary.replace(result_path)
    return result


def run_pair(repo, output, scenario, seed, code_id):
    modes = MODES if (seed + tuple(SCENARIOS).index(scenario)) % 2 == 0 else MODES[::-1]
    pair = []
    for mode in modes:
        result = run_trial(repo, output, scenario, seed, mode, code_id)
        if result["outcome"] == "infrastructure_error":
            print(f"{scenario} seed {seed}, {mode}: retrying after {result['reason']}", flush=True)
            result = run_trial(repo, output, scenario, seed, mode, code_id)
        pair.append(result)
    return pair


def write_csv(path, rows, columns):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def summarize(output, rows):
    rows.sort(key=lambda row: (tuple(SCENARIOS).index(row["scenario"]), row["seed"], row["mode"]))
    keys = ("scenario", "seed", "mode")
    write_csv(output / "runs.csv", rows, (*keys, "outcome", "reason", "planned", "injected", "delivered",
              "completed", "duration_s", "canceled_goals", "false_stops", "false_resumes", "unverified_resumes",
              "matched_poses", "pre_fault_poses", "xy_rmse_m", "yaw_rmse_rad", "pre_fault_xy_rmse_m"))
    faults = [dict(fault, **{key: row[key] for key in keys}) for row in rows for fault in row.get("faults", [])]
    write_csv(output / "faults.csv", faults, (*keys, "number", "leg", "x", "y", "yaw", "delay_s", "resumable",
              "jump_ns", "lost_ns", "held_ns", "reopened_ns", "detect_s", "zero_s", "settle_s", "recover_s",
              "amcl_xy_at_open_m", "amcl_yaw_at_open_rad", "false_resume", "unverified_resume"))
    pairs = {}
    for row in rows:
        pairs.setdefault((row["scenario"], row["seed"]), {})[row["mode"]] = row
    valid = [pair for pair in pairs.values() if len(pair) == 2 and all(
        row["outcome"] != "infrastructure_error" for row in pair.values())]
    lines = ["# Paired warehouse benchmark", "", f"Valid pairs: {len(valid)}/{len(pairs)}.", "",
             "| Scenario | Seed | Hold only | Recovery |", "| --- | ---: | --- | --- |"]
    for (scenario, seed), pair in pairs.items():
        statuses = [pair.get(mode, {}).get("outcome", "missing") for mode in MODES]
        lines.append(f"| {scenario} | {seed} | {statuses[0]} | {statuses[1]} |")
    lines.extend(("", "| Mode | Delivered | Completed | False stops | False resumes | Unverified resumes |",
                  "| --- | ---: | ---: | ---: | ---: | ---: |"))
    for mode in MODES:
        attempts = [pair[mode] for pair in valid]
        count = len(attempts)
        totals = [sum(trial[key] for trial in attempts) for key in (
            "delivered", "completed", "false_stops", "false_resumes", "unverified_resumes")]
        lines.append(f"| {mode} | {totals[0]}/{count} | {totals[1]}/{count} | {totals[2]} | {totals[3]} | {totals[4]} |")
    (output / "summary.md").write_text("\n".join(lines) + "\n")


def code_identity(repo):
    names = subprocess.check_output(
        ["git", "ls-files", "-z", "--", "launch", "config", "worlds", "description", "src", "demo/landmarks.sdf",
         "demo/landmarks.yaml", "demo/landmarks.pgm", "benchmarks/recovery/symmetric"], cwd=repo,
    )
    digest = sha256(Path(__file__).read_bytes() + names)
    for name in names.decode().split("\0")[:-1]:
        digest.update(sha256((repo / name).read_bytes()).digest())
    for name in ("scripts/trajectory.py", "benchmarks/recovery/process.py"):
        digest.update(sha256((ROOT / name).read_bytes()).digest())
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=ROOT, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seeds", default=DEFAULT_SEEDS, type=int, nargs="+")
    parser.add_argument("--scenarios", default=BASE_SCENARIOS, choices=SCENARIOS, nargs="+")
    parser.add_argument("--jobs", default=2, type=int)
    args = parser.parse_args()
    if args.jobs < 1 or any(not 0 <= seed <= 2_147_483_647 for seed in args.seeds):
        parser.error("jobs must be positive and seeds must be between 0 and 2147483647")
    jobs = [(case, seed) for case in dict.fromkeys(args.scenarios) for seed in dict.fromkeys(args.seeds)]
    args.output.mkdir(parents=True, exist_ok=True)
    output, repo = args.output.resolve(), args.repo.resolve()
    code_id = code_identity(repo)
    rows = []
    with ProcessPoolExecutor(max_workers=min(args.jobs, len(jobs))) as executor:
        futures = [executor.submit(run_pair, repo, output, case, seed, code_id) for case, seed in jobs]
        for future in as_completed(futures):
            pair = future.result()
            rows.extend(pair)
            summarize(output, rows)
            print(f"{pair[0]['scenario']} seed {pair[0]['seed']}: " + ", ".join(
                f"{row['mode']}={row['outcome']}" for row in pair), flush=True)
    return int(any(row["outcome"] == "infrastructure_error" for row in rows))


if __name__ == "__main__":
    sys.exit(main())
