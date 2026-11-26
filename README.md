# AisleOps AMR

A ROS 2 simulation for evaluating an autonomous mobile robot in a warehouse. It includes a custom differential-drive robot, 2D LiDAR and IMU, a three-leg Nav2 delivery mission, and an evaluation tool that measures wheel odometry against Gazebo ground truth.

[Watch the recovery demo](demo/recovery.mp4) · [Run it from a clone and inspect the recorded bag](demo/README.md)

## Run the warehouse simulation

The custom differential-drive AMR uses a Xacro description with a 0.48 × 0.34 m chassis, 0.410 m between wheel centers, 2D LiDAR and IMU. The 14 × 10 m warehouse has two shelf rows and marked receiving and delivery bays. All geometry is local. [Gazebo URDF spawning](https://gazebosim.org/docs/harmonic/spawn_urdf/) · [DiffDrive system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)

Use Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. The headless LiDAR uses an EGL-capable renderer. [ROS and Gazebo compatibility](https://gazebosim.org/docs/harmonic/ros_installation/) · [Gazebo headless rendering](https://gazebosim.org/api/sim/8/headless_rendering.html)

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros-py ros-jazzy-rclpy
source /opt/ros/jazzy/setup.bash
ros2 launch launch/warehouse.launch.py gui:=true
```

The default ROS namespace is `/aisleops`. Send motion commands on `/aisleops/cmd_vel` as `geometry_msgs/msg/TwistStamped`. The simulation publishes `scan`, `imu` and `odom` in the same namespace. [ROS–Gazebo bridge documentation](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md)

The warehouse launch defaults Gazebo Transport to loopback (`127.0.0.1`) so Gazebo, the spawn process and the ROS bridge can discover each other without relying on a network adapter. For an external `gz` command, export `GZ_IP=127.0.0.1` and use the same `GZ_PARTITION` value passed to the launch as `gz_partition:=<name>`. To run Gazebo Transport across hosts, set `GZ_IP` to the appropriate interface before launching. [Gazebo Transport environment variables](https://gazebosim.org/api/transport/13/envvars.html)

## Build and save a warehouse map

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server ros-jazzy-teleop-twist-keyboard
source /opt/ros/jazzy/setup.bash
ros2 launch launch/warehouse.launch.py slam:=true gui:=true
```

SLAM Toolbox uses simulated `/aisleops/clock`, the namespaced 2D LiDAR scan and robot TF. It publishes an occupancy grid at `/aisleops/map` with 0.05 m cells and supplies the `map → odom` transform. [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) · [Nav2 mapping guide](https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/)

In a second terminal, source ROS and drive the robot around both shelf rows and through the center aisle. Stop the robot and exit teleop before saving the grid.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -r cmd_vel:=/aisleops/cmd_vel
mkdir -p ~/aisleops-maps
map_dir=$(mktemp -d ~/aisleops-maps/warehouse.XXXXXX)
ros2 run nav2_map_server map_saver_cli -t /aisleops/map \
  -f "$map_dir/warehouse" --fmt pgm --mode trinary
```

The saver writes `warehouse.yaml` and `warehouse.pgm`. Keep the two files together when loading them into Nav2 Map Server. A headless run produced a **278 × 199** grid that Nav2 Map Server restored cell for cell. [Nav2 Map Server](https://api.nav2.org/nav2-jazzy/html/md_nav2_map_server_README.html)

## Navigate on a saved map

Use a map built from this warehouse and spawn pose. Passing `map:=` loads the saved map, starts AMCL at the known simulation spawn `(0, 0, 0)` in the map frame, and activates Nav2. Run this in a fresh session after stopping the mapping launch. [AMCL configuration](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/)

```bash
sudo apt install ros-jazzy-navigation2
source /opt/ros/jazzy/setup.bash
ros2 launch launch/warehouse.launch.py map:=/path/to/warehouse.yaml gui:=true
```

After the lifecycle manager reports that the managed nodes are active, send the delivery-bay goal from another terminal with the ROS environment sourced.

```bash
ros2 action send_goal /aisleops/navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 9.0, y: 3.5}, orientation: {w: 1.0}}}}'
```

NavFn plans through known free space, and Regulated Pure Pursuit follows the path. The local costmap uses live LiDAR and the robot footprint. Velocity commands pass through the velocity smoother and Collision Monitor to the Gazebo bridge as `TwistStamped`. [Nav2 controller server](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_controller_server/) · [Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_collision_monitor_node/)

A headless run reached the delivery bay in **20.35 s** of simulation time. The settled Gazebo pose was **5.0 cm** from the goal, and AMCL had **5.0 cm XY RMSE** over 88 matched samples.

## Run a delivery mission

Keep the saved-map Nav2 launch running. In another terminal, build and run the C++ mission client.

```bash
sudo apt install python3-colcon-common-extensions ros-jazzy-rosidl-default-generators
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to aisleops_mission \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
ros2 run aisleops_mission delivery_mission --ros-args \
  -r __ns:=/aisleops -r /clock:=/aisleops/clock -p use_sim_time:=true
```

The client waits for Nav2 to become active, then sends a `NavigateToPose` goal for each stop. A leg advances after Nav2 reports success and fresh odometry shows that the robot has settled. The node logs progress and publishes it on `/aisleops/mission_status`. A deadline or Ctrl+C requests cancellation of the active goal and ends the mission. [ROS 2 actions](https://design.ros2.org/articles/actions.html)

| Leg | Pose in `map` | ROS parameter |
| --- | --- | --- |
| Receive | `[0.0, -3.5, 0.0]` | `receive_pose` |
| Deliver | `[9.0, 3.5, 0.0]` | `deliver_pose` |
| Return | `[0.0, 0.0, 0.0]` | `return_pose` |

The pose arrays use meters and radians in `[x, y, yaw]` order. `leg_timeout_sec` is the wall-clock deadline after goal acceptance and defaults to 75 seconds.

A headless run completed all three goals in about **64 seconds** of simulation time. The settled positions measured in Gazebo were within **14 cm** of their targets.

## Monitor LiDAR agreement with the map

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select aisleops_localization \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
ros2 launch launch/warehouse.launch.py map:=/path/to/warehouse.yaml scan_monitor:=true
```

In another sourced terminal, read the monitor output.

```bash
ros2 topic echo /aisleops/localization/scan_agreement diagnostic_msgs/msg/DiagnosticArray
```

The monitor uses TF at the scan timestamp to project finite LiDAR endpoints into the occupancy grid. An endpoint matches if an occupied cell center is within **0.18 m**. It reports the matched fraction of comparable endpoints alongside coverage of the finite returns. The default thresholds are 30 comparable endpoints, 0.55 coverage and 0.65 agreement. [LaserScan message](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/LaserScan.msg) · [OccupancyGrid message](https://github.com/ros2/common_interfaces/blob/jazzy/nav_msgs/msg/OccupancyGrid.msg)

The diagnostic distinguishes `matching`, `low_agreement`, `insufficient_overlap` and stale inputs. A full nominal delivery mission reported **639/639** matching scans, with a minimum agreement of **0.9666**. In a separate test, shifting the monitor's reference map by 0.65 m on each axis gave a median agreement of **0.1413** across 10 scans. Similar aisle geometry can produce high agreement for several poses.

The same diagnostic carries a `health_state` for consumers that need evidence across scans. A low scan marks the estimate `suspect`. Three consecutive low scans confirm `lost`, while five matching scans establish or restore `tracking`. Missing or unsupported measurements report `unavailable`, preserving any confirmed loss until matching evidence returns.

In one simulation experiment, Gazebo relocated the robot body by **5.11 m** during the receiving leg. Wheel odometry changed **1.2 cm** over the same 20 ms interval. The monitor reported `lost` after **0.20 s** of simulation time.

## Hold motion after localization loss

Build `aisleops_mission` and `aisleops_localization` in the same ROS workspace. Pass `guard_localization:=true` to both the warehouse launch and the delivery mission.

```bash
ros2 launch launch/warehouse.launch.py map:=/path/to/warehouse.yaml guard_localization:=true
```

```bash
ros2 run aisleops_mission delivery_mission --ros-args \
  -r __ns:=/aisleops -r /clock:=/aisleops/clock -p use_sim_time:=true \
  -p guard_localization:=true
```

The launch places a velocity gate between Collision Monitor and the Gazebo bridge. It opens when scan health reaches `tracking`. Confirmed loss, unavailable localization, or 750 ms without a fresh health update latches `HOLD`. The gate sends zero velocity and the mission cancels its current goal. In hold-only mode, the pending leg is reported on `/aisleops/mission_status` and remains held until the session ends.

In a simulated relocation during the receiving leg, the first zero command followed the `lost` scan timestamp by **16 ms**. Nav2 confirmed cancellation. After settling, the gate sent **40 zero commands** during two more simulated seconds of injected nonzero input.

## Search for localization candidates

Build `aisleops_localization` and `aisleops_mission` with the ROS 2 Jazzy interface generators installed (`ros-jazzy-rosidl-default-generators`). Launch against a saved map, then call the service once the robot has settled.

```bash
ros2 launch launch/warehouse.launch.py map:=/path/to/warehouse.yaml \
  guard_localization:=true pose_search:=true
```

```bash
ros2 service call /aisleops/localization/search_pose aisleops_localization/srv/SearchPose '{}'
```

The service searches free positions and headings across the occupancy grid using a single recent, stationary LiDAR scan. It uses the sensor mounting transform and wheel velocity, so it can run independently of the current global pose estimate. It returns up to five distinct poses in the `map` frame at the scan timestamp, with scores in descending order. The scores measure how closely the projected LiDAR endpoints align with occupied cells. The search supports maps up to 30 m per side and 500,000 cells at 0.02–0.20 m per cell.

| Status | Meaning |
| --- | --- |
| `CANDIDATE` (`1`) | The top score is at least 0.60 and leads the next distinct pose by at least 0.06, or is the only result. |
| `AMBIGUOUS` (`2`) | The top score clears 0.60 and two distinct poses are within 0.06 of each other. |
| `NO_MATCH` (`3`) | Too few valid returns or the search found no pose scoring at least 0.60. |
| `UNAVAILABLE` (`4`) | The search needs a valid map, recent stationary scan, recent wheel odometry and a scan-to-base transform. |

In a relocation trial, the two best fits differed by **0.0020** in score. The pose closest to Gazebo ground truth ranked second, within **6.1 cm** and **0.001 rad** of the true pose. The service returned `AMBIGUOUS` and the mission remained in `HOLD`. The response provides pose hypotheses for inspection while AMCL and the velocity gate retain control of localization and motion.

## Recover localization and resume a delivery

Build through `aisleops_mission` as above and source the workspace in both terminals. Enable recovery on the launch and on the mission client.

```bash
ros2 launch launch/warehouse.launch.py map:=/path/to/warehouse.yaml recover_localization:=true
```

```bash
ros2 run aisleops_mission delivery_mission --ros-args \
  -r __ns:=/aisleops -r /clock:=/aisleops/clock -p use_sim_time:=true \
  -p recover_localization:=true
```

Watch `/aisleops/localization/gate_state` and `/aisleops/mission_status` with `ros2 topic echo`. After a localization fault, the gate holds zero velocity and the mission waits for its old Nav2 goal to reach a terminal state. It then requests recovery for the pending leg.

1. Three distinct stationary scans must support a unique candidate within 0.20 m and 0.15 rad of the first. The gate seeds AMCL with the last candidate and requests filter updates while stationary. [Nav2 Jazzy AMCL initial pose and no-motion services](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L1473-L1485)
2. A fresh AMCL estimate must agree with the seed within 0.25 m and 0.20 rad with bounded covariance. The gate reports `READY` after five fresh matching scans following the verified estimate. `READY` authorizes a resume request while the evidence is fresh.
3. Once the gate opens, the mission announces `RESUMED <leg>` and sends a new goal for that same leg. A leg gets one automatic retry. The gate rejects velocity messages stamped before it reopened.

Ambiguous evidence, conflicting candidates or a recovery timeout put the gate in `NEEDS_HELP` and preserve zero velocity. The stock warehouse has symmetric geometry that can trigger this outcome. For a warehouse with distinct landmarks, `world:=/path/to/warehouse.sdf` accepts an SDF containing a world named `warehouse` with the standard robot spawn and matching saved map. The same option can be used when building that map with `slam:=true`.

If a retried leg loses localization again, the mission cancels the replacement goal and reports `NEEDS_HELP <leg> retry_limit`. The gate latches that terminal state, publishes zero velocity, and rejects both `localization/recover` and `localization/resume`. Start a new simulation and mission to clear the terminal hold.

In a Gazebo trial with two additional static landmarks, the gate held zero for **52 commands** and reopened **2.58 s** after confirmed loss. The last verified AMCL pose before release was **3.0 cm** and **0.018 rad** from ground truth. The mission canceled the first receiving goal and then completed receiving, delivery and return in order. In the symmetric warehouse, recovery reported `NEEDS_HELP`, rejected a resume request and continued publishing zero velocity under injected motion commands.

A separate trial relocated the robot during both delivery and return within one mission. The gate reopened **2.63 s** and **2.26 s** after confirmed loss, respectively, and each leg completed once. A second relocation on the same receiving leg produced `NEEDS_HELP retry_limit`. Over two simulated seconds of injected commands, the gate published **40 zero commands** and measured body drift was **0.2 mm**.

## Paired simulation comparison

Pass `sim_seed:=17` to the warehouse launch to seed Gazebo. The trials pair a hold-only run and a recovery-enabled run using the same map, Gazebo seed, and planned relocation. The five cases cover a nominal mission, one relocation during receiving, one during delivery, a symmetric warehouse, and two relocations on the receiving leg. The second relocation is triggered only after a replacement goal starts.

Delivery is scored against Gazebo ground truth at the reported arrival, within 0.25 m and 0.22 rad of the station. Completion applies the same check to all three stops. Detection, zero command, wheel settling and reopening delays use simulation time. The Jazzy AMCL particle filter seeds itself from wall time, so the pairing controls the planned inputs while AMCL and message scheduling can vary. [Nav2 Jazzy particle filter initialization](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/pf/pf.c#L43-L60)

The benchmark completed **30 valid pairs (60 runs)** across Gazebo seeds 17, 23, 41, 53, 67 and 89.

| Scenario | Hold-only delivered | Recovery delivered | Hold-only completed | Recovery completed |
| --- | ---: | ---: | ---: | ---: |
| Nominal | 5/6 | 6/6 | 5/6 | 6/6 |
| Relocation during receiving | 0/6 | 5/6 | 0/6 | 4/6 |
| Relocation during delivery | 0/6 | 5/6 | 0/6 | 3/6 |
| Symmetric warehouse | 0/6 | 0/6 | 0/6 | 0/6 |
| Repeated relocation | 0/6 | 0/6 | 0/6 | 0/6 |
| **Total** | **5/30** | **16/30** | **5/30** | **13/30** |

Recovery increased the observed delivery rate by **36.7 percentage points** and mission completion by **26.7 points**. All **52 injected relocations** were detected. In both modes the median was **0.240 s** from relocation to confirmed loss, **0.012 s** from loss to zero command and **1.25 s** from relocation to wheel settling. The **18 verified reopenings** took a median **2.67 s** from confirmed loss (P95 **2.69 s**). At reopening the median AMCL position error was **1.87 cm** (P95 **4.59 cm**) and median yaw error was **0.59°** (P95 **1.47°**).

The benchmark observed **0 false resumes in 18 reopenings** and false stops in **2/30 hold-only** and **6/30 recovery-enabled** runs. All eight false stops reported `localization_stale` without a matching relocation. This prevented injection of the first planned relocation in one hold-only run and the second relocation in one recovery-enabled run. All six symmetric trials with recovery reported `NEEDS_HELP ambiguous_pose`. The five recovery-enabled repeat trials that received both relocations reported `NEEDS_HELP retry_limit` after the second relocation.

A follow-up with loopback transport repeated all five scenarios on seeds 53, 67 and 89. The same 30 trial slots had **eight `localization_stale` stops** in the original run and **zero** in the follow-up. Recovery completed **9/15** missions and hold-only completed **3/15**. One hold-only run timed out waiting for the Nav2 planner to acknowledge a goal before the relocation was injected.

## Measure odometry error

```bash
/usr/bin/python3 scripts/smoke.py --trajectory "/tmp/aisleops-$(date +%s)-$$.csv"
```

The runner drives a timed L route of about 1 m forward, a 90° left turn, then 1 m forward. Gazebo publishes ground truth at 50 Hz on an evaluation topic enabled with `ground_truth:=true`. The runner interpolates ground truth at each odometry timestamp and applies the fixed spawn transform to compare both poses in the warehouse frame. It rejects extrapolation and gaps over 100 ms in ground truth. [Gazebo Harmonic PosePublisher source](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/systems/pose_publisher/PosePublisher.cc)

The Gazebo DiffDrive plugin uses an effective rolling track of 0.365 m, estimated from a stationary turn on the warehouse floor. It uses this value to generate wheel commands and integrate odometry. [Gazebo Harmonic DiffDrive source](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/systems/diff_drive/DiffDrive.cc)

In Gazebo Harmonic 8.11.0, the L route paired 363 of 363 samples with each configuration. These results describe the simulated warehouse surface and model.

| DiffDrive rolling track | Peak yaw error | Final XY error |
| --- | ---: | ---: |
| 0.410 m | 11.10° | 193.3 mm |
| 0.365 m | <0.01° | <0.1 mm |

The CSV records both poses and the errors for each leg.

## Verify the simulation

```bash
/usr/bin/python3 scripts/smoke.py --warehouse
/usr/bin/python3 scripts/test_trajectory.py
```

The warehouse smoke test checks simulation time, TF at the LiDAR timestamp, IMU readings and the response to drive commands. Each run starts and stops its own Gazebo and ROS processes.

To check the ROS–Gazebo installation with the TurtleBot3 reference model, install `ros-jazzy-turtlebot3-gazebo` and run `/usr/bin/python3 scripts/smoke.py`. [ROBOTIS simulation documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
