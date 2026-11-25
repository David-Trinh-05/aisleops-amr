# AisleOps AMR

A ROS 2 simulation for evaluating an autonomous mobile robot in a warehouse. It includes a custom differential-drive robot, 2D LiDAR and IMU, a three-leg Nav2 delivery mission, and an evaluation tool that measures wheel odometry against Gazebo ground truth.

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
sudo apt install python3-colcon-common-extensions
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select aisleops_mission \
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
