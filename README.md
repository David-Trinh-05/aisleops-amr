# AisleOps AMR

A ROS 2 warehouse robot simulation under development. The project will measure whether an autonomous delivery robot can stop when it loses localization, recover its pose, and resume the interrupted delivery. It will also test cases where similar aisles make the pose ambiguous.

## Day 2 · Warehouse AMR bringup

The custom differential-drive AMR uses a Xacro description with a 0.48 × 0.34 m chassis, a 0.41 m wheel track, 2D LiDAR and IMU. The 14 × 10 m warehouse has two shelf rows and marked receiving and delivery bays. All geometry is local. [Gazebo URDF spawning](https://gazebosim.org/docs/harmonic/spawn_urdf/) · [DiffDrive system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros-py ros-jazzy-rclpy
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 scripts/smoke.py --warehouse
```

The test resolves the LiDAR transform at the scan timestamp, compares forward wheel odometry against the change in LiDAR range to the east wall, checks the IMU frame and gravity reading, and commands a turn with matching signs in odometry and IMU yaw rate.

For an interactive Gazebo session, run `ros2 launch launch/warehouse.launch.py gui:=true`. The bridge publishes topics under `/aisleops` by default. The velocity input is `/aisleops/cmd_vel` with type `geometry_msgs/msg/TwistStamped`. [ROS–Gazebo bridge documentation](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md)

## Day 1 · Simulation smoke test

The smoke test runs a TurtleBot3 Burger in a small Gazebo world with a floor and a wall. The reference robot uses the installed ROBOTIS model, URDF and bridge config. The world has no online model dependencies. [ROBOTIS simulation documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

### Run

Use Ubuntu 24.04 with ROS 2 Jazzy, Gazebo Harmonic and an EGL-capable renderer for the simulated LiDAR. [ROS and Gazebo compatibility](https://gazebosim.org/docs/harmonic/ros_installation/) · [Gazebo headless rendering](https://gazebosim.org/api/sim/8/headless_rendering.html)

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-robot-state-publisher ros-jazzy-tf2-ros-py ros-jazzy-rclpy
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 scripts/smoke.py
```

The script starts and stops Gazebo and the ROS nodes. Each run uses a unique ROS namespace and Gazebo partition, with ROS discovery restricted to the local machine. A failed check exits nonzero and prints the end of the simulator and ROS node logs. A wall-clock deadline also catches a stalled simulation clock.

### What passes

- The ROS node uses simulation time and advances at least 0.4 seconds.
- The LiDAR returns 360 rays and the front ray hits the wall about 1.9 m ahead.
- TF resolves `odom → base_footprint → base_link → base_scan` at the LiDAR scan timestamp.
- A stamped velocity command drives the simulated robot at least 0.10 m. A zero command brings the longitudinal speed below 0.02 m/s at the odometry check. Both readings come from wheel odometry.

Example from ROS 2 Jazzy and Gazebo Harmonic 8.11.0:

```text
PASS  /clock advanced, /scan returned 360 rays (front 1.942 m), TF available at scan timestamp
PASS  /cmd_vel moved the robot 0.102 m in /odom, then stopped
```
