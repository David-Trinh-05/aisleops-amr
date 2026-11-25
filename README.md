# AisleOps AMR

A ROS 2 warehouse robot simulation under development. The project will measure whether an autonomous delivery robot can stop when it loses localization, recover its pose, and resume the interrupted delivery. It will also test cases where similar aisles make the pose ambiguous.

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

The next milestone is the custom differential-drive model and warehouse world, with documented LiDAR and IMU mounting frames.
