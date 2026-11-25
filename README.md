# AisleOps AMR

A ROS 2 simulation for evaluating an autonomous mobile robot in a warehouse. It includes a custom differential-drive robot, 2D LiDAR and IMU, and an evaluation tool that measures wheel odometry against Gazebo ground truth.

## Run the warehouse simulation

The custom differential-drive AMR uses a Xacro description with a 0.48 × 0.34 m chassis, a 0.41 m wheel track, 2D LiDAR and IMU. The 14 × 10 m warehouse has two shelf rows and marked receiving and delivery bays. All geometry is local. [Gazebo URDF spawning](https://gazebosim.org/docs/harmonic/spawn_urdf/) · [DiffDrive system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)

Use Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. The headless LiDAR uses an EGL-capable renderer. [ROS and Gazebo compatibility](https://gazebosim.org/docs/harmonic/ros_installation/) · [Gazebo headless rendering](https://gazebosim.org/api/sim/8/headless_rendering.html)

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros-py ros-jazzy-rclpy
source /opt/ros/jazzy/setup.bash
ros2 launch launch/warehouse.launch.py gui:=true
```

The default ROS namespace is `/aisleops`. Send motion commands on `/aisleops/cmd_vel` as `geometry_msgs/msg/TwistStamped`. The simulation publishes `scan`, `imu` and `odom` in the same namespace. [ROS–Gazebo bridge documentation](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md)

## Measure odometry error

```bash
/usr/bin/python3 scripts/smoke.py --trajectory "/tmp/aisleops-$(date +%s)-$$.csv"
```

The runner drives a timed L route of about 1 m forward, a 90° left turn, then 1 m forward. Gazebo publishes ground truth at 50 Hz on an evaluation topic enabled with `ground_truth:=true`. The runner interpolates ground truth at each odometry timestamp and applies the fixed spawn transform to compare both poses in the warehouse frame. It rejects extrapolation and gaps over 100 ms in ground truth. [Gazebo Harmonic PosePublisher source](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/systems/pose_publisher/PosePublisher.cc)

One headless run paired 363 of 363 samples and measured **0.068 m XY RMSE** and **0.193 m final XY error**. The body turned about 101° in Gazebo while wheel odometry reported about 90°. The CSV records both poses and the errors for each leg.

## Verify the simulation

```bash
/usr/bin/python3 scripts/smoke.py --warehouse
/usr/bin/python3 scripts/test_trajectory.py
```

The warehouse smoke test checks simulation time, TF at the LiDAR timestamp, IMU readings and the response to drive commands. Each run starts and stops its own Gazebo and ROS processes.

To check the ROS–Gazebo installation with the TurtleBot3 reference model, install `ros-jazzy-turtlebot3-gazebo` and run `/usr/bin/python3 scripts/smoke.py`. [ROBOTIS simulation documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
