# Warehouse recovery demo

[Watch the recorded mission](recovery.mp4). Gazebo shows the simulated robot on the left. RViz shows the map, robot estimate, scan and planned path on the right.

The video plays at 2× speed. The recorder measured 0.266 s from relocation to confirmed loss and 2.684 s from loss to gate reopening. At the receiving, delivery and return stations, the ground-truth position errors were 0.183 m, 0.080 m and 0.157 m. Mission status uses transient-local delivery, so the first message can reach the bag after the mission starts.

To inspect the [recorded MCAP bag](recovery-bag.tar.gz) from the repository root, run the following in a sourced ROS terminal.

```bash
tar -xzf demo/recovery-bag.tar.gz
/usr/bin/python3 demo/analyze_bag.py demo20-bag
```

## Run from a clone

Use the ROS 2 Jazzy and Gazebo Harmonic dependencies in the [project README](../README.md). Run from the repository root. Source `/opt/ros/jazzy/setup.bash` in each terminal.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to aisleops_mission \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
ros2 launch launch/warehouse.launch.py map:=demo/landmarks.yaml world:=demo/landmarks.sdf \
  gz_partition:=aisleops-demo sim_seed:=53 recover_localization:=true ground_truth:=true gui:=true
```

Open RViz in a second terminal.

```bash
rviz2 -d demo/warehouse.rviz --ros-args -r /tf:=/aisleops/tf \
  -r /tf_static:=/aisleops/tf_static -r /clock:=/aisleops/clock -p use_sim_time:=true
```

Start recording in a third terminal. Use a fresh bag directory for each run.

```bash
ros2 bag record -s mcap -o recovery-bag --topics /aisleops/mission_status \
  /aisleops/localization/gate_state /aisleops/localization/scan_agreement \
  /aisleops/amcl_pose /aisleops/evaluation/ground_truth /aisleops/cmd_vel_guarded
```

When Nav2 reports that the managed nodes are active, start the mission in another terminal.

```bash
source install/local_setup.bash
ros2 run aisleops_mission delivery_mission --ros-args -r __ns:=/aisleops \
  -r /clock:=/aisleops/clock -p use_sim_time:=true -p recover_localization:=true
```

About four seconds after `STARTED receive`, relocate the moving simulated robot once.

```bash
GZ_IP=127.0.0.1 GZ_PARTITION=aisleops-demo gz service -s /world/warehouse/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name:"aisleops" position:{x:4.45 y:0.25 z:0.005} orientation:{z:-0.522687 w:0.852525}'
```

After `COMPLETED`, stop recording with Ctrl+C. The analysis reports delays in seconds on the bag receipt clock and checks the three arrivals against Gazebo ground truth.

```bash
ros2 bag info recovery-bag
/usr/bin/python3 demo/analyze_bag.py recovery-bag
```
