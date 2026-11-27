# Warehouse recovery benchmark

Run the five standard scenarios in both hold-only and recovery modes. Each pair shares a world, map, Gazebo seed and relocation plan. The two landmark fixtures come from the demo. The symmetric trials use the stock warehouse and the checked-in occupancy grid from SLAM. AMCL uses its own time-based seed, so individual trials can vary.

## Run from a clone

Install the ROS Jazzy and Gazebo Harmonic packages listed in the demo instructions. Build and source both workspace packages. [Dependencies and build](../../demo/README.md#run-from-a-clone)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to aisleops_mission \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
/usr/bin/python3 benchmarks/recovery/run.py --output /tmp/aisleops-recovery --jobs 2
/usr/bin/python3 benchmarks/recovery/report.py /tmp/aisleops-recovery
```

Use a fresh output directory for each measurement campaign. Restarting the same command reuses finished trials with matching source and fault-plan fingerprints. The runner tries startup or transport failures once more and keeps both attempt logs. Mission failures and timeouts stay in the data. Lower `--jobs` on hosts that have trouble starting several Gazebo and Nav2 sessions together.

The output contains one `result.json` per mode and seed with mission and gate events in simulation time. Each attempt also has ROS and Gazebo logs. `runs.csv` is the run-level summary, and `faults.csv` has one row per actual relocation. For a cloud run, use **Actions → ROS 2 checks → Run workflow** and select **Run all 30 recovery pairs and upload results**.

## Published measurements

The full benchmark and offset reports include CSV files and an archive of the JSON records consumed by the analyzer. To check the published full benchmark, extract the archive into an empty directory and point the analyzer at it.

```bash
record_dir=$(mktemp -d)
tar -xzf benchmarks/recovery/results/warehouse/records.tar.gz -C "$record_dir"
/usr/bin/python3 benchmarks/recovery/report.py "$record_dir"
```

[Full benchmark](results/warehouse/report.md) · [Offset benchmark](results/offsets/report.md)

## Read the results

| Scenario | Expected recovery behavior |
| --- | --- |
| Nominal | Reach all three stations |
| Receiving and delivery relocations | Verify the new pose, resume the interrupted leg and reach all three stations |
| Symmetric warehouse | Hold zero velocity and report `NEEDS_HELP ambiguous_pose` |
| Second relocation on the same leg | Resume once, then hold zero velocity and report `NEEDS_HELP retry_limit` after the second injection |

The analysis counts a station as reached when Gazebo ground truth is within 0.25 m and 0.22 rad at the arrival event. A verified reopening requires an AMCL pose within those limits at release. The report uses the recovery-enabled trial for each expected behavior and keeps trials that ended before their planned injection in the denominator. An unmatched HOLD is a gate hold without an associated planned relocation.

## Additional poses and a LiDAR gap

```bash
/usr/bin/python3 benchmarks/recovery/run.py --output /tmp/aisleops-offsets \
  --scenarios receive-offset deliver-offset --seeds 101 137 --jobs 1
/usr/bin/python3 benchmarks/recovery/report.py /tmp/aisleops-offsets \
  --scenarios receive-offset deliver-offset --seeds 101 137
/usr/bin/python3 tests/lidar_gap.py
```

The offset cases change both the relocation position and heading. The LiDAR test slows actual Gazebo scans while the robot is moving and observes the watchdog and zero velocity output. After scan flow returns, it checks for either a verified reopening on the receiving leg or continued zero velocity after a terminal hold or a Nav2 abort.
