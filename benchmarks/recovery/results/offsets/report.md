# Paired warehouse recovery

4 pairs, 8 runs. Input fingerprint `76b9bc2e7715765366a7f2c799fa843c51a19fe446706b7edbe718e2e31ac68a`.

| Scenario | Hold only delivered | Recovery delivered | Hold only completed | Recovery completed |
| --- | ---: | ---: | ---: | ---: |
| receive-offset | 0/2 | 2/2 | 0/2 | 2/2 |
| deliver-offset | 0/2 | 0/2 | 0/2 | 0/2 |

| Recovery behavior | Observed |
| --- | ---: |
| Verified pose and resumed the interrupted leg | 4/4 |
| Verified recovery, same-leg resume and all stops reached | 2/4 |

Confirmed loss on 8/8 relocations.
False or unverified reopenings 0/4.
Unmatched HOLD events 0.
Infrastructure errors across recorded attempts 0.

## Runs with mission failures, deadlines or unmatched holds

- deliver-offset/101/recover · mission_failed Mission halted at deliver · 0 unmatched HOLD
- deliver-offset/137/recover · mission_failed Mission halted at deliver · 0 unmatched HOLD

## Measurement environment

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8.11.0, Nav2 1.3.12 and ros_gz_bridge 1.0.22. The recorder and simulator inputs are tracked in commit `21fa4f9`.

```bash
/usr/bin/python3 benchmarks/recovery/run.py --output /tmp/aisleops-benchmark-offsets --jobs 1 --scenarios receive-offset deliver-offset --seeds 101 137
```

Both delivery-offset runs verified their pose and resumed the delivery leg. Nav2 then reported Failed to make progress and the mission did not reach the delivery station.

[Run data](runs.csv) · [Relocation data](faults.csv) · [Trial records](records.tar.gz)
