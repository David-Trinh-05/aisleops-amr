# Paired warehouse recovery

30 pairs, 60 runs. Input fingerprint `76b9bc2e7715765366a7f2c799fa843c51a19fe446706b7edbe718e2e31ac68a`.

| Scenario | Hold only delivered | Recovery delivered | Hold only completed | Recovery completed |
| --- | ---: | ---: | ---: | ---: |
| nominal | 6/6 | 5/6 | 6/6 | 5/6 |
| receive | 0/6 | 6/6 | 0/6 | 6/6 |
| deliver | 0/6 | 5/6 | 0/6 | 5/6 |
| symmetric | 0/6 | 0/6 | 0/6 | 0/6 |
| repeat | 0/6 | 0/6 | 0/6 | 0/6 |

| Recovery behavior | Observed |
| --- | ---: |
| Verified pose and resumed the interrupted leg | 11/12 |
| Verified recovery, same-leg resume and all stops reached | 11/12 |
| Held after ambiguous localization | 6/6 |
| Held after a second relocation on the retried leg | 6/6 |

Confirmed loss on 53/53 relocations.
False or unverified reopenings 0/17.
Unmatched HOLD events 2.
Infrastructure errors across recorded attempts 0.

## Runs with mission failures, deadlines or unmatched holds

- nominal/89/recover · needs_help retry_limit · 1 unmatched HOLD
- deliver/41/recover · needs_help retry_limit · 1 unmatched HOLD

## Measurement environment

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8.11.0, Nav2 1.3.12 and ros_gz_bridge 1.0.22. The recorder and simulator inputs are tracked in commit `21fa4f9`.

```bash
/usr/bin/python3 benchmarks/recovery/run.py --output /tmp/aisleops-benchmark-v2 --jobs 4
```

The two unmatched holds occurred before the first navigation goal. The gate reported localization_stale in nominal/89/recover and localization_unavailable in deliver/41/recover. The second of these runs ended before its planned relocation.

[Run data](runs.csv) · [Relocation data](faults.csv) · [Trial records](records.tar.gz)
