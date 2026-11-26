"""Read the recorded delivery and report timings on the bag receipt clock."""
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""))
types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()}
events, gates, truth, lost, zeros = [], [], [], [], []
while reader.has_next():
    topic, data, t = reader.read_next()
    msg = deserialize_message(data, types[topic])
    if topic.endswith("/mission_status"):
        events.append((t, msg.data))
    elif topic.endswith("/gate_state"):
        if not gates or gates[-1][1] != msg.data:
            gates.append((t, msg.data))
    elif topic.endswith("/ground_truth"):
        truth.append((t, msg.pose))
    elif topic.endswith("/scan_agreement"):
        if any(k.key == "health_state" and k.value == "lost" for s in msg.status for k in s.values):
            lost.append(t)
    elif topic.endswith("/cmd_vel_guarded"):
        if msg.twist.linear.x == 0 and msg.twist.angular.z == 0:
            zeros.append(t)

start = next(t for t, e in events if e == "STARTED receive")
jump = next(b for (a, p), (b, q) in zip(truth, truth[1:]) if b > start and
            math.hypot(p.position.x-q.position.x, p.position.y-q.position.y) > 2)
loss = next(t for t in lost if t >= jump)
zero = next(t for t in zeros if t >= loss)
held = next(t for t, state in gates if t >= jump and state == "HOLD localization_lost")
opened = next(t for t, state in gates if t > held and state == "OPEN")
assert [e for t, e in events if e.startswith("REACHED")] == ["REACHED receive", "REACHED deliver", "REACHED return"]
assert "RESUMED receive" in [e for t, e in events] and events[-1][1] == "COMPLETED"
print("elapsed_seconds,event")
for t, e in events:
    print(f"{(t-start)/1e9:.3f},{e}")
print(f"jump_to_lost_s={(loss-jump)/1e9:.3f} lost_to_zero_s={(zero-loss)/1e9:.3f} lost_to_open_s={(opened-loss)/1e9:.3f}")
for leg, x, y in [("receive",-4.5,-3.5), ("deliver",4.5,3.5), ("return",-4.5,0)]:
    stamp = next(t for t, e in events if e == "REACHED " + leg)
    t, p = min(truth, key=lambda item: abs(item[0]-stamp))
    error = math.hypot(p.position.x-x, p.position.y-y)
    q = p.orientation
    yaw = abs(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    assert abs(t-stamp) < 100_000_000 and error <= 0.25 and yaw <= 0.22
    print(f"{leg} xy_m={error:.3f} yaw_rad={yaw:.3f}")
