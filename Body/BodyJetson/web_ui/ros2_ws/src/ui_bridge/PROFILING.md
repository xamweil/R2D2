# Profiling ui_bridge

How to profile the Python bridge on the Jetson with [py-spy](https://github.com/benfred/py-spy) and compare the result with the baseline in `web_ui/profile.svg`.

The exact command of a profile is stored in its SVG as the flamegraph title. The baseline used:

```bash
py-spy record --threads --native -d 60 -o profile.svg --pid 84
```

- `--threads` keeps each thread separate in the flamegraph.
- `--native` includes C frames, which is what makes zlib and the `_json` encoder visible.
- `-d 60` records for 60 seconds, at the default 100 samples per second.
- `--pid 84` is the bridge process inside the container. The PID changes on every start.

## Requirements

Both are needed and are set up in the working copy:

- `py-spy` in the `pip3 install` list in `web_ui/Dockerfile`
- `cap_add: SYS_PTRACE` for `web_ui` in `docker-compose.yml`

## Recording a profile

Run these on the Jetson, from `Body/BodyJetson`.

1. Start the rest of the system so that all topics and services are up, as for the baseline.

2. Start the container and the bridge. `docker/auto_launch.sh` only sleeps, so the node is started by hand:

   ```bash
   docker compose up -d --build web_ui
   docker exec -it web_ui bash
   source /home/ros/ros2_ws/install/setup.bash
   ros2 run ui_bridge api_node
   ```

3. Open the UI in a browser and leave it on the page with the camera. The baseline had `/mjpeg` streaming and a WebSocket client connected.

4. In a second shell, record:

   ```bash
   docker exec -it web_ui bash
   py-spy record --threads --native -d 60 -o /home/ros/profile_after.svg --pid "$(pgrep -f lib/ui_bridge/api_node)"
   ```

   The `pgrep` pattern matches the bridge itself, not the `ros2 run` wrapper process. py-spy finishes with a line like `Samples: 4915 Errors: 0`. A high error count means the profile is unreliable and should be repeated.

5. Copy the result out of the container. `/home/ros` is not a mounted path.

   ```bash
   docker cp web_ui:/home/ros/profile_after.svg web_ui/profile_after.svg
   ```

## Keeping runs comparable

- Keep the same flags and the same 60 second duration.
- Use the same load: same topics running, and the same number of browser tabs with the camera open. The number of WebSocket clients during the baseline was not recorded. Each client adds send work, so use one tab and note it.
- An open `/mjpeg` stream matters here. Without a viewer the bridge no longer subscribes to the image topic, so a run with the camera closed flatters the result.

## Comparing profiles

Compare absolute sample counts, not percentages. Percentages shift as soon as the total shrinks, so something can keep the same cost and still show a higher share.

With the same duration and rate, the total is a rough measure of load. A 60 second run has 6000 sampling ticks, so 4915 samples means about 0.8 threads busy on average. It is not the same as process CPU in `top`, and no `top` figure exists for the baseline.

Save this as `compare_profiles.py`:

```python
import html
import re
import sys

GROUPS = {
    "all samples": r"^all$",
    "MainThread (event loop)": r"^thread \(\d+\): MainThread",
    "ROS thread": r"ros_thread_main\)$",
    "executor threads (uvloop_N)": r"^thread \(\d+\): uvloop_",
    "snapshot_state": r"^snapshot_state ",
    "to_jsonable": r"^to_jsonable ",
    "json.dumps": r"^dumps \(json/",
    "deflate": r"^deflate ",
    "broadcast_loop": r"^broadcast_loop ",
    "_state_frame": r"^_state_frame ",
    "rclpy spin_once": r"^spin_once \(rclpy/__init__|^spin_once \(rclpy/executors",
}


def load(path):
    svg = open(path, encoding="utf-8").read()
    pattern = r'<title>(.*?) \([\d,]+ samples?, [\d.]+%\)</title><rect [^>]*?fg:x="(\d+)" fg:w="(\d+)"'
    return [(html.unescape(n), int(x), int(w)) for n, x, w in re.findall(pattern, svg)]


def samples(frames, regex):
    # Merge intervals so recursive frames are not counted twice
    spans = sorted((x, x + w) for n, x, w in frames if re.search(regex, n))
    total, end = 0, 0
    for start, stop in spans:
        total += max(0, stop - max(start, end))
        end = max(end, stop)
    return total


profiles = [load(path) for path in sys.argv[1:]]
print(f"{'':30}" + "".join(f"{p[-28:]:>30}" for p in sys.argv[1:]))
for label, regex in GROUPS.items():
    row = ""
    for frames in profiles:
        count, total = samples(frames, regex), samples(frames, GROUPS["all samples"])
        row += f"{count:>20} {count / total:>9.1%}"
    print(f"{label:30}{row}")
```

Then run it with both profiles:

```bash
python3 compare_profiles.py web_ui/profile.svg web_ui/profile_after.svg
```

py-spy leaves very narrow frames out of the SVG, so a group that took almost no time can show 0.

## Baseline

`web_ui/profile.svg`, taken before the latest-message store. `snapshot_state` ran on the ROS thread, and `permessage-deflate` was still on.

| Group | Samples | Share |
| --- | --- | --- |
| All samples | 4915 | 100.0% |
| MainThread (event loop) | 2732 | 55.6% |
| ROS thread | 1868 | 38.0% |
| Executor threads (`uvloop_N`) | 315 | 6.4% |
| `broadcast_loop` | 2289 | 46.6% |
| `deflate` | 1495 | 30.4% |
| `snapshot_state` | 1466 | 29.8% |
| `to_jsonable` | 1464 | 29.8% |
| `json.dumps` | 748 | 15.2% |
| `rclpy` `spin_once` | 369 | 7.5% |

## What to look for

With the latest-message store and `ws_per_message_deflate=False`:

- `snapshot_state`, `to_jsonable`, `json.dumps` and `deflate` should be gone. These names no longer exist in the state path, so seeing them means old code or old settings are running.
- The `uvloop_N` threads should drop a lot. They ran `_get_reply` for every snapshot, and now only serve `/mjpeg` frame polling and occasional service calls.
- The ROS thread should be mostly `spin_once`.
- `broadcast_loop` should be a small part of the total. Serialization now runs on the event loop inside `_state_frame`, so if that frame is large, it blocks HTTP and WebSocket handling and needs a closer look.

Both changes landed together, so one before/after run cannot separate the effect of disabling deflate from the effect of the store.
