# POC Fusion — Environment & On-Robot Verification Log

Running record for the LiDAR + Aurora depth camera → Nav2 local costmap fusion POC
(`.superpowers/sdd/2026-08-05-lidar-depth-costmap-fusion/`). Each task appends its own
dated section below. Container commands use the standard preamble unless noted:

```bash
docker exec -u ubuntu MentorPi bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ubuntu/ros2_ws/install/setup.bash
  <command>
'
```

---

## Task 0: Environment preflight (2026-08-07)

### Step 1 — Baseline facts (as confirmed live 2026-08-06, per CONSTRAINTS.md)

These were captured the day before this session by prior work on this plan; Task 0
re-confirms the ones with an explicit re-verification command in Steps 2–5 below and
carries the rest forward unchanged.

| | |
|---|---|
| LiDAR | LD19 driver, node `/LD19`; platform provisioned `LIDAR_TYPE=MS200` (vendor launch routes the MS200 branch to the LD19 driver) |
| Scan topic | `/scan_raw` — `/scan` does not exist (the `laser_filters` chain is commented out of the launch description, so the scan is unfiltered) |
| Scan rate | 9.87 Hz (2026-08-06 measurement) |
| Depth image | `/ascamera/camera_publisher/depth0/image_raw`, 640×400, 14.7 Hz |
| Depth encoding | `mono16` — NOT `16UC1`. Must be relabelled (Task 4 Step 1a) |
| Camera pose (operating) | ~40° below horizontal, 0.246 m above floor; coverage ceiling ~0.94 m |
| Depth camera frame | `depth_camera_link` (the URDF also defines `depth_cam_link` — a different frame, not the one topics use) |
| `camera_info` | `/ascamera/camera_publisher/depth0/camera_info` — valid, `k = [423.92, 0, 319.30; 0, 424.83, 190.86]` |
| Velocity topic (ours) | `/cmd_vel` |
| Velocity topic (vendor) | `/controller/cmd_vel` — 5 vendor publishers, off-limits |

### Step 2 — Scan topic + rate re-confirmation (2026-08-07, ~02:56 HKT / 2026-08-06 ~18:56 UTC)

Command:
```
ros2 topic list | grep -i scan
ros2 topic hz /scan_raw
```

Actual output:
```
$ ros2 topic list | grep -i scan
/scan_raw

$ ros2 topic hz /scan_raw
WARNING: topic [/scan_raw] does not appear to be published yet
average rate: 9.978
	min: 0.001s max: 0.341s std dev: 0.07548s window: 14
average rate: 10.001
	min: 0.001s max: 0.341s std dev: 0.05650s window: 25
```
(Sampling was cut short by a client-side timeout wrapper after ~8s; the two printed
windows are real live samples, not fabricated.)

**Result: still `/scan_raw`, only topic matching "scan".** Rate measured at ~9.98–10.0 Hz
this session vs. 9.87 Hz on 2026-08-06 — consistent with the LD19's normal jitter, not a
driver change. No discrepancy worth flagging.

Written to `poc_fusion/config/costmap_params.yaml` as `scan_topic: /scan_raw` — the
single source of truth for this value going forward; no other file may hard-code it.

### Step 2a — LiDAR model discrepancy (2026-08-07)

`lidar.launch.py` excerpt (`/home/ubuntu/ros2_ws/src/peripherals/launch/lidar.launch.py`,
confirmed at this exact path — no relocation):

```python
lidar_type = os.environ['LIDAR_TYPE']
...
if lidar_type == 'MS200':
    # lidar_launch_path = os.path.join(peripherals_package_path, 'launch/include/ms200_scan.launch.py')
    lidar_launch_path = os.path.join(peripherals_package_path, 'launch/include/ldlidar_LD19.launch.py')
elif lidar_type == 'LD19':
    lidar_launch_path = os.path.join(peripherals_package_path, 'launch/include/ldlidar_LD19.launch.py')
else:
    lidar_launch_path = os.path.join(peripherals_package_path, 'launch/include/ms200_scan.launch.py')
...
    return LaunchDescription([
        scan_topic_arg,
        scan_raw_arg,
        lidar_frame_arg,

        lidar_launch,
        # laser_filter_node
    ])
```

So: the platform is provisioned as `LIDAR_TYPE=MS200`, but the `MS200` branch routes to
the **LD19** launch include (`ldlidar_LD19.launch.py`), not an MS200-specific one — the
robot is physically running an LD19 driver despite the `MS200` env label. The
`laser_filter_node` (the `laser_filters` chain that would remap `scan_raw` → `scan`) is
commented out of the returned `LaunchDescription`, which is why `/scan` never appears and
`/scan_raw` is the unfiltered, only scan topic on the graph.

`LIDAR_TYPE` process-env evidence (read from the live bringup process, not a config file
on disk):
```
$ ps aux | grep -i "lidar\|ldlidar\|bringup" | grep -v grep
ubuntu       357  ...  /usr/bin/python3 /opt/ros/humble/bin/ros2 launch bringup bringup.launch.py
ubuntu      1607 ...  /home/ubuntu/third_party_ros2/third_party_ws/install/ldlidar_stl_ros2/lib/ldlidar_stl_ros2/ldlidar_stl_ros2_node --ros-args -r __node:=LD19 --params-file /tmp/launch_params_bigiwv5x -r scan:=scan_raw
ubuntu      1613 ...  /home/ubuntu/ros2_ws/install/app/lib/app/lidar_controller --ros-args

$ tr '\0' '\n' < /proc/357/environ | grep -i LIDAR_TYPE
LIDAR_TYPE=MS200

$ tr '\0' '\n' < /proc/1607/environ | grep -i LIDAR_TYPE
LIDAR_TYPE=MS200
```

Node identity re-confirmed:
```
$ ros2 node list | grep -i ld19
/LD19

$ ros2 topic info /scan_raw -v
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Node name: LD19
...
Subscription count: 0
```

**Result: `LIDAR_TYPE=MS200` and node `/LD19` both still hold — no driver change since
2026-08-06.** The `laser_filters` chain is confirmed commented out in the launch source
itself (line `# laser_filter_node`), matching the "scan is unfiltered" fact. Feeds Task 16
Steps 3a/3b unchanged.

### Step 3 — Install `depth_image_proc` and `image_proc`

Pre-install check (as `ubuntu`, confirming the gap the brief describes):
```
$ ros2 pkg prefix depth_image_proc
Package not found
```

Install command (the one legitimate `-u root` use in the whole plan — touches the system
image, not the workspace; nothing is built as root):
```
docker exec -u root MentorPi bash -lc 'apt-get update && apt-get install -y ros-humble-depth-image-proc ros-humble-image-proc'
```

**First attempt failed** — the container could not resolve any of its configured apt
mirrors (Tsinghua mirrors, `ports.ubuntu.com`):
```
Err:2 https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports noble InRelease
  Temporary failure resolving 'mirrors.tuna.tsinghua.edu.cn'
Err:1 http://ports.ubuntu.com/ubuntu-ports noble-security InRelease
  Temporary failure resolving 'ports.ubuntu.com'
...
E: Unable to fetch some archives, maybe run apt-get update or try with --fix-missing?
```

Root cause, diagnosed directly (per "verify infra claims" practice — checked the actual
container state rather than assuming a flaky network):
```
$ docker exec -u root MentorPi bash -lc 'cat -A /etc/resolv.conf'
# Generated by NetworkManager$

$ cat /etc/resolv.conf        # host, for comparison
# Generated by NetworkManager
search mshome.net
nameserver 192.168.137.1

$ docker inspect MentorPi --format '{{.HostConfig.NetworkMode}}'
host
```
The container runs with `--network host` but its own `/etc/resolv.conf` was missing the
`nameserver` line the host has (host has connectivity — confirmed `ping 8.8.8.8` from the
host succeeded, 0% loss). This is a container DNS-config gap, not a genuine absence of
network access or unavailable packages, so it does not meet the brief's BLOCKED bar
("apt-get cannot reach the network or the packages are unavailable"). Fix applied is a
one-line addition of the host's own nameserver to the container's resolv.conf — a system
network-config fix, not a source build or workspace change:
```
$ docker exec -u root MentorPi bash -lc "echo 'nameserver 192.168.137.1' >> /etc/resolv.conf"
```

Retried `apt-get update` — succeeded (mirrors resolved, index fetched). Retried install —
succeeded:
```
$ docker exec -u root MentorPi bash -lc 'apt-get install -y ros-humble-depth-image-proc ros-humble-image-proc'
...
The following NEW packages will be installed:
  ros-humble-depth-image-proc ros-humble-image-proc
  ros-humble-tracetools-image-pipeline
...
Setting up ros-humble-depth-image-proc (3.0.9-1jammy.20260607.104333) ...
Setting up ros-humble-tracetools-image-pipeline (3.0.9-1jammy.20260307.161552) ...
Setting up ros-humble-image-proc (3.0.9-1jammy.20260607.110055) ...
```

Post-install verification, as `ubuntu` (per the brief's exact command):
```
$ docker exec -u ubuntu MentorPi bash -lc 'source /opt/ros/humble/setup.bash && ros2 pkg prefix depth_image_proc'
/opt/ros/humble

$ docker exec -u ubuntu MentorPi bash -lc 'source /opt/ros/humble/setup.bash && ros2 pkg prefix image_proc'
/opt/ros/humble
```

**Result: both packages installed and resolvable as `ubuntu`.** Task 5's dependency gap is
closed. Note for future sessions: the container's `/etc/resolv.conf` fix is a live,
in-container edit — it is not persisted through the vendor's image/container lifecycle
management, so a fresh container recreate may need the same nameserver line re-added
before any future `apt-get` in this plan.

### Step 4 — Regression check: rest of the dependency set

```
$ for p in nav2_costmap_2d nav2_lifecycle_manager rclcpp_components cv_bridge tf2_tools; do
    echo -n "$p: "; ros2 pkg prefix $p
  done
nav2_costmap_2d: /opt/ros/humble
nav2_lifecycle_manager: /opt/ros/humble
rclcpp_components: /opt/ros/humble
cv_bridge: /opt/ros/humble
tf2_tools: /opt/ros/humble

$ ros2 pkg executables nav2_costmap_2d
nav2_costmap_2d nav2_costmap_2d
nav2_costmap_2d nav2_costmap_2d_cloud
nav2_costmap_2d nav2_costmap_2d_markers

$ python3 -c "import scipy; print(scipy.__version__)"
1.8.0

$ python3 -c "import cv2; print(cv2.__version__)"
4.10.0
```

**Result: all present, all versions unchanged from the prior verification** — `scipy
1.8.0`, `OpenCV 4.10.0`, and `nav2_costmap_2d`'s plain `nav2_costmap_2d` executable name
all confirmed exactly as stated in CONSTRAINTS.md. No regression.

### Step 5 — Velocity-topic decision (re-confirmed 2026-08-07)

```
$ ros2 topic info /cmd_vel -v
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1
Node name: odom_publisher
...

$ ros2 topic info /controller/cmd_vel -v
Type: geometry_msgs/msg/Twist
Publisher count: 5
  Node name: lidar_app
  Node name: line_following
  Node name: object_tracking
  Node name: hand_gesture
  Node name: joystick_control
Subscription count: 1
Node name: odom_publisher
```

**Decision (unchanged): `/cmd_vel` is this POC's output topic** — 0 current publishers, 1
subscriber (`odom_publisher`); this project's own `path_tracker` is the intended future
publisher there. **`/controller/cmd_vel` is off-limits** — 5 vendor publishers already on
it (`lidar_app`, `line_following`, `object_tracking`, `hand_gesture`, `joystick_control`).
Publishing into the wrong topic would produce a robot that silently ignores stop commands.
This decision is resolved now, not during debugging, per CONSTRAINTS.md.

Per HARD SAFETY RULE 2, no velocity command was published during this verification — both
checks above are `ros2 topic info`, which is read-only.

### Additional spot-check (informational, not a required brief step)

Re-confirmed the depth camera topic set is still present and unchanged:
```
$ ros2 topic list | grep -i ascamera
/ascamera/camera_publisher/depth0/camera_info
/ascamera/camera_publisher/depth0/image_raw
/ascamera/camera_publisher/depth0/points
/ascamera/camera_publisher/ir0/image
/ascamera/camera_publisher/rgb0/camera_info
/ascamera/camera_publisher/rgb0/image
```

### Task 0 summary

All five steps complete. One real finding: the container's DNS was misconfigured (missing
`nameserver` line, unrelated to package availability), fixed with a single additive line
so `apt-get` could reach the ROS/Ubuntu mirrors. No HARD SAFETY RULE was touched — no
`cmd_vel` publish, no arm motion, no `.stop_ros.sh`, no edits to `proximity_alert/` or the
vendor `ros2_ws`. `poc_fusion/config/costmap_params.yaml` now holds the single
source-of-truth `scan_topic: /scan_raw`.
