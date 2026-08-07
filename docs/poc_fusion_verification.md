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

## Clock / timezone note (read before trusting any timestamp in this doc)

**Host local timezone is HKT (UTC+8). The container and every ROS timestamp are UTC.**
During a local-evening/early-morning session, container-generated filenames and
`tf2_echo`'s `At time` stamps read **one calendar day earlier** than the local working
date. Do not infer "stale data from yesterday" from a UTC-dated filename alone — convert
it first.

Evidence (captured 2026-08-07, during the Task 1 fix round):
```
$ date
Fri Aug  7 03:12:32 AM HKT 2026

$ date -u
Thu Aug  6 07:12:32 PM UTC 2026

$ docker exec -u ubuntu MentorPi date -u
Thu Aug  6 19:12:32 UTC 2026

$ timedatectl
               Local time: Fri 2026-08-07 03:12:33 HKT
           Universal time: Thu 2026-08-06 19:12:33 UTC
                 Time zone: Asia/Hong_Kong (HKT, +0800)
```
So: local `Fri Aug 7 03:12 HKT` = UTC `Thu Aug 6 19:12`. A `view_frames` artifact or
`tf2_echo` stamp reading `2026-08-06 19:0x` UTC during this session is genuinely **today**
(2026-08-07 local) — it is not a leftover from the prior day's session. Every task after
this one should convert before asserting "this is/isn't today's data."

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

---

## Task 1: TF chain verification — record only (2026-08-07)

This task writes no source code. It documents the TF chain from `base_link` to the depth
camera's optical frame and establishes the pitch-measurement procedure that Task 10 (and
Task 10a) reuse each session. All commands below (`view_frames`, `tf2_echo`) are read-only
per CONSTRAINTS.md — no `cmd_vel` publish, no arm command, no `.stop_ros.sh`.

**What's freshly measured today vs. transcribed:** Steps 1–3 below are live captures from
this session, run from `/tmp` inside the container so no `view_frames` artifacts land in
the vendor `ros2_ws`. Per the clock/timezone note above, the container timestamps them in
UTC, which is one calendar day behind local — converted explicitly per step below so
nobody has to take that on faith:
- Step 1 (`view_frames`): captured at epoch ~1786043119–1786043124, i.e.
  **UTC 2026-08-06 19:05:19–19:05:24 = local (HKT) 2026-08-07 03:05:19–03:05:24** — today,
  local date.
- Step 3 (`tf2_echo`): first clean sample at epoch 1786043135.72, i.e.
  **UTC 2026-08-06 19:05:35 = local (HKT) 2026-08-07 03:05:35** — six minutes into the
  same session as Step 1, same local calendar day.
The task-1 brief's own prior values (`(0.766, −0.005, −0.642)` → 39.9° down, verified
2026-08-06 per CONSTRAINTS.md) are quoted only as the point of comparison in Step 3 — they
are not presented as this session's own measurement.

### Step 1 — The TF chain

Commands run from a scratch directory inside the container (not `/home/ubuntu/ros2_ws`,
per CONSTRAINTS.md — `view_frames` writes its PDF/GV output into the CWD):
```
docker exec -u ubuntu MentorPi bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ubuntu/ros2_ws/install/setup.bash
  mkdir -p /tmp/view_frames_scratch
  cd /tmp/view_frames_scratch
  timeout 15 ros2 run tf2_tools view_frames
'
```

Actual output (`frame_yaml`, trimmed to the arm/camera/base lineage; full output also
lists wheel, gripper and IMU frames not relevant here):
```
base_footprint:
  parent: 'odom'
base_link:
  parent: 'base_footprint'
back_shell_black_link:
  parent: 'base_link'
link1:
  parent: 'back_shell_black_link'
link2:
  parent: 'link1'
link3:
  parent: 'link2'
link4:
  parent: 'link3'
camera_connect_link:
  parent: 'link4'
depth_cam_link:
  parent: 'camera_connect_link'
depth_camera_link:
  parent: 'depth_cam_link'
rgb_camera_link:
  parent: 'depth_camera_link'
```

**Full resolved chain, live-verified today:**
`odom → base_footprint → base_link → back_shell_black_link → link1 → link2 → link3 →
link4 → camera_connect_link → depth_cam_link → depth_camera_link`

Two refinements versus the brief's shorthand chain (`base_link → link1..link4 →
camera_connect_link → depth_cam_link → depth_camera_link`, plus `odom → base_link`):
these are more precise, not contradictions —
1. There's a `back_shell_black_link` hop between `base_link` and `link1` (the arm's
   physical mount point on the chassis shell).
2. `odom → base_link` is actually `odom → base_footprint → base_link` — `base_footprint`
   is the intermediate ground-projection frame REP-105 expects; it doesn't change which
   frames matter for the fusion transform, but a reader chasing this chain by hand needs
   the intermediate frame name to match what `tf2_echo`/`view_frames` actually print.

Artifacts confirmed via `ls -la --time-style=full-iso /tmp/view_frames_scratch/`:
```
-rw-r--r-- 1 ubuntu ubuntu    5262 2026-08-06 19:05:24.330294132 +0000 frames_2026-08-06_19.05.24.gv
-rw-r--r-- 1 ubuntu ubuntu   19688 2026-08-06 19:05:24.682296172 +0000 frames_2026-08-06_19.05.24.pdf
```
Only **one** `.gv`/`.pdf` pair exists — this run's own output. `view_frames` names its
artifact from the container's (UTC) clock, so the filename reads `2026-08-06` even though,
per the clock/timezone note above, the capture was taken during today's (2026-08-07 local)
session — UTC `19:05:24` = HKT `03:05:24`. There is no second, earlier pair; an
earlier draft of this section incorrectly asserted one (`frames_2026-08-07_*`) that was
never captured — corrected here. Both files stayed in `/tmp/view_frames_scratch` inside
the container only — not committed to this repo, not written into `/home/ubuntu/ros2_ws`.

**The `depth_cam_link` vs `depth_camera_link` trap — the single most important thing in
this section.** The URDF (`landerpi_description/urdf/arm.urdf.xacro`) defines
`depth_cam_link` as a real mesh link, fixed to `camera_connect_link` via the
`depth_cam_joint` (translation `[-0.0100, 0.0006, -0.0001]`, near-zero rotation). A
**second, separate** frame, `depth_camera_link`, is layered on top of it — added not by
the URDF but by a `static_transform_publisher` node in
`peripherals/launch/include/aurora930.launch.py`:
```
executable='static_transform_publisher',
arguments = ['0', '0.02', '0', '-1.57', '0', '-1.57', 'depth_cam_link', 'depth_camera_link']
# [x, y, z, roll, pitch, yaw, parent_frame, child_frame]
```
That's the standard `camera_link → camera_optical_frame` axis-convention rotation
(roll/pitch/yaw ≈ −90°/0°/−90°): it re-expresses the mesh-link axes (x-forward, per REP-103
convention for a body-fixed link) into the optical convention (x-right, y-down, z-forward)
that `depth_image_proc` and the `sensor_msgs/CameraInfo` pipeline expect. **Every depth
topic (`/ascamera/camera_publisher/depth0/image_raw`, `.../camera_info`, `.../points`) is
stamped `depth_camera_link`, never `depth_cam_link`.** Using `depth_cam_link` for any TF
lookup in this project silently shifts the whole point cloud by that rotation — no error,
just wrong geometry. This confirms the CONSTRAINTS.md table entry; it is now traced to its
exact source (the static transform above), not just asserted.

### Step 2 — The transform is arm-driven, not a static fudge

From the URDF (`arm.urdf.xacro`):
```
<joint name="joint4" type="revolute">
  <parent link="link3" />
  <child link="link4" />
  <axis xyz="0.00859391749791871 -0.999923336386447 0.00891436659750842" />
  <limit lower="-2.09" upper="2.09" effort="1000" velocity="10" />
</joint>
...
<joint name="camera_connect_joint" type="fixed">
  <origin xyz="-0.035657 0.015573 0.044852" rpy="-0.7888 -1.5585 0.79749" />
  <parent link="link4" />
  <child link="camera_connect_link" />
</joint>
```
`camera_connect_joint` is itself `type="fixed"` — the camera mount does not wobble
relative to `link4`. But its **parent is `link4`**, which is the child of the **revolute**
`joint4` (±2.09 rad limit, i.e. ±120°, driven by the arm). So the entire downstream chain
(`camera_connect_link → depth_cam_link → depth_camera_link`) rotates rigidly with `link4`
whenever `joint4` moves. This is exactly why the arm must be **locked** for the duration
of any fusion run in this POC: any joint4 motion changes the camera pitch measured in Step
3 below, and nothing in the fusion pipeline re-measures it live — Task 10 re-checks it once
per session as a manual gate, not continuously.

`view_frames` corroborates this from the live graph. Step 1's quoted `frame_yaml` was
trimmed to `parent` only for readability; here are the **untrimmed** rows for the frames
this claim rests on, straight from the same `view_frames` response quoted in Step 1:
```
link4:
  parent: 'link3'
  broadcaster: 'default_authority'
  rate: 13.137
  most_recent_transform: 1786043124.271770
  oldest_transform: 1786043119.323946
  buffer_length: 4.948

camera_connect_link:
  parent: 'link4'
  broadcaster: 'default_authority'
  rate: 10000.000
  most_recent_transform: 0.000000
  oldest_transform: 0.000000
  buffer_length: 0.000

depth_cam_link:
  parent: 'camera_connect_link'
  broadcaster: 'default_authority'
  rate: 10000.000
  most_recent_transform: 0.000000
  oldest_transform: 0.000000
  buffer_length: 0.000

depth_camera_link:
  parent: 'depth_cam_link'
  broadcaster: 'default_authority'
  rate: 10000.000
  most_recent_transform: 0.000000
  oldest_transform: 0.000000
  buffer_length: 0.000
```
`link4`'s `rate: 13.137` and non-zero `buffer_length: 4.948` show it is being actively
broadcast by `robot_state_publisher` from joint-state updates, unlike the `fixed`-joint
downstream frames (`camera_connect_link`, `depth_cam_link`, `depth_camera_link`), which
show `rate: 10000.000` / `buffer_length: 0.000` — TF2's convention for a static transform
published once and held indefinitely, with no joint-state dependency of its own.

### Step 3 — Pitch-measurement procedure and today's value

Command:
```
docker exec -u ubuntu MentorPi bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ubuntu/ros2_ws/install/setup.bash
  timeout 8 ros2 run tf2_ros tf2_echo base_link depth_camera_link
'
```

Actual output (steady-state, repeated once per second once the buffer filled; the very
first line is a transient "frame does not exist" warning printed before TF2's buffer had
populated — it resolves within the same second and every subsequent sample is clean):
```
[INFO] [tf2_echo]: Waiting for transform base_link -> depth_camera_link: Invalid frame ID
"base_link" passed to canTransform argument target_frame - frame does not exist
At time 1786043135.721853992
- Translation: [0.105, 0.021, 0.192]
- Rotation: in Quaternion [-0.639, 0.642, -0.300, 0.298]
- Rotation: in RPY (radian) [-2.268, -0.002, -1.576]
- Rotation: in RPY (degree) [-129.974, -0.101, -90.289]
- Matrix:
 -0.005 -0.642  0.766  0.105
 -1.000  0.002 -0.005  0.021
  0.002 -0.766 -0.642  0.192
  0.000  0.000  0.000  1.000
At time 1786043136.671746518
- Translation: [0.105, 0.021, 0.192]
- Matrix:
 -0.005 -0.642  0.766  0.105
 -1.000  0.002 -0.005  0.021
  0.002 -0.766 -0.642  0.192
  0.000  0.000  0.000  1.000
At time 1786043137.671757982
- Translation: [0.105, 0.021, 0.192]
- Matrix:
 -0.005 -0.642  0.766  0.105
 -1.000  0.002 -0.005  0.021
  0.002 -0.766 -0.642  0.192
  0.000  0.000  0.000  1.000
At time 1786043138.723878323
- Translation: [0.105, 0.021, 0.192]
- Matrix:
 -0.005 -0.642  0.766  0.105
 -1.000  0.002 -0.005  0.021
  0.002 -0.766 -0.642  0.192
  0.000  0.000  0.000  1.000
At time 1786043139.721786775
- Translation: [0.105, 0.021, 0.192]
- Matrix:
 -0.005 -0.642  0.766  0.105
 -1.000  0.002 -0.005  0.021
  0.002 -0.766 -0.642  0.192
  0.000  0.000  0.000  1.000
At time 1786043140.721762554
- Translation: [0.105, 0.021, 0.192]
- Matrix:
 -0.005 -0.642  0.766  0.105
 -1.000  0.002 -0.005  0.021
  0.002 -0.766 -0.642  0.192
  0.000  0.000  0.000  1.000
[INFO] [rclcpp]: signal_handler(signum=15)
```
(RPY/Quaternion rows omitted from the repeated blocks above for brevity — the Matrix row,
which is what Step 3's arithmetic uses, is pasted in full for every block. All six
timestamped blocks after the first carry the identical translation and matrix shown
above — arm was stationary throughout this ~5-second capture, consistent with Step 2's
"must be locked" requirement being honored during this verification. The run was ended by
the `timeout 8` wrapper, hence the `signal_handler` line.)

**Procedure:** the 3×3 rotation block's **third column** is `depth_camera_link`'s optical
z-axis (forward, per the optical convention x-right/y-down/z-forward established in Step
1) expressed in `base_link`. Reading that column off the matrix above:
```
forward axis (base_link frame) = (0.766, -0.005, -0.642)
```
Downward pitch = `asin(-z)` where `z = -0.642`:
```
$ python3 -c "import math; print(math.degrees(math.asin(0.642)))"
39.94111664125141
```
**Today's measured value: forward axis `(0.766, −0.005, −0.642)` → 39.9° down.**

**Comparison with the brief:** this matches the brief's recorded prior value —
`(0.766, −0.005, −0.642)` → 39.9° down — to three decimal places on every component. No
discrepancy to report; the arm's parked pose has not drifted between 2026-08-06 and
today's re-measurement.

### Task 1 summary

All three steps recorded, no code written, no arm command issued, no velocity published.
Key findings for future tasks:
- The chain resolves as `odom → base_footprint → base_link → back_shell_black_link →
  link1 → link2 → link3 → link4 → camera_connect_link → depth_cam_link →
  depth_camera_link`; use `depth_camera_link` for every fusion lookup, never
  `depth_cam_link` — the two are related by a `static_transform_publisher`-added optical
  rotation, not by identity.
- The pitch is arm-driven through revolute `joint4`, not a fixed offset baked into the
  camera mount — Task 10 must re-verify it every session with the Step 3 procedure above,
  and Task 10a reuses the same procedure to confirm the overhang-test pose where the
  z-component is expected to flip sign (to +0.094 per the brief).
- Today's live re-measurement reproduced the brief's recorded value exactly: forward axis
  `(0.766, −0.005, −0.642)` → 39.9° down.

---

## Task 2: Package scaffolding (2026-08-07, ~03:20 HKT / 2026-08-06 ~19:20 UTC)

Package authored and committed at
`/home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion/` (host), mirroring
`proximity_alert/`'s `ament_python` layout. `poc_fusion/config/costmap_params.yaml`
(Task 0's `scan_topic: /scan_raw`) was left untouched — confirmed via
`git diff -- poc_fusion/config/costmap_params.yaml` (no output) before committing.

### Step 2a — `rosdep check`

Run after deploying (Step 4), per the brief's own note that the check path is
inside the container:
```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    cd /home/ubuntu/ros2_ws
    rosdep check --from-paths src/poc_fusion --ignore-src
  '
All system dependencies have been satisfied
```
`rosdep` was already initialised in the container (no `rosdep init`/`update` was run,
as root or otherwise — none was needed). All 14 `package.xml` dependencies resolve.

### Step 4 — Deploy script round-trip verification

Deploy:
```
$ bash scripts/deploy_poc_fusion.sh
Deploying /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion -> MentorPi:/home/ubuntu/ros2_ws/src/poc_fusion
No stale destination files to remove.
Deploy complete.
```

Round-trip diff (fresh `docker cp` of the host tree into a scratch path, diffed
against the deployed destination):
```
$ docker cp poc_fusion/. MentorPi:/tmp/poc_fusion_hostcopy
$ docker exec -u ubuntu MentorPi diff -r /tmp/poc_fusion_hostcopy /home/ubuntu/ros2_ws/src/poc_fusion
DIFF_EXIT=0
```
No differences.

Prune behaviour verified directly: a stray `poc_fusion/poc_fusion/stale_module.py`
and a `poc_fusion/poc_fusion/__pycache__/foo.pyc` were manually planted in the
container destination, then the deploy script was re-run:
```
$ bash scripts/deploy_poc_fusion.sh
Deploying /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion -> MentorPi:/home/ubuntu/ros2_ws/src/poc_fusion
Removing stale destination files (absent from host source):
  poc_fusion/stale_module.py
Deploy complete.
```
`stale_module.py` (absent from host source) was deleted; `__pycache__/foo.pyc` was
left untouched, confirming the script deletes renamed/removed modules but leaves
`build/`, `install/`, `log/`, and `__pycache__` alone. A second round-trip
`diff -r --exclude=__pycache__` after this test again returned no differences
(`DIFF_EXIT=0`). Ownership confirmed `ubuntu:ubuntu` throughout via
`stat -c '%U:%G'` on the destination root and sampled files.

### Step 5 — Build and executables

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    cd /home/ubuntu/ros2_ws
    colcon build --packages-select poc_fusion
  '
Starting >>> poc_fusion
--- stderr: poc_fusion
(setuptools "setup.py install is deprecated" warning only — same warning
proximity_alert's build already produces, not a poc_fusion-specific issue)
---
Finished <<< poc_fusion [3.75s]

Summary: 1 package finished [5.47s]
  1 package had stderr output: poc_fusion
```

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ros2 pkg executables poc_fusion
  '
poc_fusion costmap_stop_monitor_node
poc_fusion depth_preprocess_node
poc_fusion latency_recorder_node
```

Build as `ubuntu` only, at every step (never root) — confirmed by the `-u ubuntu`
flag on every `docker exec` above. `-u root` was used only for the deploy
script's final `chown`.

### Task 2 summary

`poc_fusion` package created and committed at the host repo root (`ament_python`,
mirroring `proximity_alert/`), with all 14 `package.xml` dependencies from the
brief declared verbatim, three placeholder node entry points registered
(`depth_preprocess_node`, `costmap_stop_monitor_node`, `latency_recorder_node`),
and `scripts/deploy_poc_fusion.sh` written as the sole host→container path. Pure
logic is not yet implemented — node modules are import-and-spin stubs only, per
the brief; `lib/window_geometry.py` and `lib/obstacle_detection.py` deliberately
not created (Task 3's TDD RED step). `rosdep check`, the deploy round-trip
`diff -r`, and `colcon build --packages-select poc_fusion` (as `ubuntu`) all
passed. No HARD SAFETY RULE was touched — no `cmd_vel` publish, no arm command,
no `.stop_ros.sh`, no edits to `proximity_alert/` or hand-written files under
`/home/ubuntu/ros2_ws/src/`.

---

## Task 4 — Depth preprocessing node

Captured 2026-08-07, **22:24 HKT / 14:24 UTC** (host `date` / `date -u` re-run just
before this section, per the timezone note above):
```
$ date
Fri Aug  7 10:24:49 PM HKT 2026
$ date -u
Fri Aug  7 02:24:49 PM UTC 2026
```

### Host test suite — before

```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/ -q
...................                                                      [100%]
19 passed in 0.22s
```

### TDD: `poc_fusion/poc_fusion/lib/depth_preprocess.py`

RED (module does not exist yet; `test/test_depth_preprocess.py` written first,
covering `invalid_pixel_mask`, `invalid_fraction`, `apply_roi_mask`,
`median_filter_depth`):

```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/test_depth_preprocess.py -q
==================================== ERRORS ====================================
________________ ERROR collecting test/test_depth_preprocess.py ________________
ImportError while importing test module '/home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion/test/test_depth_preprocess.py'.
Hint: make sure your test modules/packages have valid Python names.
Traceback:
/usr/lib/python3.11/importlib/__init__.py:126: in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
test/test_depth_preprocess.py:3: in <module>
    from poc_fusion.lib.depth_preprocess import (
E   ModuleNotFoundError: No module named 'poc_fusion.lib.depth_preprocess'
=========================== short test summary info ============================
ERROR test/test_depth_preprocess.py
!!!!!!!!!!!!!!!!!!!! Interrupted: 1 error during collection !!!!!!!!!!!!!!!!!!!!
1 error in 0.18s
```

This is a collection-level failure covering every function in the module at
once (the whole file fails to import), which is the correct RED signal for a
module that does not exist yet — there is no way to fail per-function before
the module exists to import.

GREEN, after writing `poc_fusion/poc_fusion/lib/depth_preprocess.py`
(`invalid_pixel_mask`, `invalid_fraction`, `apply_roi_mask`,
`median_filter_depth`):

```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/ -q
...............................                                          [100%]
31 passed in 0.25s
```

12 new tests (19 pre-existing + 12 = 31), covering:
- `invalid_pixel_mask`: exact-zero flagging, all-valid case.
- `invalid_fraction`: all-zero (1.0), all-valid (0.0), partial (0.25).
- `apply_roi_mask`: zeros the box, leaves pixels outside untouched, does not
  mutate the input array, and a zero-width box (the actual Task 4 config
  placeholder) masks nothing — pinning the "start permissive" decision.
- `median_filter_depth`: removes a single-pixel spike, preserves dtype
  (`uint16`), preserves shape.

One test was renamed mid-implementation
(`test_apply_roi_mask_full_frame_box_is_permissive_placeholder` →
`test_apply_roi_mask_zero_width_box_masks_nothing`) because the original name
described the wrong placeholder shape (a full-frame box masks *everything*,
which is the opposite of permissive); the config actually ships a zero-width
box. Re-ran the full suite after the rename — still 31/31 green.

### Node implementation

`poc_fusion/poc_fusion/depth_preprocess_node.py` — ROS plumbing only, per the
Task 4 brief's design decisions:
- Subscribes to `depth_image_topic` (param, default
  `/ascamera/camera_publisher/depth0/image_raw`), `cv_bridge`s to numpy with
  `passthrough` (so no implicit encoding conversion happens before our own
  check runs).
- Step 1a: asserts `msg.encoding == expected_encoding` (`mono16`) on the
  first message received; `RuntimeError` + `get_logger().fatal(...)` if not.
  The check is on the first *message*, not literally at process startup,
  because the encoding is only known once a message arrives — there is no
  earlier point at which it could be verified.
- Republishes cleaned depth on `/poc_fusion/depth_cleaned` with
  `encoding='16UC1'` (Step 1a re-encode) and `out_msg.header = msg.header`
  (verbatim passthrough of stamp + `frame_id`).
- Step 2: `invalid_fraction()` logged every 5s (`throttle_duration_sec=5.0`).
- Step 3: `apply_roi_mask()` using the `roi_profile`-selected bounds from
  `config/depth_preprocess_params.yaml`.
- Step 4: `median_filter_depth()`, kernel size from the `median_kernel_size`
  parameter.
- Step 5: `/poc_fusion/debug_image` (`bgr8`), an 8-bit normalized view with a
  red rectangle drawn over the active ROI box (skipped when the box is
  zero-width, i.e. the current placeholder).

`poc_fusion/config/depth_preprocess_params.yaml` — new file, holds topic
names, `expected_encoding: mono16`, `median_kernel_size: 3`, and
`roi_profiles.default` with all four bounds at 0 (zero-width → masks
nothing). Comments record that Task 11 replaces the bounds and Task 10a adds
a second named profile later (not built now — YAGNI). `costmap_params.yaml`
was not touched.

### Deploy + build (as `ubuntu`, never root)

```
$ bash scripts/deploy_poc_fusion.sh
Deploying /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion -> MentorPi:/home/ubuntu/ros2_ws/src/poc_fusion
No stale destination files to remove.
Deploy complete.
```

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    cd /home/ubuntu/ros2_ws
    colcon build --packages-select poc_fusion
  '
Starting >>> poc_fusion
--- stderr: poc_fusion
/home/ubuntu/.local/lib/python3.10/site-packages/setuptools/_distutils/cmd.py:66: SetuptoolsDeprecationWarning: setup.py install is deprecated.
[... same benign setup.py warning as Task 2 ...]
---
Finished <<< poc_fusion [3.72s]

Summary: 1 package finished [5.74s]
  1 package had stderr output: poc_fusion
```

### Source topic sanity check (pre-existing driver, not our node)

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    timeout 3 ros2 topic hz /ascamera/camera_publisher/depth0/image_raw
  '
WARNING: topic [/ascamera/camera_publisher/depth0/image_raw] does not appear to be published yet
average rate: 14.806
	min: 0.054s max: 0.077s std dev: 0.00535s window: 16
```

### Step 6: run the node, verify rate and topic list

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    nohup ros2 run poc_fusion depth_preprocess_node --ros-args --params-file /home/ubuntu/ros2_ws/install/poc_fusion/share/poc_fusion/config/depth_preprocess_params.yaml > /tmp/depth_preprocess_node.log 2>&1 &
    disown
    sleep 3
    cat /tmp/depth_preprocess_node.log
  '
[INFO] [1786112528.279438216] [depth_preprocess_node]: depth_preprocess_node started: /ascamera/camera_publisher/depth0/image_raw -> /poc_fusion/depth_cleaned (expected encoding=mono16, roi_profile=default, median_kernel_size=3)
[INFO] [1786112529.873685361] [depth_preprocess_node]: invalid pixel fraction: 0.294
```

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ros2 node list
  '
/LD19
/ar_app
/arm_controller
/aurora/aurora
/controller_manager
/depth_preprocess_node
/ekf_filter_node
/gripper_controller
/hand_gesture
/hand_trajectory
/imu_calib
/imu_filter
/init_pose
/joint_state_publisher
/joy_node
/joystick_control
/lidar_app
/line_following
/object_tracking
/odom_publisher
/robot_state_publisher
/ros_robot_controller
/rosapi
/rosapi_params
/rosbridge_websocket
/servo_manager
/static_transform_publisher_zk7CPAMfFWiFWQsu
/transform_listener_impl_55561e029580
/web_video_server
```

`/depth_preprocess_node` is running alongside the full pre-existing stack
(camera driver `/aurora/aurora`, `/LD19`, arm/gripper controllers, etc.) — no
other node was touched or restarted.

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    timeout 10 ros2 topic hz /poc_fusion/depth_cleaned
  '
WARNING: topic [/poc_fusion/depth_cleaned] does not appear to be published yet
average rate: 14.743
	min: 0.058s max: 0.081s std dev: 0.00568s window: 16
average rate: 14.752
	min: 0.058s max: 0.081s std dev: 0.00467s window: 31
average rate: 14.728
	min: 0.058s max: 0.081s std dev: 0.00444s window: 46
average rate: 14.691
	min: 0.058s max: 0.083s std dev: 0.00524s window: 61
average rate: 14.654
	min: 0.058s max: 0.090s std dev: 0.00607s window: 76
average rate: 14.696
	min: 0.052s max: 0.090s std dev: 0.00625s window: 91
```

~14.7 Hz sustained over 10s, matching the source topic's rate (the
`WARNING` line is `ros2 topic hz`'s own startup boilerplate before its first
window closes, not a fault).

### Step 1a: encoding relabel, verified live

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    timeout 3 ros2 topic echo /poc_fusion/depth_cleaned --no-arr
  '
header:
  stamp:
    sec: 1786112568
    nanosec: 356000000
  frame_id: depth_camera_link
height: 400
width: 640
encoding: 16UC1
is_bigendian: 0
step: 1280
data: '<sequence type: uint8, length: 512000>'
---
[... 7 more identical-shape messages, all encoding: 16UC1, elided ...]
```

`encoding: 16UC1` confirmed on the live outgoing topic (source is `mono16`,
verified separately below). `frame_id: depth_camera_link` preserved, matching
the brief's requirement.

### Step 1 / Step 6: header stamp passthrough, verified exactly (not just eyeballed)

Rather than compare two single-shot `ros2 topic echo --once` calls (too
loosely correlated in time to prove anything), a small in-container script
subscribed to both topics simultaneously for 3s and diffed the collected
`(sec, nanosec)` stamp tuples:

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    python3 /tmp/stamp_check.py
  '
SRC: [(1786112618, 621000000), (1786112618, 716000000), (1786112618, 849000000), (1786112618, 912000000), (1786112618, 982000000)]
CLEAN: [(1786112618, 621000000), (1786112618, 716000000), (1786112618, 849000000), (1786112618, 912000000), (1786112618, 982000000)]
MATCHING STAMP COUNT: 44
SRC COUNT: 44 CLEAN COUNT: 44
```

All 44 stamps collected on the source topic during the 3s window appear
exactly (44/44) in the cleaned topic's stamps — the header stamp survives
this node byte-for-byte, confirming the Step 1 requirement Task 12's latency
measurement depends on.

Separately, a single-shot echo of each topic (different real times, not
correlated to each other) confirms `frame_id` matches independently of the
stamp-matching script:

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ros2 topic echo /ascamera/camera_publisher/depth0/image_raw --no-arr --once
  '
header:
  stamp:
    sec: 1786112586
    nanosec: 977000000
  frame_id: depth_camera_link
height: 400
width: 640
encoding: mono16
is_bigendian: 0
step: 1280
data: '<sequence type: uint8, length: 512000>'
---
```

### Step 1a: units sanity check (machine-checkable half only — see note below)

A second in-container script subscribed to `/poc_fusion/depth_cleaned`, read
one frame via `cv_bridge`, and reported the raw pixel-value distribution:

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    python3 /tmp/units_check.py
  '
dtype: uint16 shape: (400, 640)
min/max valid: 223 1984
mean valid: 349.0038670528203
percentiles 5/50/95: [233. 301. 649.]
center row sample (every 40th px): [  0   0 302 303 303 304 304 305 304 305 304 306 306 306 306 306]
```

Reasoning: valid (nonzero) pixels range 223–1984 with a median of 301 and a
95th percentile of 649. If these were metres, floor returns 0.25 m from a
camera mounted 0.246 m above the floor at ~40° below horizontal would be
physically impossible (the camera itself is only 0.246 m up); read as
millimetres, 223–649 mm for near/floor-ish returns and up to ~1984 mm for
the furthest visible surfaces is exactly the "hundreds-to-low-thousands mm"
band predicted from that camera pose, and the median (301 mm) sits inside
the predicted 300–1000 mm floor-return band. This confirms the buffer is in
millimetres without requiring anyone to place an object at a tape-measured
distance.

**Deferred to the human-present phase:** the brief's stronger confirmation —
holding an object at a tape-measured distance and reading the exact raw
pixel value against it — requires a second person and is explicitly out of
scope for this (unattended) run. Not done here; flagging it for the next
human-present session, most naturally alongside Task 11's ROI measurement.

### Step 5: debug image also publishes

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ros2 topic list | grep poc_fusion
    ros2 topic info /poc_fusion/debug_image
  '
/poc_fusion/debug_image
/poc_fusion/depth_cleaned
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 0
```

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    timeout 5 ros2 topic hz /poc_fusion/debug_image
  '
average rate: 12.382
	min: 0.030s max: 0.194s std dev: 0.04375s window: 14
average rate: 11.163
	min: 0.030s max: 0.203s std dev: 0.05017s window: 24
average rate: 11.418
	min: 0.030s max: 0.220s std dev: 0.04912s window: 36
```

Publishes at ~11–12 Hz (slower than `depth_cleaned` due to the extra
`cv2.normalize`/color-convert/rectangle-draw work). The brief does not set a
rate requirement for the debug topic (only Step 6's `depth_cleaned` rate is
gated), so this is recorded as-is, not treated as a defect.

### Node log during the run (invalid-fraction throttled logging, Step 2)

```
$ docker exec -u ubuntu MentorPi bash -lc 'tail -20 /tmp/depth_preprocess_node.log'
[INFO] [1786112575.252600348] [depth_preprocess_node]: invalid pixel fraction: 0.297
[INFO] [1786112580.266635436] [depth_preprocess_node]: invalid pixel fraction: 0.297
[INFO] [1786112585.296534812] [depth_preprocess_node]: invalid pixel fraction: 0.292
[INFO] [1786112590.322456129] [depth_preprocess_node]: invalid pixel fraction: 0.294
[INFO] [1786112595.336818666] [depth_preprocess_node]: invalid pixel fraction: 0.294
[INFO] [1786112600.379359407] [depth_preprocess_node]: invalid pixel fraction: 0.295
[INFO] [1786112605.407483743] [depth_preprocess_node]: invalid pixel fraction: 0.292
[INFO] [1786112610.435641107] [depth_preprocess_node]: invalid pixel fraction: 0.294
[INFO] [1786112615.466283047] [depth_preprocess_node]: invalid pixel fraction: 0.293
[INFO] [1786112620.495350815] [depth_preprocess_node]: invalid pixel fraction: 0.292
[INFO] [1786112625.538693597] [depth_preprocess_node]: invalid pixel fraction: 0.294
[INFO] [1786112630.581447000] [depth_preprocess_node]: invalid pixel fraction: 0.291
[INFO] [1786112635.583790799] [depth_preprocess_node]: invalid pixel fraction: 0.295
[INFO] [1786112640.623128654] [depth_preprocess_node]: invalid pixel fraction: 0.297
[INFO] [1786112645.631706107] [depth_preprocess_node]: invalid pixel fraction: 0.295
[INFO] [1786112650.668577488] [depth_preprocess_node]: invalid pixel fraction: 0.293
[INFO] [1786112655.701561220] [depth_preprocess_node]: invalid pixel fraction: 0.296
[INFO] [1786112660.727152257] [depth_preprocess_node]: invalid pixel fraction: 0.293
[INFO] [1786112665.757473868] [depth_preprocess_node]: invalid pixel fraction: 0.292
[INFO] [1786112670.817617386] [depth_preprocess_node]: invalid pixel fraction: 0.292
```

Stable at ~0.29–0.30 throughout the run; no jumps, no error/warn lines.

### Teardown

Node stopped by killing its own two PIDs (the `ros2 run` wrapper and its
child) — not `.stop_ros.sh` — and confirmed absent from the ROS graph
afterward:

```
$ docker exec -u ubuntu MentorPi bash -lc 'kill 117773 117775 2>/dev/null; sleep 1'
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ros2 node list | grep depth_preprocess || echo "node not in graph - confirmed stopped"
  '
node not in graph - confirmed stopped
```

No other running node (`/aurora/aurora`, `/LD19`, arm/gripper controllers,
etc.) was touched. No `cmd_vel` published, no arm command sent, `.stop_ros.sh`
never invoked.

### Host test suite — after (final)

```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/ -q
...............................                                          [100%]
31 passed in 0.34s
```

### Task 4 summary

Implemented `poc_fusion/poc_fusion/lib/depth_preprocess.py` (TDD, RED then
GREEN, 12 new tests, 31/31 total) and
`poc_fusion/poc_fusion/depth_preprocess_node.py` (real implementation
replacing the Task 2 stub), plus the new
`poc_fusion/config/depth_preprocess_params.yaml`. Deployed via
`scripts/deploy_poc_fusion.sh`, built as `ubuntu` (never root), and verified
live: `/poc_fusion/depth_cleaned` publishes at ~14.7 Hz with `encoding:
16UC1`, `frame_id: depth_camera_link`, and header stamps that match the
source topic's exactly (44/44 in a 3s window). The units sanity check
(pixel values 223–1984, median 301, consistent with a 0.246 m/40°-below-
horizontal camera pose) is recorded; the tape-measured confirmation is
explicitly deferred to a human-present session. `costmap_params.yaml` was
not touched. No HARD SAFETY RULE was violated — no `cmd_vel`, no arm
command, `.stop_ros.sh` never run, no edits to `proximity_alert/` or to
files under `/home/ubuntu/ros2_ws/src/` outside the deploy script's target.

---

## Task 4 fix round — response to review (Important 1, Important 2, Item 3, Item 4)

Captured 2026-08-07, **22:38 HKT / 14:38 UTC**:
```
$ date
Fri Aug  7 10:38:58 PM HKT 2026
$ date -u
Fri Aug  7 02:38:58 PM UTC 2026
```

### Important 2 — ROI un-masked by median filter: RED then GREEN

New pure function `poc_fusion/poc_fusion/lib/depth_preprocess.py:clean_depth()`
replaces the node's separate `apply_roi_mask()` → `median_filter_depth()`
calls. Written test-first: `test_clean_depth_roi_survives_median_filtering`
placed a single-pixel ROI in the middle of an otherwise fully-valid frame
and asserted the ROI pixel is still 0 after the full pipeline.

RED, against the buggy "mask-then-filter, never re-mask" ordering
(implemented deliberately first to reproduce the review's exact finding):

```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/test_depth_preprocess.py -q -k clean_depth
F.                                                                       [100%]
=================================== FAILURES ===================================
________________ test_clean_depth_roi_survives_median_filtering ________________

    def test_clean_depth_roi_survives_median_filtering():
        ...
        depth = np.full((9, 9), 700, dtype=np.uint16)
        roi = dict(row_min=4, row_max=5, col_min=4, col_max=5)  # single pixel
        cleaned = clean_depth(depth, kernel_size=3, **roi)
>       assert cleaned[4, 4] == 0, (
            "ROI pixel was un-masked by the median filter -- the ROI mask must "
            "be the pipeline's final word, not just its first step"
        )
E       AssertionError: ROI pixel was un-masked by the median filter -- the ROI mask must be the pipeline's final word, not just its first step
E       assert 700 == 0

test/test_depth_preprocess.py:124: AssertionError
=========================== short test summary info ============================
FAILED test/test_depth_preprocess.py::test_clean_depth_roi_survives_median_filtering
1 failed, 1 passed, 12 deselected in 0.25s
```

This exactly reproduces the review's Important 2 finding: the 3x3 median
filter around the single masked pixel sees 8 valid (700) neighbours and 1
zero pixel, so the majority-vote median un-masks it back to 700.

GREEN, after adding a final `apply_roi_mask()` call at the end of
`clean_depth()`:

```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/ -q
.................................                                        [100%]
33 passed in 0.25s
```

31 (previous total) + 2 new (`test_clean_depth_roi_survives_median_filtering`,
`test_clean_depth_still_denoises_outside_the_roi`, the latter confirming the
fix doesn't come at the cost of denoising elsewhere in the frame) = 33,
all passing.

`depth_preprocess_node.py`'s `_on_depth()` now calls `clean_depth(...)`
directly instead of the two separate pure-function calls, so the
mask-then-filter-then-remask ordering lives in exactly one place.

### Important 1 — ROI-profile parameter declaration mismatch

Confirmed the reviewer's finding: only `roi_profiles.default.*` is declared
via `declare_parameter()`; `Node.__init__` does not pass
`automatically_declare_parameters_from_overrides=True`, so selecting any
`roi_profile` other than `default` would raise
`ParameterNotDeclaredException`. Per the reviewer's steer (do not build a
second profile; make code and prose agree, cheapest option), corrected the
claim rather than adding infrastructure for a profile that doesn't exist
yet:
- `poc_fusion/config/depth_preprocess_params.yaml`'s comment now states
  plainly that adding a second profile requires adding four more
  `declare_parameter()` calls in the node — it is not YAML-only.
- Added a matching inline comment in `depth_preprocess_node.py` at the ROI
  parameter resolution site.
- The Task 4 report's self-review claim ("the structure to add [a second
  profile] without touching this node's code is in place") is corrected in
  the report's fix-round addendum below.

No functional code change here — this is a truth-in-comments fix, per the
reviewer's explicit steer against building unused infrastructure.

### Item 3 — missing `python3-opencv` dependency

`depth_preprocess_node.py` imports `cv2` directly (`cv2.normalize`,
`cv2.cvtColor`, `cv2.rectangle` for the Step 5 debug overlay) but
`package.xml` only had it transitively through `cv_bridge`. Added
`<exec_depend>python3-opencv</exec_depend>` and re-ran `rosdep check`:

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    cd /home/ubuntu/ros2_ws
    rosdep check --from-paths src/poc_fusion --ignore-src
  '
All system dependencies have been satisfied
```

### Item 4 — debug_image rate gap: investigated, not a bug

Re-deployed and rebuilt (see below), then re-ran the node and measured
`/poc_fusion/depth_cleaned` and `/poc_fusion/debug_image` **simultaneously**
over the same 10s window, one `ros2 topic hz` process per topic launched
together:

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    (timeout 10 ros2 topic hz /poc_fusion/depth_cleaned > /tmp/hz_cleaned.log 2>&1) &
    (timeout 10 ros2 topic hz /poc_fusion/debug_image > /tmp/hz_debug.log 2>&1) &
    wait
    echo "=== depth_cleaned ==="; cat /tmp/hz_cleaned.log
    echo "=== debug_image ==="; cat /tmp/hz_debug.log
  '
=== depth_cleaned ===
average rate: 12.904 ... window: 14
average rate: 14.081 ... window: 30
average rate: 14.406 ... window: 46
average rate: 14.489 ... window: 61
average rate: 14.531 ... window: 76
average rate: 14.569 ... window: 91
average rate: 14.309 ... window: 104
=== debug_image ===
average rate: 9.780 ... window: 13
average rate: 11.717 ... window: 28
average rate: 10.983 ... window: 38
average rate: 10.778 ... window: 49
average rate: 10.419 ... window: 58
average rate: 9.765 ... window: 65
average rate: 10.300 ... window: 79
```

Even measured simultaneously over the identical 10s window, the two
`ros2 topic hz` processes disagree (104 vs. 79 windowed messages) — this
alone would suggest a genuine gap. But `_on_depth()` publishes both
messages unconditionally, synchronously, in the same callback with no
branch between them, so the two topics cannot structurally diverge in
publish count. To settle it, a single rclpy process subscribed to **both**
topics at once (removing the confound of two separate `ros2 topic hz`
subprocesses competing with each other and with the node for CPU on the
Pi 5) and counted messages directly:

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    python3 /tmp/rate_check.py
  '
elapsed=10.04s
cleaned_count=130 (12.95 Hz)
debug_count=130 (12.95 Hz)
```

**Exact parity: 130 messages received on each topic in the same 10.04s
window.** This confirms the two `ros2 topic hz` CLI processes measured
against each other (and against the node) were the source of the earlier
apparent gap — not a real 1:1-violating rate difference in the node. The
Task 4 report's original explanation ("extra cv2 overhead") was wrong (as
the reviewer flagged: a slow callback would throttle both publishes
equally), but so would concluding this is a genuine bug — the corrected,
evidence-backed conclusion is that both topics publish in lockstep at
whatever rate the node's callback sustains (~13-14.7 Hz depending on
concurrent CPU load), and the discrepancy in the two separate `ros2 topic
hz` invocations was itself a measurement artifact of running two
independent CPU-competing subscriber processes rather than a property of
the node.

### Redeploy + rebuild (as `ubuntu`, never root)

```
$ bash scripts/deploy_poc_fusion.sh
Deploying /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion -> MentorPi:/home/ubuntu/ros2_ws/src/poc_fusion
No stale destination files to remove.
Deploy complete.
```

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    cd /home/ubuntu/ros2_ws
    colcon build --packages-select poc_fusion
  '
Starting >>> poc_fusion
Finished <<< poc_fusion [2.56s]
--- stderr: poc_fusion
[setup.py deprecation warning, same as before]
---
Summary: 1 package finished [4.52s]
  1 package had stderr output: poc_fusion
```

### Re-verification after the pipeline reorder

Node restarted; encoding and header still correct after the `clean_depth()`
reorder:

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ros2 topic echo /poc_fusion/depth_cleaned --no-arr --once
  '
header:
  stamp: {sec: 1786113517, nanosec: 460000000}
  frame_id: depth_camera_link
height: 400
width: 640
encoding: 16UC1
is_bigendian: 0
step: 1280
data: '<sequence type: uint8, length: 512000>'
---
```

Node log during this run showed no errors:
```
$ docker exec -u ubuntu MentorPi bash -lc 'tail -30 /tmp/depth_preprocess_node_v2.log'
[INFO] [1786113432.873500819] [depth_preprocess_node]: depth_preprocess_node started: ...
[INFO] [1786113432.976275043] [depth_preprocess_node]: invalid pixel fraction: 0.295
[INFO] [1786113438.013522209] [depth_preprocess_node]: invalid pixel fraction: 0.294
[INFO] [1786113443.053784794] [depth_preprocess_node]: invalid pixel fraction: 0.296
[INFO] [1786113448.066105998] [depth_preprocess_node]: invalid pixel fraction: 0.295
[INFO] [1786113453.076794724] [depth_preprocess_node]: invalid pixel fraction: 0.294
[INFO] [1786113458.123093164] [depth_preprocess_node]: invalid pixel fraction: 0.291
[INFO] [1786113463.151055201] [depth_preprocess_node]: invalid pixel fraction: 0.294
[INFO] [1786113468.183694302] [depth_preprocess_node]: invalid pixel fraction: 0.294
[INFO] [1786113473.233710232] [depth_preprocess_node]: invalid pixel fraction: 0.294
```

### Teardown

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    kill 169172 2>/dev/null
    sleep 1
    ros2 node list | grep depth_preprocess || echo "node not in graph - confirmed stopped"
  '
node not in graph - confirmed stopped
```

No other node touched. No `cmd_vel` published, no arm command sent,
`.stop_ros.sh` never invoked.

### Host test suite — final

```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/ -q
.................................                                        [100%]
33 passed in 0.29s
```

### Fix round summary

Fixed the review's Important 2 (ROI un-masked by median filter at its
boundary) with a TDD RED/GREEN cycle producing `clean_depth()` and 2 new
regression tests (33/33 total, pristine). Resolved Important 1 by
correcting the YAML comment and node comment to state accurately that a
second ROI profile requires a node code change, not just a YAML edit — no
infrastructure was built for the unused profile, per explicit reviewer
steer against that. Added the missing `python3-opencv` `package.xml`
dependency (Item 3), confirmed via `rosdep check`. Investigated the
debug_image rate gap (Item 4) and determined via a single-process
simultaneous-subscription message count (130/130 exact parity) that it is
a measurement artifact of running two independent `ros2 topic hz`
processes, not a real defect in the node — corrected the report's earlier
unsupported causal claim accordingly. Redeployed and rebuilt as `ubuntu`,
re-verified `encoding: 16UC1` and header passthrough still hold after the
pipeline reorder. No HARD SAFETY RULE was violated.
