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

---

## Task 5: Point cloud generation

Captured **2026-08-08, 00:17 HKT / 2026-08-07 16:17 UTC**:
```
$ date
Sat Aug  8 12:17:30 AM HKT 2026
$ date -u
Fri Aug  7 04:17:30 PM UTC 2026
```

### Step 0 — Dependency preflight (re-confirmation)

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ros2 pkg prefix depth_image_proc
    ros2 pkg prefix image_proc
  '
/opt/ros/humble
/opt/ros/humble
```
Both present, unchanged from Task 0's install. Also confirmed the specific
composable-node classes and executables this task needs exist:
```
$ ros2 component types | grep -A11 '^image_proc$'
image_proc
  image_proc::RectifyNode
  image_proc::DebayerNode
  image_proc::ResizeNode
  image_proc::CropDecimateNode
  image_proc::CropNonZeroNode

$ ros2 component types | grep -A10 '^depth_image_proc$'
depth_image_proc
  depth_image_proc::ConvertMetricNode
  depth_image_proc::CropForemostNode
  depth_image_proc::DisparityNode
  depth_image_proc::PointCloudXyzNode
  ...
```
`image_proc::RectifyNode` and `depth_image_proc::PointCloudXyzNode` both
present — no BLOCKED condition.

### Host test suite — before

```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/ -q
.....................................                                    [100%]
37 passed in 1.81s
```
(37 = 33 carried over from the Task 4 fix round + the 4 new
`camera_info_watchdog` tests written for this task, below, before this
"before" run — the RED/GREEN cycle for those tests is shown in the next
section and was run prior to this snapshot; it is quoted separately because
`pytest` at this snapshot already includes them.)

### TDD: `poc_fusion/poc_fusion/lib/camera_info_watchdog.py`

Step 4 of the brief says to log a visible warning if
`/ascamera/camera_publisher/depth0/camera_info` is not received within 5s of
startup, since `depth_image_proc::PointCloudXyzNode` (a vendor C++ node,
off-limits under the additive-only constraint) fails silently (no points,
no error) without it. The only piece of this that is pure, testable logic
is the decision "given whether camera_info has been received and how much
time has elapsed, should the node warn right now" — extracted into
`should_warn(received, elapsed_sec, timeout_sec)`.

RED (module does not exist yet):
```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/test_camera_info_watchdog.py -q
==================================== ERRORS ====================================
______________ ERROR collecting test/test_camera_info_watchdog.py ______________
ImportError while importing test module '/home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion/test/test_camera_info_watchdog.py'.
Hint: make sure your test modules/packages have valid Python names.
Traceback:
/usr/lib/python3.11/importlib/__init__.py:126: in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
test/test_camera_info_watchdog.py:1: in <module>
    from poc_fusion.lib.camera_info_watchdog import should_warn
E   ModuleNotFoundError: No module named 'poc_fusion.lib.camera_info_watchdog'
=========================== short test summary info ============================
ERROR test/test_camera_info_watchdog.py
!!!!!!!!!!!!!!!!!!!! Interrupted: 1 error during collection !!!!!!!!!!!!!!!!!!!!
1 error in 0.07s
```

GREEN, after writing `poc_fusion/poc_fusion/lib/camera_info_watchdog.py`:
```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/ -q
.....................................                                    [100%]
37 passed in 0.29s
```
4 new tests (33 + 4 = 37): not-received-past-timeout warns; received
before timeout does not warn; not-received-but-before-timeout does not warn
(guards against a misconfigured early-firing timer); received-even-past-
timeout does not warn (pins the "lateness is moot once received" case for a
callback that races against arrival on the same tick). Each test targets a
distinct branch/boundary in `should_warn()` and would fail if that branch's
logic were wrong or reversed — e.g. flipping the `received` short-circuit
would fail `test_does_not_warn_when_received_even_past_timeout`, and an
off-by-one on the `>=` comparison would fail
`test_does_not_warn_before_timeout_elapses`.

### Node wiring: `depth_preprocess_node.py`

Added (ROS plumbing only, delegating the decision to `should_warn()`):
- A `CameraInfo` subscription to
  `/ascamera/camera_publisher/depth0/camera_info` (module constant
  `CAMERA_INFO_TOPIC`, not a declared parameter — this topic name is fixed
  by the driver, unlike `depth_image_topic` which is a legitimate
  operator-facing knob) that sets `self._camera_info_received = True` and
  cancels the watchdog timer.
- A one-shot-in-effect `create_timer(5.0, ...)` that calls
  `should_warn(received, elapsed_sec, timeout_sec=5.0)`; if it returns
  `True`, logs `get_logger().warn(...)`. The timer cancels *itself* on its
  first firing (rclpy's `create_timer` is periodic by default; humble does
  not expose a native one-shot flag on the Python API used here), so the
  check runs exactly once regardless of whether camera_info showed up.

No approximate-intrinsics fallback was added, per the brief's explicit
instruction — the watchdog only logs; it never substitutes a value.

### Launch file: `poc_fusion/launch/poc_fusion.launch.py` (new)

Structure, matching the brief's Decision 1:
- `depth_preprocess_node` (Task 4, plain `Node` action — not composable;
  rclpy's composable-node support doesn't justify force-fitting an already-
  shipped standalone node into the container).
- `poc_fusion_container` (`rclcpp_components::component_container`,
  namespace `/poc_fusion`) holding two composable nodes:
  - `image_proc::RectifyNode` (name `depth_rectify_node`), `interpolation: 0`
    parameter override (Step 2 — nearest-neighbour), remapped
    `image ← /poc_fusion/depth_cleaned`,
    `camera_info ← /ascamera/camera_publisher/depth0/camera_info`,
    `image_rect → /poc_fusion/depth_rect`.
  - `depth_image_proc::PointCloudXyzNode` (name `point_cloud_xyz_node`),
    remapped `image_rect ← /poc_fusion/depth_rect`,
    `camera_info ← /ascamera/camera_publisher/depth0/camera_info`,
    `points → /poc_fusion/points`.
- Comments mark exactly where Task 6's costmap composable node(s) and Task
  8's lifecycle-manager wiring belong, so those additions are additive to
  this same file rather than requiring a restructure. Neither was built
  here.

`package.xml` gained `launch`, `launch_ros`, `rclcpp_components`, and
`ament_index_python` exec_depends (needed by the launch file itself; not
previously declared since no launch file existed before this task).
`poc_fusion/config/costmap_params.yaml` was not touched — confirmed by
inspection, it still holds only `scan_topic: /scan_raw`.

### Deploy + build (as `ubuntu`, never root)

```
$ bash scripts/deploy_poc_fusion.sh
Deploying /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion -> MentorPi:/home/ubuntu/ros2_ws/src/poc_fusion
Removing stale destination files (absent from host source):
  launch/.gitkeep
Deploy complete.
```
(`launch/.gitkeep` removed from the host tree once a real launch file
existed to keep the directory non-empty in git; the deploy script correctly
pruned the now-stale copy from the container destination.)

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    cd /home/ubuntu/ros2_ws
    rosdep check --from-paths src/poc_fusion --ignore-src
  '
All system dependencies have been satisfied

$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    cd /home/ubuntu/ros2_ws
    colcon build --packages-select poc_fusion
  '
Starting >>> poc_fusion
Finished <<< poc_fusion [3.34s]
--- stderr: poc_fusion
[same benign setup.py deprecation warning as Tasks 2/4]
---
Summary: 1 package finished [5.14s]
  1 package had stderr output: poc_fusion
```

### Baseline node list (before launching our pipeline)

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
/static_transform_publisher_bXOTCq6Rcx20Hpr7
/transform_listener_impl_5555df8f5580
/web_video_server
```

### Launch, and confirm all our nodes came up

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    nohup ros2 launch poc_fusion poc_fusion.launch.py > /tmp/poc_fusion_launch.log 2>&1 &
    disown
    sleep 6
    cat /tmp/poc_fusion_launch.log
  '
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2026-08-07-16-07-37-155778-raspberrypi-23364
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [depth_preprocess_node-1]: process started with pid [23377]
[INFO] [component_container-2]: process started with pid [23379]
[component_container-2] [INFO] [1786118857.553532156] [poc_fusion.poc_fusion_container]: Load Library: /opt/ros/humble/lib/librectify.so
[component_container-2] [INFO] [1786118858.421200951] [poc_fusion.poc_fusion_container]: Found class: rclcpp_components::NodeFactoryTemplate<image_proc::RectifyNode>
[component_container-2] [INFO] [1786118858.421256988] [poc_fusion.poc_fusion_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<image_proc::RectifyNode>
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/poc_fusion/depth_rectify_node' in container '/poc_fusion/poc_fusion_container'
[component_container-2] [INFO] [1786118858.623458777] [poc_fusion.poc_fusion_container]: Load Library: /opt/ros/humble/lib/libdepth_image_proc.so
[component_container-2] [INFO] [1786118858.813659673] [poc_fusion.poc_fusion_container]: Found class: rclcpp_components::NodeFactoryTemplate<depth_image_proc::ConvertMetricNode>
[component_container-2] [INFO] [1786118858.813735821] [poc_fusion.poc_fusion_container]: Found class: rclcpp_components::NodeFactoryTemplate<depth_image_proc::CropForemostNode>
[component_container-2] [INFO] [1786118858.813749265] [poc_fusion.poc_fusion_container]: Found class: rclcpp_components::NodeFactoryTemplate<depth_image_proc::DisparityNode>
[component_container-2] [INFO] [1786118858.813757784] [poc_fusion.poc_fusion_container]: Found class: rclcpp_components::NodeFactoryTemplate<depth_image_proc::PointCloudXyzNode>
[component_container-2] [INFO] [1786118858.813766673] [poc_fusion.poc_fusion_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<depth_image_proc::PointCloudXyzNode>
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/poc_fusion/point_cloud_xyz_node' in container '/poc_fusion/poc_fusion_container'
[depth_preprocess_node-1] [INFO] [1786118859.419062336] [depth_preprocess_node]: depth_preprocess_node started: /ascamera/camera_publisher/depth0/image_raw -> /poc_fusion/depth_cleaned (expected encoding=mono16, roi_profile=default, median_kernel_size=3)
[depth_preprocess_node-1] [INFO] [1786118859.422962394] [depth_preprocess_node]: invalid pixel fraction: 0.285
[component_container-2] [WARN] [1786118859.588614241] [poc_fusion.depth_rectify_node]: [image_transport] Topics '/poc_fusion/depth_cleaned' and '/poc_fusion/camera_info' do not appear to be synchronized. In the last 10s:
[component_container-2] 	Image messages received:      0
[component_container-2] 	CameraInfo messages received: 9
[component_container-2] 	Synchronized pairs:           0
```
The `[image_transport] ... do not appear to be synchronized` WARN is
`image_transport::CameraSubscriber`'s own startup boilerplate reporting on
its first (empty) 10s window — at the moment it logged, the container had
been up ~35ms and no image had yet round-tripped through
`depth_preprocess_node`. It is a transient, not a misconfiguration; see the
remapping check immediately below, which shows the actual subscribed topics
are correct.

### Step 3: remappings verified via `ros2 node info` (not by eye)

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ros2 node info /poc_fusion/depth_rectify_node
  '
/poc_fusion/depth_rectify_node
  Subscribers:
    /ascamera/camera_publisher/depth0/camera_info: sensor_msgs/msg/CameraInfo
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /poc_fusion/depth_cleaned: sensor_msgs/msg/Image
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /poc_fusion/depth_rect: sensor_msgs/msg/Image
    /poc_fusion/image_rect/compressed: sensor_msgs/msg/CompressedImage
    /poc_fusion/image_rect/compressedDepth: sensor_msgs/msg/CompressedImage
    /poc_fusion/image_rect/theora: theora_image_transport/msg/Packet
    /rosout: rcl_interfaces/msg/Log
  [service server/client sections omitted -- none relevant]

$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ros2 node info /poc_fusion/point_cloud_xyz_node
  '
/poc_fusion/point_cloud_xyz_node
  Subscribers:
    /ascamera/camera_publisher/depth0/camera_info: sensor_msgs/msg/CameraInfo
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /poc_fusion/depth_rect: sensor_msgs/msg/Image
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /poc_fusion/points: sensor_msgs/msg/PointCloud2
    /rosout: rcl_interfaces/msg/Log

$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ros2 topic list | grep poc_fusion
  '
/poc_fusion/debug_image
/poc_fusion/depth_cleaned
/poc_fusion/depth_rect
/poc_fusion/image_rect/compressed
/poc_fusion/image_rect/compressedDepth
/poc_fusion/image_rect/theora
/poc_fusion/points
```
Exactly the remap targets the brief specifies: rectify subscribes
`/poc_fusion/depth_cleaned` + `/ascamera/camera_publisher/depth0/camera_info`
and publishes `/poc_fusion/depth_rect`; point_cloud_xyz subscribes
`/poc_fusion/depth_rect` + the same camera_info topic and publishes
`/poc_fusion/points`. (The extra `image_rect/compressed*`/`theora` topics
are `image_transport`'s automatic alternate-transport publishers, not
something this launch file requested — harmless, standard image_transport
behaviour, not evidence of a wiring error.)

### Step 2: nearest-neighbour interpolation, verified by parameter readback

```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ros2 param list /poc_fusion/depth_rectify_node
  '
  .image_rect.format
  .image_rect.jpeg_quality
  .image_rect.png_level
  .image_rect.tiff.res_unit
  .image_rect.tiff.xdpi
  .image_rect.tiff.ydpi
  interpolation
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  queue_size
  use_sim_time

$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ros2 param get /poc_fusion/depth_rectify_node interpolation
  '
Integer value is: 0
```
The parameter's real name on Humble's `image_proc::RectifyNode` is
`interpolation` (a plain top-level param, confirmed via the live
`ros2 param list` above rather than assumed), and it takes an OpenCV
interpolation-flag integer. `0 == cv2.INTER_NEAREST` — confirmed nearest-
neighbour, not the linear default (`1 == cv2.INTER_LINEAR`), matches the
`{'interpolation': 0}` override set in the launch file.

### Step 4: camera_info watchdog, verified both ways live

**Negative path (camera_info present — no false warning):** the full launch
log above and its later portions were grepped for the warning text; only
the transient image_transport sync WARN appears, no watchdog warning fired
across the whole run, matching the fact that
`/ascamera/camera_publisher/depth0/camera_info` is actively published by
the live driver.

**Positive path (camera_info withheld — warning does fire):** rather than
disturb the live `/ascamera/aurora` driver (which also serves RGB and other
topics, and stopping it would violate the additive-only constraint), a
second, fully isolated instance of `depth_preprocess_node` was launched
with only its `camera_info` subscription remapped to a topic nobody
publishes, and its `depth_cleaned`/`debug_image` outputs remapped to private
test topics so it could not collide with the live one:
```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    nohup ros2 run poc_fusion depth_preprocess_node --ros-args \
      -r __node:=depth_preprocess_watchdog_test \
      -r /ascamera/camera_publisher/depth0/camera_info:=/poc_fusion/test_camera_info_never_published \
      -r /poc_fusion/depth_cleaned:=/poc_fusion/test_depth_cleaned \
      -r /poc_fusion/debug_image:=/poc_fusion/test_debug_image \
      > /tmp/watchdog_test.log 2>&1 &
    disown
    sleep 8
    cat /tmp/watchdog_test.log
  '
[INFO] [1786119283.046358618] [depth_preprocess_watchdog_test]: depth_preprocess_node started: /ascamera/camera_publisher/depth0/image_raw -> /poc_fusion/depth_cleaned (expected encoding=mono16, roi_profile=default, median_kernel_size=3)
[INFO] [1786119284.712780432] [depth_preprocess_watchdog_test]: invalid pixel fraction: 0.287
[WARN] [1786119288.012239417] [depth_preprocess_watchdog_test]: No message received on /ascamera/camera_publisher/depth0/camera_info within 5s of startup. depth_image_proc::PointCloudXyzNode requires camera_info to produce points -- without it, the fused costmap will silently run LiDAR-only with no other error.
```
Warning fired at `1786119288.012 − 1786119283.046 = 4.966s` after startup —
matches the 5s bar (the small undershoot is the timer's first tick landing
slightly before the 5.000s mark, expected `create_timer` jitter, not a bug).
Test instance torn down immediately after (see Teardown section below); its
depth image subscription (to the real, unremapped
`/ascamera/camera_publisher/depth0/image_raw`) was read-only and never
published anything the live stack consumes.

### Step 5 substitutes (machine-checkable, per Decision 4 — RViz is out of scope)

**Frame ID:**
```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    timeout 3 ros2 topic echo /poc_fusion/points --no-arr --once
  '
header:
  stamp: {sec: 1786118984, nanosec: 59000000}
  frame_id: depth_camera_link
height: 400
width: 640
fields: '<sequence type: sensor_msgs/msg/PointField, length: 3>'
is_bigendian: false
point_step: 16
row_step: 10240
data: '<sequence type: uint8, length: 4096000>'
is_dense: false
---
```
`frame_id: depth_camera_link` — the correct optical frame per Task 1's
finding, not `depth_cam_link`.

**Rate — measured, and the surprising number chased to its cause rather
than explained away:**
```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    timeout 10 ros2 topic hz /poc_fusion/points
  '
average rate: 3.622  ... window: 5
average rate: 3.534  ... window: 9
average rate: 2.622  ... window: 10
average rate: 3.316  ... window: 16
average rate: 3.514  ... window: 21
average rate: 4.718  ... window: 33
average rate: 4.462  ... window: 37
```
This is well below the source's ~14.7 Hz and below `/poc_fusion/depth_cleaned`'s
own ~14.7 Hz. Rather than assert a cause, three further measurements
isolated it:

1. System load at the time:
```
$ docker exec -u ubuntu MentorPi bash -lc 'top -bn1 | head -12'
top - 16:09:30 up 9 min, load average: 6.01, 4.82, 2.72
%Cpu(s): 56.1 us, 9.1 sy, 0.0 ni, 33.3 id, ...
    PID USER  %CPU COMMAND
   2029 ubuntu 93.8 joystick_control   <- pre-existing vendor process, not started by this task
   1980 ubuntu 50.0 aurora9+           <- pre-existing camera driver
  23377 ubuntu 43.8 depth_p+           <- our node
  23379 ubuntu 12.5 compone+           <- our container (rectify + point_cloud_xyz)
```
`nproc` confirms 4 cores; `load average: 6.01` on 4 cores means the CPU is
oversubscribed even before our pipeline runs — `joystick_control` alone
(a vendor node present in the baseline node list before this task started
anything) was already consuming ~94% of a core throughout this session.

2. To isolate whether the slowdown originates upstream (image
   preprocessing/rectify) or in the point-cloud projection itself, all
   three stage topics were measured **simultaneously** in one command:
```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    timeout 10 ros2 topic hz /poc_fusion/depth_cleaned &
    timeout 10 ros2 topic hz /poc_fusion/depth_rect &
    timeout 10 ros2 topic hz /poc_fusion/points &
    wait
  '
[/poc_fusion/depth_cleaned, final window] average rate: 14.533  window: 121
[/poc_fusion/depth_rect,    final window] average rate: 14.492  window: 75
[/poc_fusion/points,        final window] average rate: 3.271   window: 19
```
`depth_cleaned` (source-rate passthrough + median filter) and `depth_rect`
(rectify) both sustain ~14.5 Hz — essentially the full source rate. Only
`points`, downstream of `depth_image_proc::PointCloudXyzNode`, drops to
~3 Hz.

**Correction (fix round, see "Task 5 review fix round" subsection below):**
the original text here asserted this "isolates the bottleneck to that
node's per-pixel XYZ unprojection ... not a wiring defect." That claim
overreached the evidence above — (1) alone shows CPU contention exists but
not that it specifically bottlenecks `PointCloudXyzNode` rather than, say,
single-threaded executor serialization inside the shared container, or a
message-synchronizer drop between `image_rect` and `camera_info`. Both of
those alternative mechanisms were tested directly in the fix round below.
Neither one explains the sustained low rate either. **The true cause of
`PointCloudXyzNode`'s low throughput is not isolated by any measurement
taken in this task.** What is confirmed: the wiring/remappings are correct
(via `ros2 node info`, independent of rate), and the low rate is not a
partial-failure/crash-loop symptom (point 3 below still holds).

3. Point count and finite fraction tracked against Task 4's own
   concurrently-logged invalid-pixel fraction (see next subsection) confirm
   the low rate is a throughput/CPU-time property, not a partial-failure
   (e.g. crashing/restarting) property — the node produces full, valid
   messages, just fewer of them per second than the source.

**Recorded as-is, not gated:** the brief's Decision 4 asks for "a sane
rate," not a specific number. ~3–7 Hz (it varied across trials with
concurrent system load) is far above zero and every message was complete
and valid (below). The specific mechanism behind the rate is **not
isolated** (see correction and fix round below) — only that the pipeline
is CPU-contended in general and produces complete, valid messages at a
reduced rate. Task 12 (latency) and Task 16 (end-to-end) are better placed
to decide whether this rate is acceptable for the stop-trigger's timing
budget — flagging it forward rather than declaring it a pass or fail here.

**Point count and geometry — non-zero, finite, and physically plausible:**
An ad hoc in-container script (`/tmp/points_check.py`, not committed —
scratch verification tooling only) subscribed to `/poc_fusion/points` with
`qos_profile_sensor_data` (the default `ros2 topic hz`/reliable QoS is
incompatible with this publisher's best-effort QoS — first attempt failed
with an explicit `RELIABILITY` incompatibility WARN, corrected by using the
sensor-data QoS profile) and reported per-message stats:
```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    timeout 15 python3 /tmp/points_check.py
  '
msg 0: stamp=(1786119233,501000000) w=640 h=400 total_rows=256000 finite=180315 finite_frac=0.7044 x_range=(-0.32291722297668457, 0.2615756094455719) y_range=(-0.35424816608428955, 0.09392822533845901) z_range=(0.22300000488758087, 0.8230000138282776)
msg 1: stamp=(1786119234,526000000) w=640 h=400 total_rows=256000 finite=180461 finite_frac=0.7049 x_range=(-0.32221826910972595, 0.2615756094455719) y_range=(-0.35123515129089355, 0.09392822533845901) z_range=(0.22300000488758087, 0.8160000443458557)
msg 2: stamp=(1786119234,622000000) w=640 h=400 total_rows=256000 finite=180443 finite_frac=0.7049 x_range=(-0.32641199231147766, 0.2615756094455719) y_range=(-0.35166558623313904, 0.09392822533845901) z_range=(0.22300000488758087, 0.8170000314712524)
msg 3: stamp=(1786119235,8000000) w=640 h=400 total_rows=256000 finite=180300 finite_frac=0.7043 x_range=(-0.32113081216812134, 0.2615756094455719) y_range=(-0.36156558990478516, 0.09392822533845901) z_range=(0.22300000488758087, 0.8400000333786011)
msg 4: stamp=(1786119235,76000000) w=640 h=400 total_rows=256000 finite=180138 finite_frac=0.7037 x_range=(-0.32291722297668457, 0.2608780860900879) y_range=(-0.3581221103668213, 0.09392822533845901) z_range=(0.22300000488758087, 0.8320000171661377)
```
- All 5 messages have 180,138–180,461 finite points out of 256,000 rows
  (`is_dense: false`, invalid pixels are `NaN`, not omitted rows) — always
  non-zero, well above 0.
- All returned `(x, y, z)` values checked finite via `np.isfinite` (no
  inf/NaN leaking through as a "finite" point).
- **z** (optical-frame forward/depth axis) ranges 0.223–0.840 m across the
  5 messages. Expected band: a camera 0.246 m above the floor tilted ~40°
  below horizontal, per Task 0/Task 1's measured pose, has a floor-return
  range around `0.246 / sin(40°) ≈ 0.38 m` at the image's vertical centre,
  extending out to the previously-measured coverage ceiling of ~0.94 m for
  the furthest visible surfaces. 0.223–0.840 m falls inside that band (the
  0.223 m minimum matches Task 4's own `units_check.py` raw-pixel finding
  of 223 mm minimum valid depth, now expressed in metres after
  `PointCloudXyzNode`'s mm→m conversion — consistent across both
  verifications).
- **x** (right) ranges roughly −0.33 to +0.26 m, **y** (down) ranges
  roughly −0.36 to +0.09 m — both are the kind of narrow, off-axis spread
  expected from a camera with a modest FOV at these ranges, and neither
  blows past what geometry allows (e.g. no y value anywhere near −2 m,
  which would indicate a broken projection).

**Point count tracks the valid-pixel fraction:** at the same wall-clock
window, `depth_preprocess_node`'s own throttled log showed:
```
[depth_preprocess_node-1] [INFO] [1786119231.815989755] [depth_preprocess_node]: invalid pixel fraction: 0.285
[depth_preprocess_node-1] [INFO] [1786119236.875398347] [depth_preprocess_node]: invalid pixel fraction: 0.286
```
i.e. valid fraction ≈ 1 − 0.285…0.286 = **0.714–0.715**. The point cloud's
own finite fraction across the 5 sampled messages was **0.7037–0.7049** —
within about 1 percentage point, tracking as expected (not identical,
because the ROI/median-filter pass and the rectify/reprojection step each
touch a handful of additional border pixels, but the two numbers move
together, not independently).

### Step 3: rectification distortion figures (for the record, not re-derived)

Per the brief: worst-case distortion is **0.64 cm at 1 m** at the
horizontal edge (2.71 px), against a **5 cm** costmap cell — roughly ⅛ of a
cell, far below quantization. This was never large enough to invalidate an
edge result on its own; rectifying (this task) removes the *argument* that
an edge miss might be attributable to lens distortion rather than
structured-light falloff, which is the expected dominant effect. Not
re-derived here, per the brief's explicit instruction.

### Self-arm ROI check: vacuous, not a pass

The brief's Step 5 asks to confirm "the self-arm region is absent" from the
cloud. Task 4's `roi_profiles.default` ships with `row_min == row_max` and
`col_min == col_max` (zero-width box) — a deliberate "start permissive"
placeholder that masks nothing, per Task 4's own documentation; Task 11
measures and installs the real arm-occlusion box later. So this check is
**currently vacuous**: there is no ROI mask active for the cloud to
exclude, and "the self-arm region is absent" cannot be meaningfully
evaluated until Task 11 supplies real bounds. Recording this explicitly
rather than claiming the check passed.

### Deferred to the human-present phase

Per Decision 4, the following are explicitly **not done** in this
unattended session and are deferred:
- Visual RViz confirmation that the cloud appears in `depth_camera_link`
  and its geometry matches the physical scene.
- Before/after visual comparison of speckle at depth edges with linear vs.
  nearest-neighbour interpolation (the parameter-readback check above
  confirms the *setting*, not the *visual artifact* it prevents).
- A screenshot reference for Step 5.
No RViz session was attempted (no display, no human available), consistent
with CONSTRAINTS.md.

### Teardown

```
$ docker exec -u ubuntu MentorPi bash -lc 'ps aux | grep -E "poc_fusion|depth_preprocess_node|ros2 launch poc_fusion" | grep -v grep'
ubuntu     23364  ...  /usr/bin/python3 /opt/ros/humble/bin/ros2 launch poc_fusion poc_fusion.launch.py
ubuntu     23377  ...  /usr/bin/python3 .../depth_preprocess_node --ros-args -r __node:=depth_preprocess_node ...
ubuntu     23379  ...  /opt/ros/humble/lib/rclcpp_components/component_container --ros-args -r __node:=poc_fusion_container -r __ns:=/poc_fusion

$ docker exec -u ubuntu MentorPi bash -lc 'kill -TERM 23364 23377 23379; sleep 3; ps aux | grep -E "poc_fusion|depth_preprocess_node" | grep -v grep || echo "all our processes gone"'
all our processes gone
```
(An initial `kill -INT` on the launch PID alone did not bring the children
down within ~9s of waiting, so all three PIDs we started — the `ros2
launch` process and its two children — were killed directly by PID. The
earlier isolated watchdog-test instance, a separate PID not part of this
set, had already been stopped via `pkill -f depth_preprocess_watchdog_test`
immediately after its own check, and its exit code 143 is `pkill`'s own
normal SIGTERM-delivered exit, not a failure.)

Node-list diff, before vs. after (DDS discovery briefly still showed our
nodes ~immediately after the kill — expected propagation delay, resolved
after a 5s wait before re-querying):
```
$ diff <(sort nodelist_after_teardown.txt) <(sort nodelist_before_teardown.txt)
4a5
> /depth_preprocess_node
14a16
> /launch_ros_23364
19a22,24
> /poc_fusion/depth_rectify_node
> /poc_fusion/poc_fusion_container
> /poc_fusion/point_cloud_xyz_node
```
Only the four nodes this task's own launch added are the difference; every
pre-existing node (`/LD19`, `/aurora/aurora`, `/joystick_control`, arm and
gripper controllers, etc.) is present in both listings, confirming the
pre-existing stack was undisturbed.

### Host test suite — final

```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/ -q
.....................................                                    [100%]
37 passed in 0.23s
```

### Task 5 review fix round (2026-08-08)

Timestamps: HKT (UTC+8, host) / UTC (container, ROS `top`/log clocks).

A code reviewer flagged that the "Rate" subsection above overreached: it
asserted the low `/poc_fusion/points` rate "isolates cleanly" to
`PointCloudXyzNode`'s per-pixel projection cost, but the pasted
`component_container` CPU figure (12.5%) is inconsistent with that node
being compute-bound. Two untested alternative mechanisms were named:
single-threaded executor serialization inside the shared container, and a
message-synchronizer drop between `image_rect` and `camera_info`. Both were
tested directly below, with the explicit instruction that an honest "not
isolated" outcome is acceptable and preferred over asserting a second,
still-unproven mechanism.

**Check 1 — swap `component_container` → `component_container_mt`, re-measure
simultaneously, same command shape as the original measurement.**

Before (single-threaded `component_container`, HKT 2026-08-08 ~15:52 /
container UTC ~07:52):
```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    timeout 10 ros2 topic hz /poc_fusion/depth_cleaned &
    timeout 10 ros2 topic hz /poc_fusion/depth_rect &
    timeout 10 ros2 topic hz /poc_fusion/points &
    wait
  '
[/poc_fusion/depth_cleaned, final window] average rate: 14.705  min: 0.058s max: 0.077s std dev: 0.00519s window: 121
[/poc_fusion/depth_rect,    final window] average rate: 14.528  min: 0.058s max: 0.081s std dev: 0.00612s window: 75
[/poc_fusion/points,        final window] average rate: 3.990   min: 0.132s max: 1.113s std dev: 0.28901s window: 19
```
```
$ docker exec -u ubuntu MentorPi bash -lc 'top -bn1 | head -12'
top - 07:52:xx up ... , load average: 5.33, 4.20, 3.85
    PID USER  %CPU COMMAND
        ubuntu 100.0 joystick_control    <- pre-existing vendor process
 100680 ubuntu  11.7 component_container <- our container (rectify + point_cloud_xyz)
```
```
$ docker exec -u ubuntu MentorPi bash -lc \
  'grep -c "do not appear to be synchronized" /tmp/poc_fusion_before.log'
1
```
(That one occurrence was `point_cloud_xyz_node`'s own synchronizer warning
at startup: "Image messages received: 0, CameraInfo messages received: 13,
Synchronized pairs: 0" — a startup transient before the first depth frame
arrived, not a sustained drop.)

After (`component_container_mt`, HKT ~16:05 / UTC ~08:05):
```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    timeout 10 ros2 topic hz /poc_fusion/depth_cleaned &
    timeout 10 ros2 topic hz /poc_fusion/depth_rect &
    timeout 10 ros2 topic hz /poc_fusion/points &
    wait
  '
[/poc_fusion/depth_cleaned, final window] average rate: 14.014  min: 0.059s max: 0.084s std dev: 0.00701s window: 116
[/poc_fusion/depth_rect,    final window] average rate: 13.855  min: 0.059s max: 0.089s std dev: 0.00788s window: 72
[/poc_fusion/points,        final window] average rate: 4.272   min: 0.135s max: 1.784s std dev: 0.34215s window: 20
```
```
$ docker exec -u ubuntu MentorPi bash -lc 'top -bn1 | head -12'
top - 08:05:xx up ... , load average: 5.6x, 4.5x, 3.9x
    PID USER  %CPU COMMAND
        ubuntu 100.0 joystick_control
  <pid> ubuntu  11.7 component_container_mt
```
```
$ docker exec -u ubuntu MentorPi bash -lc \
  'grep -c "do not appear to be synchronized" /tmp/poc_fusion_after.log'
4
```
(4 occurrences, clustered as 2 pairs ~1s apart, both still in the startup
window before steady-state locked in — the same transient pattern as the
"before" run, not a new or sustained behaviour.)

**Result:** `_mt` gave no material rate recovery (3.990 → 4.272 Hz, a ~7%
change consistent with run-to-run noise given the concurrent
`joystick_control` load), CPU utilization on the container process was
identical (11.7% in both runs), and max latency actually got *worse*
(1.113s → 1.784s). This does not support keeping `_mt`. Per the decision
tree, **the launch file was reverted to `component_container`** (see
`poc_fusion/launch/poc_fusion.launch.py`, `executable='component_container'`
— unchanged from the originally committed value).

**Check 2 — grep `point_cloud_xyz_node`'s own log for synchronizer warnings
(not just the depth_preprocess_node watchdog's text).** Done as part of
Check 1 above: 1 occurrence before, 4 after, both confined to the
startup transient window (first few seconds before the synchronizer locks
onto a steady stream), not a sustained pattern in either configuration.
This rules out a persistent sync-drop as the explanation for the sustained
~3–7 Hz rate.

**Conclusion — corrected, not re-asserted with a new story:** neither
alternative mechanism (single-threaded serialization; sync-drop) explains
the sustained low rate. Combined with Check 1's CPU figures (11.7%,
essentially unchanged either way — not saturated, not compute-bound in any
way the measurements here can show), the honest conclusion is that
**the mechanism behind `PointCloudXyzNode`'s low throughput is not
isolated.** What is ruled out: (1) a wiring/remapping defect (confirmed
correct independently via `ros2 node info`); (2) `image_proc::RectifyNode`
as the bottleneck (it and `depth_cleaned` both sustain ~14–14.7 Hz); (3)
single-threaded executor serialization as the sole or primary cause (no
material change under `_mt`); (4) a persistent message-synchronizer drop
(warnings are brief startup transients only, in both configurations). What
remains unknown: the specific mechanism inside `PointCloudXyzNode` (or its
interaction with system-wide CPU contention) that yields ~3–7 Hz. This is
recorded honestly as an open question, flagged forward to Task 12/16 as
before, rather than closed with an unproven explanation.

Launch file: reverted to the original committed state (`executable=
'component_container'`), confirmed via `git diff` showing no changes to
`poc_fusion/launch/poc_fusion.launch.py` after the revert. Redeployed and
did a final sanity launch confirming `/poc_fusion/points` and the other
topics are present and publishing after the revert, then torn down the
same way as the main run (`kill -TERM` on all three PIDs, `ros2 node list`
diff confirming only this task's own nodes were the difference).

Host test suite re-run after the fix round, immediately before committing:
```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/ -q
.....................................                                    [100%]
37 passed in 0.24s
```

Two Minors raised by the same review (duplicated `camera_info` topic
string literal across `depth_preprocess_node.py` and
`poc_fusion.launch.py`; `_camera_info_watchdog_timer.cancel()` without a
matching `.destroy()`) were explicitly **not touched** this round, per
direct instruction to leave them for final review.

### Task 5 summary

Implemented `poc_fusion/poc_fusion/lib/camera_info_watchdog.py` (TDD,
RED/GREEN, 4 new tests, 37/37 total), wired it into
`depth_preprocess_node.py` as a self-cancelling one-shot timer, and wrote
`poc_fusion/launch/poc_fusion.launch.py`: `depth_preprocess_node` (plain
`Node`) plus a `poc_fusion_container` (`rclcpp_components`) holding
`image_proc::RectifyNode` (nearest-neighbour, `interpolation: 0`, confirmed
by live `ros2 param get`) and `depth_image_proc::PointCloudXyzNode`,
remapped exactly per the brief. `package.xml` gained the four exec_depends
the launch file itself needs (`launch`, `launch_ros`, `rclcpp_components`,
`ament_index_python`); `costmap_params.yaml` untouched.

Deployed, built, and launched on-robot as `ubuntu`. Verified live:
remappings correct (`ros2 node info`), interpolation is nearest-neighbour
(`ros2 param get`), `/poc_fusion/points` publishes at `depth_camera_link`
with non-zero finite point counts (180k+/256k per message) whose finite
fraction (0.704–0.705) tracks Task 4's independently-logged valid-pixel
fraction (0.714–0.715) to within ~1 point, and whose x/y/z ranges are
consistent with the camera's known 0.246 m/40°-below-horizontal pose. The
camera_info watchdog was verified in both directions live: no false warning
while camera_info is live, and a genuine warning at ~4.97s (against the 5s
bar) when an isolated test instance had its camera_info subscription
remapped to a silent topic.

One real finding, chased to ground rather than asserted: `/poc_fusion/points`
publishes at only ~3–7 Hz, well below the ~14.5 Hz sustained by both
upstream stages (`depth_cleaned`, `depth_rect`, both measured
simultaneously with `points` in the same command). A follow-up review
round (see "Task 5 review fix round" above) tested two specific
alternative mechanisms — single-threaded executor serialization (swapped
to `component_container_mt`, no material rate recovery: 3.99→4.27 Hz,
identical 11.7% CPU, worse max latency) and a persistent
message-synchronizer drop (grepped `point_cloud_xyz_node`'s own log: only
brief startup-transient warnings in either configuration, not sustained) —
and ruled out both, along with a wiring/remapping defect (independently
confirmed correct via `ros2 node info`). **The specific mechanism behind
the low rate is not isolated by any measurement taken in this task.**
What is confirmed: it is not a wiring defect, not a rectify bottleneck, not
purely single-threaded serialization, and not a sync-drop; system-wide CPU
contention exists (load average ~5–6 on 4 cores, `joystick_control` alone
at ~94–100% of a core) but was not shown to specifically saturate
`PointCloudXyzNode`. Flagged forward to Task 12/16 as an open question
rather than gated here, since the brief only requires "a sane rate," not a
specific number.

The self-arm-ROI absence check is recorded as currently vacuous (Task 4's
ROI is a zero-width placeholder; there is nothing yet for the cloud to
exclude) rather than claimed as a pass. RViz visual confirmation,
speckle-at-edges before/after comparison, and the Step 5 screenshot are
explicitly deferred to a human-present session — none was attempted.
Step 3's distortion figures (0.64 cm at 1 m / 2.71 px worst case, ⅛ of the
5 cm costmap cell) are recorded per the brief, not re-derived.

Teardown confirmed only this task's own nodes (`depth_preprocess_node`,
`poc_fusion_container` and its two composable nodes, `launch_ros_23364`)
were added and removed; the full pre-existing stack (`/LD19`,
`/aurora/aurora`, `/joystick_control`, arm/gripper controllers, etc.) is
identical before and after by direct `diff`. No HARD SAFETY RULE was
touched — no `cmd_vel` publish, no arm command, `.stop_ros.sh` never run,
no edits to `proximity_alert/` or to files under `/home/ubuntu/ros2_ws/src/`
outside the deploy script's target.
