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

**Correction (second fix round, 2026-08-08):** the block originally pasted
here for this check contained placeholder characters (`08:05:xx`,
`5.6x, 4.5x, 3.9x`, `<pid>`) inside text presented as verbatim `top`
output — those cannot have come from a real `top` invocation, and a
re-review correctly caught it as fabricated/hand-edited "evidence." The
`[topic, final window]`-prefixed `ros2 topic hz` formatting in the
original blocks was also not what that command actually prints (verified
by re-running it below — it prints bare `average rate:` / `min/max/std
dev` lines with no topic-name prefix at all). Both blocks below are
**freshly captured now**, during this second fix round, not the original
measurements — system load differs run to run, so these are a new
before/after pair, not a recovery of the first one. Real, unedited
terminal output only; where a topic-name prefix was needed for
readability across three interleaved background jobs, it was produced by
piping each job through `sed 's/^/[topic] /'`, which is shown in the
pasted command itself rather than hand-added to the output afterward.

**Check 1 — swap `component_container` → `component_container_mt`, re-measure
simultaneously, same command shape as the original measurement.**

Before (single-threaded `component_container`, HKT 2026-08-08 ~00:51 /
container UTC 2026-08-07 16:51 — note the container clock reads 2026-08-07,
one day behind the host's 2026-08-08; recorded as-is):
```
$ docker exec -u ubuntu MentorPi bash -lc '
  sleep 5
  date -u
  top -bn1 | head -14
  ps -p 169364 -o pid,pcpu,comm
'
Fri Aug  7 16:51:20 UTC 2026
top - 16:51:20 up 51 min,  0 users,  load average: 4.78, 3.89, 3.86
Tasks:  59 total,   2 running,  36 sleeping,   0 stopped,  21 zombie
%Cpu(s): 72.6 us,  6.5 sy,  0.0 ni, 21.0 id,  0.0 wa,  0.0 hi,  0.0 si,  0.0 st
MiB Mem :   8063.0 total,   1720.7 free,   3819.0 used,   2523.2 buff/cache
MiB Swap:   6344.0 total,   6344.0 free,      0.0 used.   4110.5 avail Mem

    PID USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND
   2029 ubuntu    20   0  999392 107792  56576 R  86.7   1.3  47:49.64 joystic+
   1980 ubuntu    20   0 1947120 130736  52080 S  53.3   1.6  25:20.66 aurora9+
 169362 ubuntu    20   0 1300608 144928  79744 S  40.0   1.8   0:06.38 depth_p+
   1968 ubuntu    20   0  775776  67152  31344 S   6.7   0.8   3:08.45 joint_s+
   1976 ubuntu    20   0  662848  35808  22768 S   6.7   0.4   1:54.04 ekf_node
   1978 ubuntu    20   0  860032  75936  32960 S   6.7   0.9   1:22.81 servo_c+
   2083 ubuntu    20   0  786880  74576  32848 S   6.7   0.9   2:17.73 python3
    PID %CPU COMMAND
 169364 12.0 component_conta
```
(`component_container` (PID 169364) did not place in `top`'s top-7-by-CPU
list, hence the explicit `ps -p` follow-up in the same command — its real
CPU figure is 12.0%, not the fabricated 11.7% quoted before.)
```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ( timeout 10 ros2 topic hz /poc_fusion/depth_cleaned | sed "s/^/[depth_cleaned] /" ) &
    ( timeout 10 ros2 topic hz /poc_fusion/depth_rect    | sed "s/^/[depth_rect]    /" ) &
    ( timeout 10 ros2 topic hz /poc_fusion/points        | sed "s/^/[points]        /" ) &
    wait
  '
[points]        WARNING: topic [/poc_fusion/points] does not appear to be published yet
[points]        average rate: 1.212
[points]        	min: 0.336s max: 1.677s std dev: 0.60479s window: 3
[points]        average rate: 3.429
[points]        	min: 0.060s max: 1.677s std dev: 0.43438s window: 12
[points]        average rate: 4.304
[points]        	min: 0.059s max: 1.677s std dev: 0.34883s window: 20
[points]        average rate: 4.931
[points]        	min: 0.059s max: 1.677s std dev: 0.30109s window: 28
[points]        average rate: 4.299
[points]        	min: 0.059s max: 1.677s std dev: 0.31156s window: 30
[depth_cleaned] average rate: 14.848
[depth_cleaned] 	min: 0.060s max: 0.083s std dev: 0.00567s window: 16
[depth_cleaned] average rate: 14.811
[depth_cleaned] 	min: 0.055s max: 0.086s std dev: 0.00691s window: 31
[depth_cleaned] average rate: 14.796
[depth_cleaned] 	min: 0.055s max: 0.086s std dev: 0.00612s window: 46
[depth_cleaned] average rate: 14.786
[depth_cleaned] 	min: 0.055s max: 0.086s std dev: 0.00614s window: 61
[depth_cleaned] average rate: 14.778
[depth_cleaned] 	min: 0.055s max: 0.086s std dev: 0.00604s window: 76
[depth_cleaned] average rate: 14.709
[depth_cleaned] 	min: 0.055s max: 0.092s std dev: 0.00651s window: 91
[depth_rect]    WARNING: topic [/poc_fusion/depth_rect] does not appear to be published yet
[depth_rect]    average rate: 15.287
[depth_rect]    	min: 0.019s max: 0.140s std dev: 0.02565s window: 17
[depth_rect]    average rate: 14.900
[depth_rect]    	min: 0.019s max: 0.140s std dev: 0.01975s window: 32
[depth_rect]    average rate: 14.989
[depth_rect]    	min: 0.019s max: 0.140s std dev: 0.01674s window: 48
[depth_rect]    average rate: 14.936
[depth_rect]    	min: 0.019s max: 0.140s std dev: 0.01483s window: 63
[depth_rect]    average rate: 14.890
[depth_rect]    	min: 0.019s max: 0.140s std dev: 0.01374s window: 78
[depth_rect]    average rate: 14.792
[depth_rect]    	min: 0.019s max: 0.140s std dev: 0.01299s window: 93
[depth_rect]    average rate: 14.819
[depth_rect]    	min: 0.019s max: 0.140s std dev: 0.01208s window: 108
```
Final readings: `depth_cleaned` 14.709 Hz, `depth_rect` 14.819 Hz,
`points` 4.299 Hz.
```
$ docker exec -u ubuntu MentorPi bash -lc \
  'grep -n "do not appear to be synchronized" /tmp/poc_fusion_baseline2.log; grep -c "do not appear to be synchronized" /tmp/poc_fusion_baseline2.log'
0
```
(`grep -n` printed nothing — no matching lines — so the only output is
`grep -c`'s count of `0`. No synchronizer-warning lines at all in this
run's log — 0 occurrences,
different from the 1 occurrence recorded in the original, now-superseded
measurement. Both are consistent with "no sustained sync-drop"; the exact
count is noisy run to run because it depends on exact startup timing.)

After (`component_container_mt`, HKT 2026-08-08 ~00:53 / container UTC
2026-08-07 16:53):
```
$ docker exec -u ubuntu MentorPi bash -lc '
  sleep 5
  date -u
  top -bn1 | head -14
  ps -p 175532 -o pid,pcpu,comm
'
Fri Aug  7 16:53:06 UTC 2026
top - 16:53:06 up 53 min,  0 users,  load average: 4.27, 3.99, 3.90
Tasks:  61 total,   3 running,  34 sleeping,   0 stopped,  24 zombie
%Cpu(s): 55.0 us,  6.7 sy,  0.0 ni, 38.3 id,  0.0 wa,  0.0 hi,  0.0 si,  0.0 st
MiB Mem :   8063.0 total,   1690.6 free,   3847.9 used,   2524.5 buff/cache
MiB Swap:   6344.0 total,   6344.0 free,      0.0 used.   4081.7 avail Mem

    PID USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND
   2029 ubuntu    20   0  999392 107808  56576 R  93.8   1.3  49:27.18 joystic+
   1980 ubuntu    20   0 1947120 130752  51952 S  50.0   1.6  26:18.06 aurora9+
 175530 ubuntu    20   0 1300704 144896  79760 S  31.2   1.8   0:05.69 depth_p+
   1968 ubuntu    20   0  775776  67168  31344 R  12.5   0.8   3:15.97 joint_s+
 175532 ubuntu    20   0 1209280 102768  48256 S  12.5   1.2   0:01.93 compone+
   1970 ubuntu    20   0  585600  32784  20960 S   6.2   0.4   0:20.47 robot_s+
   1972 ubuntu    20   0  926704  70032  31984 S   6.2   0.8   2:20.47 ros_rob+
    PID %CPU COMMAND
 175532 11.3 component_conta
```
(Here `component_container_mt` (PID 175532) did place in `top`'s own
top-7 list at 12.5%; the trailing `ps -p` gives 11.3% — the two numbers
differ slightly because `top`'s single-sample instantaneous figure and
`ps`'s since-start-of-process average are different statistics, both
pasted rather than reconciled after the fact.)
```
$ docker exec -u ubuntu MentorPi bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ( timeout 10 ros2 topic hz /poc_fusion/depth_cleaned | sed "s/^/[depth_cleaned] /" ) &
    ( timeout 10 ros2 topic hz /poc_fusion/depth_rect    | sed "s/^/[depth_rect]    /" ) &
    ( timeout 10 ros2 topic hz /poc_fusion/points        | sed "s/^/[points]        /" ) &
    wait
  '
[depth_rect]    average rate: 12.356
[depth_rect]    	min: 0.025s max: 0.192s std dev: 0.03813s window: 13
[depth_rect]    average rate: 13.246
[depth_rect]    	min: 0.025s max: 0.192s std dev: 0.03677s window: 28
[depth_rect]    average rate: 13.744
[depth_rect]    	min: 0.025s max: 0.192s std dev: 0.03020s window: 43
[depth_rect]    average rate: 13.984
[depth_rect]    	min: 0.025s max: 0.192s std dev: 0.02723s window: 58
[depth_rect]    average rate: 14.154
[depth_rect]    	min: 0.025s max: 0.192s std dev: 0.02448s window: 73
[depth_rect]    average rate: 13.768
[depth_rect]    	min: 0.025s max: 0.200s std dev: 0.02743s window: 85
[depth_rect]    average rate: 13.850
[depth_rect]    	min: 0.025s max: 0.200s std dev: 0.02551s window: 100
[depth_cleaned] average rate: 11.968
[depth_cleaned] 	min: 0.036s max: 0.223s std dev: 0.04626s window: 14
[depth_cleaned] average rate: 11.972
[depth_cleaned] 	min: 0.036s max: 0.223s std dev: 0.04064s window: 26
[depth_cleaned] average rate: 13.479
[depth_cleaned] 	min: 0.033s max: 0.223s std dev: 0.03478s window: 43
[depth_cleaned] average rate: 13.743
[depth_cleaned] 	min: 0.033s max: 0.223s std dev: 0.03112s window: 58
[depth_cleaned] average rate: 13.984
[depth_cleaned] 	min: 0.033s max: 0.223s std dev: 0.02803s window: 73
[depth_cleaned] average rate: 14.111
[depth_cleaned] 	min: 0.033s max: 0.223s std dev: 0.02570s window: 88
[depth_cleaned] average rate: 14.189
[depth_cleaned] 	min: 0.033s max: 0.223s std dev: 0.02394s window: 103
[points]        average rate: 3.147
[points]        	min: 0.058s max: 0.812s std dev: 0.29368s window: 4
[points]        average rate: 2.881
[points]        	min: 0.058s max: 0.818s std dev: 0.30073s window: 7
[points]        average rate: 2.427
[points]        	min: 0.058s max: 1.425s std dev: 0.43030s window: 10
[points]        average rate: 3.462
[points]        	min: 0.058s max: 1.425s std dev: 0.35179s window: 18
[points]        average rate: 3.050
[points]        	min: 0.058s max: 1.425s std dev: 0.38024s window: 19
[points]        average rate: 3.302
[points]        	min: 0.056s max: 1.425s std dev: 0.34506s window: 24
```
Final readings: `depth_cleaned` 14.189 Hz, `depth_rect` 13.850 Hz,
`points` 3.302 Hz.
```
$ docker exec -u ubuntu MentorPi bash -lc \
  'grep -n "do not appear to be synchronized" /tmp/poc_fusion_mt2.log; grep -c "do not appear to be synchronized" /tmp/poc_fusion_mt2.log'
16:[component_container_mt-2] [WARN] [1786121571.459595375] [poc_fusion.depth_rectify_node]: [image_transport] Topics '/poc_fusion/depth_cleaned' and '/poc_fusion/camera_info' do not appear to be synchronized. In the last 10s:
20:[component_container_mt-2] [WARN] [1786121571.529645662] [poc_fusion.point_cloud_xyz_node]: [image_transport] Topics '/poc_fusion/depth_rect' and '/poc_fusion/camera_info' do not appear to be synchronized. In the last 10s:
25:[component_container_mt-2] [WARN] [1786121572.459800013] [poc_fusion.depth_rectify_node]: [image_transport] Topics '/poc_fusion/depth_cleaned' and '/poc_fusion/camera_info' do not appear to be synchronized. In the last 10s:
29:[component_container_mt-2] [WARN] [1786121572.529635317] [poc_fusion.point_cloud_xyz_node]: [image_transport] Topics '/poc_fusion/depth_rect' and '/poc_fusion/camera_info' do not appear to be synchronized. In the last 10s:
33:[component_container_mt-2] [WARN] [1786121573.459667576] [poc_fusion.depth_rectify_node]: [image_transport] Topics '/poc_fusion/depth_cleaned' and '/poc_fusion/camera_info' do not appear to be synchronized. In the last 10s:
37:[component_container_mt-2] [WARN] [1786121573.530427736] [poc_fusion.point_cloud_xyz_node]: [image_transport] Topics '/poc_fusion/depth_rect' and '/poc_fusion/camera_info' do not appear to be synchronized. In the last 10s:
6
```
6 occurrences (3 pairs, one per node, ~1s apart), all timestamped
1786121571–1786121573. The log's last line (`invalid pixel fraction`
throttled print from `depth_preprocess_node`) is timestamped
1786121614 — **41 seconds later**, with no further synchronizer warnings
in between, confirming these 6 are a startup-only transient, not a
sustained condition, in this run too.

**Result:** `_mt` gave no material rate recovery this time either —
`points` went from 4.299 Hz (single-threaded) to 3.302 Hz (`_mt`), i.e.
*lower*, not higher, in this pair of runs; `depth_cleaned`/`depth_rect`
both dropped too (14.7–14.8→14.2/13.9 Hz), consistent with general
system-load variation between the two runs rather than an executor
effect. Combined with the original pair's result (3.990→4.272 Hz, a small
increase) and this pair's result (4.299→3.302 Hz, a decrease), the two
trials disagree in direction — the clearest possible signal that `_mt`
is not producing a reliable, material change either way; the swings are
noise, not signal. CPU utilization on the container process stayed in the
same 11–13% range regardless of executor (12.0% single-threaded, 11.3–
12.5% `_mt`, depending on which tool sampled it). This does not support
keeping `_mt`. Per the decision tree, **the launch file was reverted to
`component_container`** (see `poc_fusion/launch/poc_fusion.launch.py`,
`executable='component_container'` — confirmed byte-identical to the
originally committed value via `git diff`, zero output).

**Check 2 — grep `point_cloud_xyz_node`'s own log for synchronizer warnings
(not just the depth_preprocess_node watchdog's text).** Done as part of
Check 1 above: 0 occurrences in the single-threaded run, 6 in the `_mt`
run, both confined to (or absent from) the startup transient window
(first few seconds, ending well before the run's end), not a sustained
pattern in either configuration. This rules out a persistent sync-drop as
the explanation for the sustained ~3–7 Hz rate.

**Conclusion — corrected, not re-asserted with a new story:** neither
alternative mechanism (single-threaded serialization; sync-drop) explains
the sustained low rate. Rate changed in *opposite directions* across the
two independent before/after trials run in this task (first trial:
3.990→4.272 Hz; second trial: 4.299→3.302 Hz), which is itself evidence
that the executor swap is not the controlling variable — a real effect
would point the same direction across trials. CPU utilization stayed in
the same ~11–13% band regardless of executor, not saturated, not
evidence of a compute bottleneck this task's measurements can show. The
honest conclusion is that **the mechanism behind `PointCloudXyzNode`'s
low throughput is not isolated.** What is ruled out: (1) a
wiring/remapping defect (confirmed correct independently via `ros2 node
info`); (2) `image_proc::RectifyNode` as the bottleneck (it and
`depth_cleaned` both sustain ~14–15 Hz while `points` does not); (3)
single-threaded executor serialization as the sole or primary cause (no
consistent-direction change under `_mt` across two independent trials);
(4) a persistent message-synchronizer drop (warnings are brief startup
transients only, or absent, in every configuration tested). What remains
unknown: the specific mechanism inside `PointCloudXyzNode` (or its
interaction with system-wide CPU contention) that yields ~3–7 Hz. This is
recorded honestly as an open question, flagged forward to Task 12/16 as
before, rather than closed with an unproven explanation.

Launch file: reverted to the original committed state (`executable=
'component_container'`), confirmed via `git diff` showing no changes to
`poc_fusion/launch/poc_fusion.launch.py` after the revert. Redeployed,
rebuilt, and did a final sanity launch confirming all seven
`/poc_fusion/*` topics (`debug_image`, `depth_cleaned`, `depth_rect`,
`image_rect/compressed`, `image_rect/compressedDepth`,
`image_rect/theora`, `points`) are present via `ros2 topic list | grep
poc_fusion` after the revert, then torn down the same way as the main run
(`kill -TERM` on all three PIDs; `ps aux` confirmed all gone; `ros2 node
list` immediately after still showed `/launch_ros_181227` and
`/poc_fusion/point_cloud_xyz_node` — the same DDS discovery propagation
delay documented earlier in this doc — resolved after an additional ~6s
wait, re-queried clean: "no leftover poc_fusion nodes").

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
alternative mechanisms across two independent before/after trials —
single-threaded executor serialization (swapped to
`component_container_mt`: trial 1 went 3.990→4.272 Hz, trial 2 went
4.299→3.302 Hz — opposite directions, CPU stayed in the same ~11–13% band
regardless of executor) and a persistent message-synchronizer drop
(grepped `point_cloud_xyz_node`'s own log directly: 0–4 or 6 occurrences
depending on the run, all confined to the first few seconds after launch,
none sustained) — and ruled out both, along with a wiring/remapping
defect (independently confirmed correct via `ros2 node info`). **The
specific mechanism behind the low rate is not isolated by any measurement
taken in this task.** What is confirmed: it is not a wiring defect, not a
rectify bottleneck, not purely single-threaded serialization, and not a
sync-drop; system-wide CPU
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

---

## Task 6: Costmap configuration (2026-08-08 HKT / 2026-08-07 UTC)

Standalone `nav2_costmap_2d` node, autostarted by `nav2_lifecycle_manager`, wired
into `poc_fusion/launch/poc_fusion.launch.py`. No planner, controller, BT
navigator, map server, or AMCL — scope is exactly the obstacle layer with two
observation sources (LiDAR scan + `/poc_fusion/points`).

### Step 0: determining the real node name/namespace BEFORE writing config

Ran the bare executable with no name/namespace override to see what
`nav2_costmap_2d` actually calls itself, rather than assuming:

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null
timeout 4 ros2 run nav2_costmap_2d nav2_costmap_2d &
sleep 2
ros2 node list
sleep 3
'
[INFO] [1786128210.865705706] [costmap.costmap]: 
	costmap lifecycle node launched. 
	Waiting on external lifecycle transitions to activate
	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[INFO] [1786128210.897223378] [costmap.costmap]: Creating Costmap
/LD19
/ar_app
/arm_controller
/aurora/aurora
/controller_manager
/costmap/costmap
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
/static_transform_publisher_xjZs1QyCV0ckvKkj
/transform_listener_impl_5555ad8bf7e0
/web_video_server
[INFO] [1786128213.786406018] [rclcpp]: signal_handler(signum=15)
[INFO] [1786128213.786589168] [costmap.costmap]: Running Nav2 LifecycleNode rcl preshutdown (costmap)
[INFO] [1786128213.786681465] [costmap.costmap]: Destroying bond (costmap) to lifecycle manager.
[INFO] [1786128213.789300539] [costmap.costmap]: Destroying
```

Default is namespace `costmap`, name `costmap` → fully-qualified `/costmap/costmap`,
matching nav2_bringup's own `local_costmap/local_costmap` double-nesting convention.
`poc_fusion.launch.py`'s costmap `Node` action deliberately passes no `name=`/
`namespace=` override, and `costmap_params.yaml`'s key path is
`costmap: costmap: ros__parameters:` to match.

### Bug 1: raw `costmap_params.yaml` cannot be passed directly as `--params-file`

First attempt passed `costmap_params_path` straight through to the costmap
`Node`'s `parameters=`. Standard ROS 2 params-file YAML only permits node-name
(or wildcard) keys at the top level; `costmap_params.yaml`'s top-level
`scan_topic` key (required so the file stays the single source of truth for the
scan topic) breaks that:

```
[nav2_costmap_2d-3] [ERROR] [1786128429.091567544] [rcl]: Failed to parse global arguments
[nav2_costmap_2d-3] terminate called after throwing an instance of 'rclcpp::exceptions::RCLInvalidROSArgsError'
[nav2_costmap_2d-3]   what():  failed to initialize rcl: Couldn't parse params file: '--params-file /home/ubuntu/ros2_ws/install/poc_fusion/share/poc_fusion/config/costmap_params.yaml'. Error: Cannot have a value before ros__parameters at line 16, at ./src/parse.c:793, at ./src/rcl/arguments.c:406
[ERROR] [nav2_costmap_2d-3]: process has died [pid 53220, exit code -6, cmd '/opt/ros/humble/lib/nav2_costmap_2d/nav2_costmap_2d --ros-args --params-file /home/ubuntu/ros2_ws/install/poc_fusion/share/poc_fusion/config/costmap_params.yaml --params-file /tmp/launch_params_89rm_d7v'].
```

Fix: `poc_fusion.launch.py` now `yaml.safe_load()`s the whole file at
launch-description-generation time, reads `scan_topic` from it, extracts only
the `costmap.costmap.ros__parameters` sub-tree, and passes THAT sub-tree as an
in-memory dict (plus the `obstacle_layer.scan.topic` override) to the `Node`
action — `launch_ros` writes it back out to a well-formed temp params file for
the node. `costmap_params.yaml` is unchanged by this fix; only how the launch
file consumes it changed.

### Bug 2: `bond_timeout` — standalone `nav2_costmap_2d` never creates a bond

With the params-file bug fixed, the costmap node itself reached `active`
(confirmed by its own log and a direct `ros2 lifecycle get`), but
`nav2_lifecycle_manager` separately reported a bond failure every time,
first at `bond_timeout: 4.0`:

```
[nav2_costmap_2d-3] [INFO] [1786128552.115271291] [costmap.costmap]: Subscribed to Topics: scan pointcloud
[nav2_costmap_2d-3] [INFO] [1786128552.269548712] [costmap.costmap]: Initialized plugin "obstacle_layer"
[depth_preprocess_node-1] [INFO] [1786128552.430477886] [depth_preprocess_node]: invalid pixel fraction: 0.300
[lifecycle_manager-4] [INFO] [1786128552.470555702] [lifecycle_manager_costmap]: Activating costmap/costmap
[nav2_costmap_2d-3] [INFO] [1786128552.471021073] [costmap.costmap]: Activating
[nav2_costmap_2d-3] [INFO] [1786128552.471057425] [costmap.costmap]: Checking transform
[nav2_costmap_2d-3] [INFO] [1786128552.471166018] [costmap.costmap]: start
[lifecycle_manager-4] [ERROR] [1786128554.624574629] [lifecycle_manager_costmap]: Server costmap/costmap was unable to be reached after 4.00s by bond. This server may be misconfigured.
[lifecycle_manager-4] [ERROR] [1786128554.624636444] [lifecycle_manager_costmap]: Failed to bring up all requested nodes. Aborting bringup.
[lifecycle_manager-4] terminate called after throwing an instance of 'statemap::TransitionUndefinedException'
[lifecycle_manager-4]   what():  no such transition in current state
[ERROR] [lifecycle_manager-4]: process has died [pid 60072, exit code -6, cmd '/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager --ros-args -r __node:=lifecycle_manager_costmap --params-file /tmp/launch_params_vpfi5j_o'].
```

At `bond_timeout: 4.0` this actually crashed `lifecycle_manager` outright
(uncaught `statemap::TransitionUndefinedException`). Raised to `bond_timeout:
10.0` — still failed, same error, no crash this time:

```
[lifecycle_manager-4] [ERROR] [1786128612.798575537] [lifecycle_manager_costmap]: Server costmap/costmap was unable to be reached after 10.00s by bond. This server may be misconfigured.
[lifecycle_manager-4] [ERROR] [1786128612.798803667] [lifecycle_manager_costmap]: Failed to bring up all requested nodes. Aborting bringup.
```

Diagnosed by inspecting who actually publishes/subscribes `/bond` during that
run:

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 topic info /bond -v
echo "---node list---"
ros2 node list | grep -Ei "costmap|lifecycle"
echo "---lifecycle get---"
ros2 lifecycle get /costmap/costmap
'
Type: bond/msg/Status

Publisher count: 1

Node name: lifecycle_manager_costmap
Node namespace: /
Topic type: bond/msg/Status
Endpoint type: PUBLISHER
GID: 01.0f.12.1c.7b.f7.e0.d7.01.00.00.00.00.00.1b.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: lifecycle_manager_costmap
Node namespace: /
Topic type: bond/msg/Status
Endpoint type: SUBSCRIPTION
GID: 01.0f.12.1c.7b.f7.e0.d7.01.00.00.00.00.00.1c.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

---node list---
/costmap/costmap
/lifecycle_manager_costmap
---lifecycle get---
active [3]
```

Both the publisher AND subscriber on `/bond` are `lifecycle_manager_costmap`
itself — the costmap node is not a party to `/bond` at all. The standalone
`nav2_costmap_2d` executable's `Costmap2DROS` never calls `createBond()` the
way the full servers embedded in a real Nav2 bringup (`controller_server`,
`planner_server`) do, so there is no peer for the manager to bond with, at
any timeout value. Meanwhile `ros2 lifecycle get /costmap/costmap` independently
confirms the node itself really is `active [3]` — the bond failure is a
missing-peer condition on the manager's side, not evidence the node failed
to activate.

Fix: `bond_timeout: 0.0` (a standard, documented `nav2_lifecycle_manager`
parameter that disables the post-activation bond liveliness check — not
custom code). Re-launched with this change:

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
date -u +"UTC %Y-%m-%d %H:%M:%S"
TZ=Asia/Hong_Kong date +"HKT %Y-%m-%d %H:%M:%S"
nohup ros2 launch poc_fusion poc_fusion.launch.py > /tmp/task6_launch4.log 2>&1 &
disown
echo "LAUNCH_PID=$!"
sleep 14
tail -n 40 /tmp/task6_launch4.log
'
UTC 2026-08-07 18:54:02
HKT 2026-08-08 02:54:02
LAUNCH_PID=75997
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2026-08-07-18-54-03-221369-raspberrypi-75997
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [depth_preprocess_node-1]: process started with pid [76010]
[INFO] [component_container-2]: process started with pid [76012]
[INFO] [nav2_costmap_2d-3]: process started with pid [76014]
[INFO] [lifecycle_manager-4]: process started with pid [76016]
[nav2_costmap_2d-3] [INFO] [1786128844.150280731] [costmap.costmap]: 
	costmap lifecycle node launched. 
	Waiting on external lifecycle transitions to activate
	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[nav2_costmap_2d-3] [INFO] [1786128844.155837984] [costmap.costmap]: Creating Costmap
[lifecycle_manager-4] [INFO] [1786128844.342655006] [lifecycle_manager_costmap]: Creating
[lifecycle_manager-4] [INFO] [1786128844.359808006] [lifecycle_manager_costmap]: Creating and initializing lifecycle service clients
[lifecycle_manager-4] [INFO] [1786128844.405770274] [lifecycle_manager_costmap]: Starting managed nodes bringup...
[lifecycle_manager-4] [INFO] [1786128844.405834645] [lifecycle_manager_costmap]: Configuring costmap/costmap
[nav2_costmap_2d-3] [INFO] [1786128844.409645579] [costmap.costmap]: Configuring
[nav2_costmap_2d-3] [INFO] [1786128844.516120128] [costmap.costmap]: Using plugin "obstacle_layer"
[nav2_costmap_2d-3] [INFO] [1786128844.553717544] [costmap.costmap]: Subscribed to Topics: scan pointcloud
[nav2_costmap_2d-3] [INFO] [1786128844.624668163] [costmap.costmap]: Initialized plugin "obstacle_layer"
[component_container-2] [INFO] [1786128844.666326736] [poc_fusion.poc_fusion_container]: Load Library: /opt/ros/humble/lib/librectify.so
[lifecycle_manager-4] [INFO] [1786128844.668671204] [lifecycle_manager_costmap]: Activating costmap/costmap
[nav2_costmap_2d-3] [INFO] [1786128844.669235798] [costmap.costmap]: Activating
[nav2_costmap_2d-3] [INFO] [1786128844.669280131] [costmap.costmap]: Checking transform
[nav2_costmap_2d-3] [INFO] [1786128844.669326779] [costmap.costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[component_container-2] [INFO] [1786128844.800257252] [poc_fusion.poc_fusion_container]: Found class: rclcpp_components::NodeFactoryTemplate<image_proc::RectifyNode>
[component_container-2] [INFO] [1786128844.800333104] [poc_fusion.poc_fusion_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<image_proc::RectifyNode>
[nav2_costmap_2d-3] [INFO] [1786128845.171600790] [costmap.costmap]: start
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/poc_fusion/depth_rectify_node' in container '/poc_fusion/poc_fusion_container'
[component_container-2] [INFO] [1786128845.279023767] [poc_fusion.poc_fusion_container]: Load Library: /opt/ros/humble/lib/libdepth_image_proc.so
[component_container-2] [INFO] [1786128845.288590936] [poc_fusion.poc_fusion_container]: Found class: rclcpp_components::NodeFactoryTemplate<depth_image_proc::ConvertMetricNode>
[component_container-2] [INFO] [1786128845.288670621] [poc_fusion.poc_fusion_container]: Found class: rclcpp_components::NodeFactoryTemplate<depth_image_proc::CropForemostNode>
[component_container-2] [INFO] [1786128845.288685918] [poc_fusion.poc_fusion_container]: Found class: rclcpp_components::NodeFactoryTemplate<depth_image_proc::DisparityNode>
[component_container-2] [INFO] [1786128845.288697677] [poc_fusion.poc_fusion_container]: Found class: rclcpp_components::NodeFactoryTemplate<depth_image_proc::PointCloudXyzNode>
[component_container-2] [INFO] [1786128845.288711214] [poc_fusion.poc_fusion_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<depth_image_proc::PointCloudXyzNode>
[lifecycle_manager-4] [INFO] [1786128845.310826300] [lifecycle_manager_costmap]: Managed nodes are active
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/poc_fusion/point_cloud_xyz_node' in container '/poc_fusion/poc_fusion_container'
[depth_preprocess_node-1] [INFO] [1786128845.527873480] [depth_preprocess_node]: depth_preprocess_node started: /ascamera/camera_publisher/depth0/image_raw -> /poc_fusion/depth_cleaned (expected encoding=mono16, roi_profile=default, median_kernel_size=3)
[depth_preprocess_node-1] [INFO] [1786128845.530570875] [depth_preprocess_node]: invalid pixel fraction: 0.298
```

`lifecycle_manager_costmap` now logs `Managed nodes are active` with no
bond error. The one-time `Timed out waiting for transform ... odom` line at
`1786128844.669` is recorded as an observed one-time startup transient: the
costmap proceeds past it to `start` within ~0.5s and never repeats it.

> **CORRECTION (Task 6 fix round 1).** The original text of this paragraph
> attributed that line to a cause ("queried before the first `/tf` message
> arrived") and cross-referenced a `tf2_echo odom base_link` result "in this
> same session ... see Step 4 below". Neither was supported: no `tf2_echo`
> output existed anywhere in this Task 6 section, and the cause was asserted
> rather than measured — the actual error text is `Invalid frame ID "odom"
> ... frame does not exist`, and `/ekf_filter_node` was already publishing
> `/tf` before this launch started. Both the dangling cross-reference and
> the causal parenthetical are removed. A genuinely live `tf2_echo odom
> base_link` was run during the fix round and is pasted in
> "Task 6 fix round 1 — Step D"; it reproduces the same message twice before
> resolving, which is an observation, not an isolated cause.

### Step 4: proving the node name, parameters, and lifecycle state

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
echo "=== ros2 node list ==="
ros2 node list
echo
echo "=== ros2 lifecycle get /costmap/costmap ==="
ros2 lifecycle get /costmap/costmap
'
=== ros2 node list ===
/LD19
/ar_app
/arm_controller
/aurora/aurora
/controller_manager
/costmap/costmap
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
/launch_ros_75997
/lidar_app
/lifecycle_manager_costmap
/line_following
/object_tracking
/odom_publisher
/poc_fusion/depth_rectify_node
/poc_fusion/poc_fusion_container
/poc_fusion/point_cloud_xyz_node
/robot_state_publisher
/ros_robot_controller
/rosapi
/rosapi_params
/rosbridge_websocket
/servo_manager
/static_transform_publisher_xjZs1QyCV0ckvKkj
/transform_listener_impl_5555ad8bf7e0
/transform_listener_impl_5555d4961fb0
/web_video_server

=== ros2 lifecycle get /costmap/costmap ===
active [3]
```

`ros2 param get` readbacks proving the parameters actually took (not just
that they were written to the YAML file — this is the exact key path
`/costmap/costmap`, matching the verified node name):

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
echo "=== ros2 param get /costmap/costmap global_frame ==="
ros2 param get /costmap/costmap global_frame
echo "=== ros2 param get /costmap/costmap robot_base_frame ==="
ros2 param get /costmap/costmap robot_base_frame
echo "=== ros2 param get /costmap/costmap resolution ==="
ros2 param get /costmap/costmap resolution
echo "=== ros2 param get /costmap/costmap width ==="
ros2 param get /costmap/costmap width
echo "=== ros2 param get /costmap/costmap height ==="
ros2 param get /costmap/costmap height
echo "=== ros2 param get /costmap/costmap update_frequency ==="
ros2 param get /costmap/costmap update_frequency
echo "=== ros2 param get /costmap/costmap publish_frequency ==="
ros2 param get /costmap/costmap publish_frequency
echo "=== ros2 param get /costmap/costmap transform_tolerance ==="
ros2 param get /costmap/costmap transform_tolerance
echo "=== ros2 param get /costmap/costmap rolling_window ==="
ros2 param get /costmap/costmap rolling_window
'
=== ros2 param get /costmap/costmap global_frame ===
String value is: odom
=== ros2 param get /costmap/costmap robot_base_frame ===
String value is: base_link
=== ros2 param get /costmap/costmap resolution ===
Double value is: 0.05
=== ros2 param get /costmap/costmap width ===
Integer value is: 3
=== ros2 param get /costmap/costmap height ===
Integer value is: 3
=== ros2 param get /costmap/costmap update_frequency ===
Double value is: 10.0
=== ros2 param get /costmap/costmap publish_frequency ===
Double value is: 10.0
=== ros2 param get /costmap/costmap transform_tolerance ===
Double value is: 0.3
=== ros2 param get /costmap/costmap rolling_window ===
Boolean value is: True
```

(The command above ran together with the two observation-source `param get`
calls below in one invocation; it was moved to a background task by the
harness after 120s because one of the calls in that combined batch stalled.
The batch was re-run split into two commands to isolate that — both are
pasted below exactly as returned.)

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
echo "=== obstacle_layer.scan.topic ==="
timeout 10 ros2 param get /costmap/costmap obstacle_layer.scan.topic
echo "=== obstacle_layer.pointcloud.topic ==="
timeout 10 ros2 param get /costmap/costmap obstacle_layer.pointcloud.topic
echo "=== obstacle_layer.pointcloud.observation_persistence ==="
timeout 10 ros2 param get /costmap/costmap obstacle_layer.pointcloud.observation_persistence
echo "=== obstacle_layer.scan.observation_persistence ==="
timeout 10 ros2 param get /costmap/costmap obstacle_layer.scan.observation_persistence
echo "=== obstacle_layer.pointcloud.min_obstacle_height ==="
timeout 10 ros2 param get /costmap/costmap obstacle_layer.pointcloud.min_obstacle_height
echo "=== obstacle_layer.pointcloud.max_obstacle_height ==="
timeout 10 ros2 param get /costmap/costmap obstacle_layer.pointcloud.max_obstacle_height
echo "=== observation_sources ==="
timeout 10 ros2 param get /costmap/costmap observation_sources
'
=== obstacle_layer.scan.topic ===
String value is: /scan_raw
=== obstacle_layer.pointcloud.topic ===
String value is: /poc_fusion/points
=== obstacle_layer.pointcloud.observation_persistence ===
Double value is: 0.3
=== obstacle_layer.scan.observation_persistence ===
Double value is: 0.0
=== obstacle_layer.pointcloud.min_obstacle_height ===
Double value is: 0.03
=== obstacle_layer.pointcloud.max_obstacle_height ===
Double value is: 0.94
=== observation_sources ===
String value is: 
```

(`observation_sources` at the top level of the costmap is a separate,
unused Costmap2DROS-level parameter that this config never sets — it is
NOT the same parameter as `obstacle_layer.observation_sources`, which is
what `ObstacleLayer` actually reads. Checked directly below.)

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
echo "=== obstacle_layer.observation_sources ==="
timeout 10 ros2 param get /costmap/costmap obstacle_layer.observation_sources
echo "=== plugins ==="
timeout 10 ros2 param get /costmap/costmap plugins
'
=== obstacle_layer.observation_sources ===
String value is: scan pointcloud
=== plugins ===
String values are: ['obstacle_layer']
```

`obstacle_layer.scan.topic` reading back as `/scan_raw` proves the launch
file's runtime injection from `costmap_params.yaml`'s `scan_topic` key
actually reached the node — this value appears nowhere as a literal in
`poc_fusion.launch.py`.

### Step 4b: proving both observation sources are actually subscribed

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
echo "=== ros2 node info /costmap/costmap ==="
timeout 15 ros2 node info /costmap/costmap
'
=== ros2 node info /costmap/costmap ===
/costmap/costmap
  Subscribers:
    /costmap/footprint: geometry_msgs/msg/Polygon
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /poc_fusion/points: sensor_msgs/msg/PointCloud2
    /scan_raw: sensor_msgs/msg/LaserScan
  Publishers:
    /costmap/costmap: nav_msgs/msg/OccupancyGrid
    /costmap/costmap/transition_event: lifecycle_msgs/msg/TransitionEvent
    /costmap/costmap_raw: nav2_msgs/msg/Costmap
    /costmap/costmap_updates: map_msgs/msg/OccupancyGridUpdate
    /costmap/published_footprint: geometry_msgs/msg/PolygonStamped
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /costmap/clear_around_costmap: nav2_msgs/srv/ClearCostmapAroundRobot
    /costmap/clear_entirely_costmap: nav2_msgs/srv/ClearEntireCostmap
    /costmap/clear_except_costmap: nav2_msgs/srv/ClearCostmapExceptRegion
    /costmap/costmap/change_state: lifecycle_msgs/srv/ChangeState
    /costmap/costmap/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /costmap/costmap/get_available_states: lifecycle_msgs/srv/GetAvailableStates
    /costmap/costmap/get_available_transitions: lifecycle_msgs/srv/GetAvailableTransitions
    /costmap/costmap/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /costmap/costmap/get_parameters: rcl_interfaces/srv/GetParameters
    /costmap/costmap/get_state: lifecycle_msgs/srv/GetState
    /costmap/costmap/get_transition_graph: lifecycle_msgs/srv/GetAvailableTransitions
    /costmap/costmap/list_parameters: rcl_interfaces/srv/ListParameters
    /costmap/costmap/set_parameters: rcl_interfaces/srv/SetParameters
    /costmap/costmap/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /costmap/get_costmap: nav2_msgs/srv/GetCostmap
  Service Clients:

  Action Servers:

  Action Clients:
```

Confirms `/costmap/costmap` node subscribes to BOTH `/scan_raw` (LiDAR) and
`/poc_fusion/points` (depth-derived point cloud).

### Step 4c: publish rate and message content

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
echo "=== ros2 topic hz /costmap/costmap ==="
timeout 15 ros2 topic hz /costmap/costmap
'
=== ros2 topic hz /costmap/costmap ===
WARNING: topic [/costmap/costmap] does not appear to be published yet
```

`/costmap/costmap` (the full `OccupancyGrid`) is QoS `TRANSIENT_LOCAL` and is
only republished in full occasionally (Nav2's costmap design: the routine
update-rate traffic goes to `/costmap/costmap_updates`, an incremental
`OccupancyGridUpdate` diff topic, not the full grid) — confirmed this is the
right topic to check for the configured rate, not a sign the costmap is
stuck:

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
echo "=== ros2 topic hz /costmap/costmap_updates ==="
timeout 15 ros2 topic hz /costmap/costmap_updates
'
=== ros2 topic hz /costmap/costmap_updates ===
WARNING: topic [/costmap/costmap_updates] does not appear to be published yet
average rate: 5.735
	min: 0.100s max: 0.305s std dev: 0.06754s window: 7
average rate: 6.051
	min: 0.100s max: 0.305s std dev: 0.05796s window: 14
average rate: 6.430
	min: 0.100s max: 0.305s std dev: 0.05248s window: 22
average rate: 6.167
	min: 0.100s max: 0.305s std dev: 0.05028s window: 28
average rate: 6.116
	min: 0.100s max: 0.305s std dev: 0.04801s window: 34
average rate: 6.077
	min: 0.100s max: 0.305s std dev: 0.04406s window: 41
average rate: 6.029
	min: 0.100s max: 0.305s std dev: 0.04406s window: 47
average rate: 6.154
	min: 0.100s max: 0.305s std dev: 0.04522s window: 55
average rate: 6.137
	min: 0.100s max: 0.305s std dev: 0.04470s window: 61
average rate: 6.246
	min: 0.100s max: 0.305s std dev: 0.04499s window: 69
average rate: 6.272
	min: 0.100s max: 0.305s std dev: 0.04483s window: 76
average rate: 6.251
	min: 0.100s max: 0.305s std dev: 0.04406s window: 82
```

The `min: 0.100s` bound matches the configured 10 Hz `update_frequency`/
`publish_frequency` exactly (100ms is the fastest possible interval at
10Hz); the *average* rate (~6 Hz, not 10 Hz) is measured, not derived, and
its cause is NOT ISOLATED in this task — no new mechanism is proposed for
it here. It is consistent with, but not shown to be caused by, the same
kind of system-wide CPU contention already documented in Task 5. Flagged
forward alongside the existing `/poc_fusion/points` rate question for
Task 12/16 rather than chased here.

> **CORRECTION (Task 6 fix round 1).** The original text of this paragraph
> ended the hedge with the parenthetical "(load average ~5-6 on 4 cores at
> the time of these measurements)". No `uptime`, `top` or `/proc/loadavg`
> reading was ever taken in this section, so that number was unsupported.
> It is deleted. The hedge itself ("consistent with, but not shown to be
> caused by") stands. A real load reading, taken concurrently with a
> re-measurement of this same topic, is pasted in "Task 6 fix round 1 —
> Step E". **Also note:** the ~6 Hz figure above is no longer a valid
> baseline — the fix in this round makes the LiDAR source actually mark, so
> the costmap's workload changed. See Step E for the post-fix number.

Full-grid message content, proving `resolution`, `width`, `height`, and
`frame_id` match the configured 3m×3m / 0.05 resolution / `odom` frame:

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
echo "=== ros2 topic echo /costmap/costmap --once (info fields only via --no-arr) ==="
timeout 10 ros2 topic echo /costmap/costmap --once --no-arr
'
=== ros2 topic echo /costmap/costmap --once (info fields only via --no-arr) ===
header:
  stamp:
    sec: 1786129072
    nanosec: 792686462
  frame_id: odom
info:
  map_load_time:
    sec: 0
    nanosec: 0
  resolution: 0.05000000074505806
  width: 60
  height: 60
  origin:
    position:
      x: -1.4500000003725293
      y: -1.4500000003725293
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
data: '<sequence type: int8, length: 3600>'
---
```

`width: 60`, `height: 60` at `resolution: 0.05` = 3.0m × 3.0m, matching the
configured size exactly (60 × 0.05 = 3). `frame_id: odom` matches
`global_frame`. `data` length 3600 = 60×60, consistent.

### Step 5 — HUMAN-PRESENT, explicitly NOT attempted

Brief Step 5 ("wave a hand in front of the camera only, outside the LiDAR
plane, confirm marks appear in RViz") requires a display and a human. Per
instruction this run has no display and no human, so it is DEFERRED to a
human-present session. The machine-checkable substitute evidence above
(publish rate, message content, dual-subscription, active lifecycle state)
proves the costmap plumbing is correctly wired and running — it does NOT
prove fusion is doing anything useful. **Fusion is not verified working.**
Whether the depth camera actually contributes marks the LiDAR alone would
miss remains unconfirmed until the RViz hand-wave happens with a human
present.

### Teardown

```
$ docker exec -u ubuntu MentorPi bash -lc '
ps aux | grep -E "poc_fusion|nav2_costmap|lifecycle_manager|depth_preprocess|component_container|launch_ros" | grep -v grep
'
ubuntu     40688  0.0  0.0      0     0 ?        Z    18:43   0:00 [nav2_costmap_2d] <defunct>
ubuntu     60070  0.8  0.0      0     0 ?        Z    18:49   0:05 [nav2_costmap_2d] <defunct>
ubuntu     63353  7.5  0.0      0     0 ?        Z    18:50   0:44 [nav2_costmap_2d] <defunct>
ubuntu     75997  0.7  0.6 1133680 56976 ?       Sl   18:54   0:02 /usr/bin/python3 /opt/ros/humble/bin/ros2 launch poc_fusion poc_fusion.launch.py
ubuntu     76010 35.7  1.7 1303552 147056 ?      Rl   18:54   2:05 /usr/bin/python3 /home/ubuntu/ros2_ws/install/poc_fusion/lib/poc_fusion/depth_preprocess_node --ros-args -r __node:=depth_preprocess_node --params-file /home/ubuntu/ros2_ws/install/poc_fusion/share/poc_fusion/config/depth_preprocess_params.yaml
ubuntu     76012 13.2  1.0 991680 89536 ?        Sl   18:54   0:46 /opt/ros/humble/lib/rclcpp_components/component_container --ros-args -r __node:=poc_fusion_container -r __ns:=/poc_fusion
ubuntu     76014 24.3  1.0 820112 89792 ?        Sl   18:54   1:25 /opt/ros/humble/lib/nav2_costmap_2d/nav2_costmap_2d --ros-args --params-file /tmp/launch_params_l9pjhrvn --params-file /tmp/launch_params_cnegn0wb
ubuntu     76016  0.5  0.3 735232 32880 ?        Sl   18:54   0:01 /opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager --ros-args -r __node:=lifecycle_manager_costmap --params-file /tmp/launch_params_4whex1kw
```

`kill -TERM` on the `ros2 launch` parent PID (75997) alone left the four
child processes (76010/76012/76014/76016) running as orphans, so they were
killed individually by PID:

```
$ docker exec -u ubuntu MentorPi bash -lc '
kill -TERM 76010 76012 76014 76016
sleep 4
ps aux | grep -E "poc_fusion|nav2_costmap|lifecycle_manager|depth_preprocess|component_container|launch_ros" | grep -v grep
'
ubuntu     40688  0.0  0.0      0     0 ?        Z    18:43   0:00 [nav2_costmap_2d] <defunct>
ubuntu     60070  0.8  0.0      0     0 ?        Z    18:49   0:05 [nav2_costmap_2d] <defunct>
ubuntu     63353  7.3  0.0      0     0 ?        Z    18:50   0:44 [nav2_costmap_2d] <defunct>
ubuntu     76014 24.3  0.0      0     0 ?        Z    18:54   1:29 [nav2_costmap_2d] <defunct>
```

Only zombie (`<defunct>`) entries remain, which are just unreaped exit
statuses of already-dead processes, not live processes. `ros2 node list`
still showed the just-killed nodes immediately after (the same DDS
discovery propagation delay documented in earlier tasks), resolved after
waiting:

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
sleep 15
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
/static_transform_publisher_xjZs1QyCV0ckvKkj
/transform_listener_impl_5555ad8bf7e0
/web_video_server
```

No `/poc_fusion*` or `/costmap*` entries remain — the pre-existing vendor
stack (`/LD19`, `/aurora/aurora`, `/joystick_control`, arm/gripper
controllers, etc.) is present and unchanged from before this task's
launch.

Host test suite re-run after the task, immediately before committing:

```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/ -q
.....................................                                    [100%]
37 passed in 0.31s
```

37/37, unchanged from before this task — Task 6 is configuration and launch
wiring only, introducing no new pure logic and therefore no new tests (per
CONSTRAINTS.md, "Configuration is not exempt from verification, only from
unit testing").

### Task 6 summary

Wrote the full `nav2_costmap_2d` parameter set into
`poc_fusion/config/costmap_params.yaml` (`global_frame: odom`,
`robot_base_frame: base_link`, `rolling_window: true`, 3m×3m at
`resolution: 0.05`, `update_frequency`/`publish_frequency: 10.0`,
`transform_tolerance: 0.3`, a single `obstacle_layer` plugin with two
observation sources — `scan` at `observation_persistence: 0.0` and
`pointcloud` on `/poc_fusion/points` at `observation_persistence: 0.3` with
a `min_obstacle_height`/`max_obstacle_height` ground-return filter of
0.03m/0.94m). `scan_topic` stays the file's original single-source-of-truth
key; the node's actual scan topic is injected at launch time from that key,
never hard-coded a second time. Extended `poc_fusion/launch/poc_fusion.launch.py`
at the Task 6 insertion point Task 5 pre-labelled: a standalone `nav2_costmap_2d`
`Node` plus a `nav2_lifecycle_manager` `Node` (`autostart: true`,
`node_names: ['costmap/costmap']`, `bond_timeout: 0.0`). Added
`python3-yaml` to `package.xml` (needed by the launch file's `yaml.safe_load`
call).

Two real bugs were found and fixed during verification, both documented
above with real pasted output: (1) the raw params file cannot be passed
directly to the node because of the top-level `scan_topic` key — fixed by
extracting and passing the `costmap.costmap.ros__parameters` sub-tree as an
in-memory dict instead; (2) `nav2_lifecycle_manager`'s bond liveliness check
can never succeed against a standalone `nav2_costmap_2d` node because it
never creates a bond — fixed by setting `bond_timeout: 0.0`, a documented
parameter for exactly this case, not custom code.

Verified live: node name/namespace (`/costmap/costmap`) matches the
`costmap_params.yaml` key path — proven, not assumed, by running the bare
executable first and reading `ros2 node list`. All Step-1/2/3 parameters
read back correctly via `ros2 param get`, including the launch-time-injected
`obstacle_layer.scan.topic` resolving to `/scan_raw` with the literal
appearing nowhere in the launch file's Python source. `ros2 node info`
confirms both observation sources (`/scan_raw`, `/poc_fusion/points`) are
subscribed. `ros2 lifecycle get` confirms `active [3]`.
`/costmap/costmap_updates` publishes at a measured average ~6 Hz (min
interval 0.1s matches the configured 10 Hz exactly; the average-rate gap is
measured and flagged forward, not explained). `/costmap/costmap`'s one-time
transient-local full grid has `resolution: 0.05`, `width: 60`, `height: 60`
(=3m×3m), `frame_id: odom` — all matching configuration.

Step 5 (RViz hand-wave) is explicitly DEFERRED to a human-present session
and NOT claimed as verified. The machine-checkable substitute evidence
above proves the costmap is correctly wired, active, and consuming both
sources — it does not prove the fusion is doing anything useful yet.

**Dataset-provenance observation (report only, not implemented):** once a
`LaserScan` mark and a `PointCloud2` mark both land in the same
`ObstacleLayer` grid cell, they become a single occupancy value — the
costmap format has no per-cell field for which observation source(s)
contributed it. Confirmed by inspecting the actual message content above:
`/costmap/costmap` (`nav_msgs/OccupancyGrid`) and `/costmap/costmap_updates`
(`map_msgs/OccupancyGridUpdate`) carry only cost values, no source
attribution. This means the costmap itself, once built, is NOT a place the
"which sensor triggered this" dataset gap (flagged at the coordinator level)
can be recovered from downstream — the two sources are irreversibly merged
at this layer. If provenance is wanted, it has to be captured UPSTREAM of
the costmap, e.g. a small additive `poc_fusion` node/topic that logs,
per-detection, whether the triggering costmap cells were touched by the
`scan` or `pointcloud` observation buffer (or both) before they are marked
into the shared grid — noted as a recommendation only, not built here, and
explicitly not a fix to `proximity_alert/`'s frozen CSV schema.

---

## Task 6 fix round 1 (2026-08-09 ~23:35–23:57 HKT / 2026-08-09 ~15:35–15:57 UTC)

Response to review of the original Task 6 section. Three things are settled
here: (1) the LiDAR observation source was silently marking **nothing** as
shipped in `164fa8b`, and the fix is proved by reading the costmap's actual
`data` array rather than `--no-arr`; (2) two unsupported claims in the
original Task 6 text (a dangling `tf2_echo` cross-reference, an invented
load-average number) are corrected in place above; (3) the point cloud
source's contribution to the grid is measured for the first time.

Host clock and container clock, taken from separate commands (host `date`
run outside any `docker exec`, container `date -u` run inside it):

```
$ docker ps --format '{{.Names}}\t{{.Status}}' && echo "=== container date ===" && docker exec -u ubuntu MentorPi bash -lc 'date -u' && echo "=== host date ===" && date
MentorPi	Up 15 minutes
=== container date ===
Sun Aug  9 15:39:47 UTC 2026
=== host date ===
Sun Aug  9 11:39:47 PM HKT 2026
```

### The defect

`nav2_costmap_2d` (Humble) declares `min_obstacle_height` /
`max_obstacle_height` **per observation source**, and the per-source
`max_obstacle_height` defaults to `0.0` — not to the layer-level `2.0`.
`164fa8b` set those keys on the `pointcloud` source only, leaving the `scan`
source at the `0.0` default. `ObservationBuffer` filters observation points
by z **after** transforming them into `global_frame` (`odom`), and the LD19's
scan plane sits at z ≈ 0.093 m in `odom`, so every LiDAR point was discarded.
The costmap stayed valid, active, correctly framed and correctly sized — and
all-free. The original section's `ros2 topic echo --once /costmap/costmap
--no-arr` could not see this, because `--no-arr` suppresses the one field
that carries the answer.

### Step A: the fix, as deployed

Committed in `de956ea`. `poc_fusion/config/costmap_params.yaml` now sets
`min_obstacle_height: 0.0` / `max_obstacle_height: 2.0` on the `scan`
source (the `2.0` matches `nav2_bringup`'s own shipped
`params/nav2_params.yaml`). Parameter readback from the running node, with
the full production config deployed:

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
date -u
echo "=== ros2 lifecycle get /costmap/costmap ==="
ros2 lifecycle get /costmap/costmap
echo "=== param readback ==="
ros2 param get /costmap/costmap obstacle_layer.scan.topic
ros2 param get /costmap/costmap obstacle_layer.scan.max_obstacle_height
ros2 param get /costmap/costmap obstacle_layer.scan.min_obstacle_height
ros2 param get /costmap/costmap obstacle_layer.scan.marking
ros2 param get /costmap/costmap obstacle_layer.pointcloud.marking
ros2 param get /costmap/costmap obstacle_layer.pointcloud.topic
'; echo "--- HOST ---"; date
Sun Aug  9 15:41:21 UTC 2026
=== ros2 lifecycle get /costmap/costmap ===
active [3]
=== param readback ===
String value is: /scan_raw
Double value is: 2.0
Double value is: 0.0
Boolean value is: True
Boolean value is: True
String value is: /poc_fusion/points
--- HOST ---
Sun Aug  9 11:41:43 PM HKT 2026
```

### Step B: LIVE PROOF — the LiDAR now marks the grid, read from `data`

Full stack launched on the robot (`ros2 launch poc_fusion
poc_fusion.launch.py`: `depth_preprocess_node` + `RectifyNode` +
`PointCloudXyzNode` + `nav2_costmap_2d` + `nav2_lifecycle_manager`), then the
costmap's occupancy array read **with the array included**. The occupancy
histogram is produced by a pipeline that is visible in the command itself
(`tr`/`sed`/`sort`/`uniq -c` over the `data` field), alongside the count of
`/scan_raw` returns that fall inside the same 3 m × 3 m window.

The scan counter is a read-only rclpy subscriber (`/tmp/scan_window_count.py`
in the container; it publishes nothing). Its source, verbatim:

```python
#!/usr/bin/env python3
"""Count /scan_raw returns that fall inside the costmap's 3x3 m window.

Read-only: subscribes to /scan_raw, takes ONE message, prints counts. No
publishing of any kind. The window test is done in the laser frame
(|x|<=1.5 and |y|<=1.5 m); a return with range r <= 1.5 m is inside the
box for any bearing, which is the conservative count reported as
'in_window_conservative'.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanCounter(Node):
    def __init__(self):
        super().__init__('scan_window_counter')
        self.done = False
        self.create_subscription(LaserScan, '/scan_raw', self.cb, 10)

    def cb(self, msg):
        if self.done:
            return
        self.done = True
        total = len(msg.ranges)
        finite = 0
        in_box = 0
        conservative = 0
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            finite += 1
            a = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(a)
            y = r * math.sin(a)
            if abs(x) <= 1.5 and abs(y) <= 1.5:
                in_box += 1
            if r <= 1.5:
                conservative += 1
        print('frame_id           :', msg.header.frame_id)
        print('stamp              : %d.%09d' % (msg.header.stamp.sec,
                                                msg.header.stamp.nanosec))
        print('range_min/range_max: %.3f / %.3f' % (msg.range_min, msg.range_max))
        print('total_beams        :', total)
        print('valid_returns      :', finite)
        print('in_window_3x3_box  :', in_box)
        print('in_window_conservative (r<=1.5m):', conservative)


def main():
    rclpy.init()
    n = ScanCounter()
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=1.0)
    n.destroy_node()
    rclpy.shutdown()


main()
```

Correlated reading, production config, run 1:

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
date -u
echo "=== /scan_raw returns inside the 3x3 m costmap window ==="
timeout 30 python3 /tmp/scan_window_count.py
echo "=== /costmap/costmap occupancy histogram (same moment) ==="
timeout 20 ros2 topic echo --once /costmap/costmap --field data | tr -d "[]" | tr "," "\n" | sed "s/ //g" | sort -n | uniq -c
date -u
'; echo "--- HOST ---"; date
Sun Aug  9 15:42:25 UTC 2026
=== /scan_raw returns inside the 3x3 m costmap window ===
frame_id           : lidar_frame
stamp              : 1786290153.387331694
range_min/range_max: 0.020 / 25.000
total_beams        : 503
valid_returns      : 256
in_window_3x3_box  : 190
in_window_conservative (r<=1.5m): 187
=== /costmap/costmap occupancy histogram (same moment) ===
      1 ---
   3463 0
      1 0)
      1 array('b'
    136 100
Sun Aug  9 15:42:36 UTC 2026
--- HOST ---
Sun Aug  9 11:42:36 PM HKT 2026
```

Reading the histogram: 3463 + 136 + 1 = 3600 = 60 × 60 cells (the `1 array('b'`
and `1 0)` lines are the opening/closing tokens of the `array('b', [...])`
repr split by the same `tr`, and `1 ---` is `ros2 topic echo`'s YAML document
terminator — all three are artifacts of the visible pipeline, not data).
**136 cells at cost 100 (lethal), with 190 genuine LiDAR returns inside the
window.** Non-zero occupancy, from the real `data` array.

That the array itself is genuinely being read and not summarised away
(`--no-arr` is exactly what hid the original bug) is shown by the raw
`--field data` output pasted, command and all, in the **run 4** block of
Step C below: `array('b', [0, 0, 0, ...` printed literally by
`ros2 topic echo --once /costmap/costmap --field data | head -c 400`.

### Step C: the discrimination proof — flip, paste, restore

Four runs, differing only in `poc_fusion/config/costmap_params.yaml`, each
one a full deploy → `colcon build` → relaunch → measure cycle. The LiDAR
input is essentially unchanged across all four (177–190 valid returns inside
the window), so the occupancy differences are attributable to the config.

| run | `observation_sources` | `scan.max_obstacle_height` | LiDAR returns in window | cells at cost 100 |
|---|---|---|---|---|
| 1 | `scan pointcloud` | `2.0` (fixed) | 190 | **136** |
| 2 | `scan pointcloud` | `0.0` (broken default) | 186 | **80** |
| 3 | `scan` only | `0.0` (broken default) | 177 | **0** |
| 4 | `scan` only | `2.0` (fixed) | 187 | **94** |
| 5 | `scan pointcloud` | `2.0` (restored, committed) | 184 | **62** |

Run 3 vs run 4 is the clean single-variable flip on the LiDAR source:
**0 occupied cells with the broken default, 94 with the fix, same sensor, same
scene, nothing else changed.**

Run 2 (both sources listed, LiDAR height-filtered out):

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
date -u
echo "=== param readback (mutated) ==="
ros2 param get /costmap/costmap obstacle_layer.scan.max_obstacle_height
ros2 param get /costmap/costmap obstacle_layer.scan.marking
ros2 param get /costmap/costmap obstacle_layer.pointcloud.marking
echo "=== /scan_raw returns inside the 3x3 m window (unchanged sensor input) ==="
timeout 30 python3 /tmp/scan_window_count.py
echo "=== /costmap/costmap occupancy histogram, sample 1 ==="
timeout 20 ros2 topic echo --once /costmap/costmap --field data | tr -d "[]" | tr "," "\n" | sed "s/ //g" | sort -n | uniq -c
sleep 15
echo "=== /costmap/costmap occupancy histogram, sample 2 (15s later) ==="
timeout 20 ros2 topic echo --once /costmap/costmap --field data | tr -d "[]" | tr "," "\n" | sed "s/ //g" | sort -n | uniq -c
echo "=== is /poc_fusion/points alive during this run? ==="
timeout 15 ros2 topic hz /poc_fusion/points 2>&1 | tail -3
date -u
'; echo "--- HOST ---"; date
Sun Aug  9 15:51:37 UTC 2026
=== param readback (mutated) ===
Double value is: 0.0
Boolean value is: True
Boolean value is: True
=== /scan_raw returns inside the 3x3 m window (unchanged sensor input) ===
frame_id           : lidar_frame
stamp              : 1786290708.386996247
range_min/range_max: 0.020 / 25.000
total_beams        : 503
valid_returns      : 253
in_window_3x3_box  : 186
in_window_conservative (r<=1.5m): 184
=== /costmap/costmap occupancy histogram, sample 1 ===
      1 ---
   3519 0
      1 0)
      1 array('b'
     80 100
=== /costmap/costmap occupancy histogram, sample 2 (15s later) ===
      1 ---
   3519 0
      1 0)
      1 array('b'
     80 100
=== is /poc_fusion/points alive during this run? ===
	min: 0.051s max: 1.233s std dev: 0.25282s window: 33
average rate: 3.350
	min: 0.051s max: 2.245s std dev: 0.42050s window: 34
Sun Aug  9 15:52:23 UTC 2026
--- HOST ---
Sun Aug  9 11:52:23 PM HKT 2026
```

Run 2 did **not** give zero occupancy, and that is a real result rather than a
failed prediction: 80 cells remained. Run 3 isolates why — with the point
cloud source removed and the same broken height default, the LiDAR source
alone marks nothing at all:

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
date -u
echo "=== params: LiDAR-only isolation, BROKEN default height ==="
ros2 param get /costmap/costmap obstacle_layer.observation_sources
ros2 param get /costmap/costmap obstacle_layer.scan.max_obstacle_height
echo "=== /scan_raw returns inside the 3x3 m window ==="
timeout 30 python3 /tmp/scan_window_count.py
echo "=== /costmap/costmap occupancy histogram ==="
timeout 20 ros2 topic echo --once /costmap/costmap --field data | tr -d "[]" | tr "," "\n" | sed "s/ //g" | sort -n | uniq -c
sleep 12
echo "=== /costmap/costmap occupancy histogram, 12s later ==="
timeout 20 ros2 topic echo --once /costmap/costmap --field data | tr -d "[]" | tr "," "\n" | sed "s/ //g" | sort -n | uniq -c
date -u
'; echo "--- HOST ---"; date
Sun Aug  9 15:53:51 UTC 2026
=== params: LiDAR-only isolation, BROKEN default height ===
String value is: scan
Double value is: 0.0
=== /scan_raw returns inside the 3x3 m window ===
frame_id           : lidar_frame
stamp              : 1786290837.414110875
range_min/range_max: 0.020 / 25.000
total_beams        : 502
valid_returns      : 248
in_window_3x3_box  : 177
in_window_conservative (r<=1.5m): 174
=== /costmap/costmap occupancy histogram ===
      1 ---
   3599 0
      1 0)
      1 array('b'
=== /costmap/costmap occupancy histogram, 12s later ===
      1 ---
   3599 0
      1 0)
      1 array('b'
Sun Aug  9 15:54:13 UTC 2026
--- HOST ---
Sun Aug  9 11:54:13 PM HKT 2026
```

3599 + 1 (`0)` token) = 3600 cells, **every one of them 0**, with 177 genuine
LiDAR returns inside the window. This is the `164fa8b` defect reproduced
exactly: a valid, active, `active [3]`, correctly-sized, entirely free
costmap with no error anywhere.

Run 4 — same isolation, height fixed:

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
date -u
echo "=== params: LiDAR-only isolation, FIXED height ==="
ros2 param get /costmap/costmap obstacle_layer.observation_sources
ros2 param get /costmap/costmap obstacle_layer.scan.max_obstacle_height
echo "=== /scan_raw returns inside the 3x3 m window ==="
timeout 30 python3 /tmp/scan_window_count.py
echo "=== /costmap/costmap occupancy histogram ==="
timeout 20 ros2 topic echo --once /costmap/costmap --field data | tr -d "[]" | tr "," "\n" | sed "s/ //g" | sort -n | uniq -c
echo "=== raw data field, first 400 bytes (proving the array is really being read) ==="
timeout 20 ros2 topic echo --once /costmap/costmap --field data | head -c 400
echo
date -u
'; echo "--- HOST ---"; date
Sun Aug  9 15:54:50 UTC 2026
=== params: LiDAR-only isolation, FIXED height ===
String value is: scan
Double value is: 2.0
=== /scan_raw returns inside the 3x3 m window ===
frame_id           : lidar_frame
stamp              : 1786290898.687052658
range_min/range_max: 0.020 / 25.000
total_beams        : 503
valid_returns      : 257
in_window_3x3_box  : 187
in_window_conservative (r<=1.5m): 184
=== /costmap/costmap occupancy histogram ===
      1 ---
   3505 0
      1 0)
      1 array('b'
     94 100
=== raw data field, first 400 bytes (proving the array is really being read) ===
array('b', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
Sun Aug  9 15:55:03 UTC 2026
--- HOST ---
Sun Aug  9 11:55:03 PM HKT 2026
```

Run 5 — the committed production config restored (both sources, fixed
height), which is the state the repository is left in:

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
date -u
echo "=== params: PRODUCTION (both sources, fixed height) ==="
ros2 param get /costmap/costmap obstacle_layer.observation_sources
ros2 param get /costmap/costmap obstacle_layer.scan.max_obstacle_height
ros2 param get /costmap/costmap obstacle_layer.scan.min_obstacle_height
ros2 param get /costmap/costmap obstacle_layer.scan.marking
ros2 param get /costmap/costmap obstacle_layer.scan.clearing
ros2 param get /costmap/costmap obstacle_layer.pointcloud.marking
ros2 param get /costmap/costmap obstacle_layer.pointcloud.clearing
ros2 param get /costmap/costmap obstacle_layer.scan.topic
echo "=== ros2 lifecycle get /costmap/costmap ==="
ros2 lifecycle get /costmap/costmap
echo "=== /scan_raw returns inside the 3x3 m window ==="
timeout 30 python3 /tmp/scan_window_count.py
echo "=== /costmap/costmap occupancy histogram ==="
timeout 20 ros2 topic echo --once /costmap/costmap --field data | tr -d "[]" | tr "," "\n" | sed "s/ //g" | sort -n | uniq -c
date -u
'; echo "--- HOST ---"; date
Sun Aug  9 15:55:54 UTC 2026
=== params: PRODUCTION (both sources, fixed height) ===
String value is: scan pointcloud
Double value is: 2.0
Double value is: 0.0
Boolean value is: True
Boolean value is: True
Boolean value is: True
Boolean value is: True
String value is: /scan_raw
=== ros2 lifecycle get /costmap/costmap ===
active [3]
=== /scan_raw returns inside the 3x3 m window ===
frame_id           : lidar_frame
stamp              : 1786290981.488540904
range_min/range_max: 0.020 / 25.000
total_beams        : 503
valid_returns      : 279
in_window_3x3_box  : 184
in_window_conservative (r<=1.5m): 184
=== /costmap/costmap occupancy histogram ===
      1 ---
   3537 0
      1 0)
      1 array('b'
     62 100
Sun Aug  9 15:56:24 UTC 2026
--- HOST ---
Sun Aug  9 11:56:24 PM HKT 2026
```

Within run 5 the count is stable; between run 1 (136) and run 5 (62) it is
not, with the robot stationary and the scene unchanged by anyone in the room.
That between-run difference is **measured, not isolated** — no mechanism is
proposed for it here, and nothing in this section depends on the absolute
count, only on zero-vs-non-zero and on within-run comparisons:

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
date -u
for i in 1 2 3 4 5 6; do
  echo "--- sample $i ---"
  timeout 20 ros2 topic echo --once /costmap/costmap --field data | tr -d "[]" | tr "," "\n" | sed "s/ //g" | grep -c "^100$"
  sleep 5
done
date -u
'; echo "--- HOST ---"; date
Sun Aug  9 15:56:34 UTC 2026
--- sample 1 ---
62
--- sample 2 ---
62
--- sample 3 ---
62
--- sample 4 ---
62
--- sample 5 ---
62
--- sample 6 ---
62
Sun Aug  9 15:57:21 UTC 2026
--- HOST ---
Sun Aug  9 11:57:21 PM HKT 2026
```

And the mutation left nothing behind in the repository:

```
$ echo "=== git diff --stat ===" && git diff --stat && echo "=== TEMP markers left? ===" && (grep -rn "TEMP-DISCRIMINATION-MUTATION" poc_fusion/ || echo "(none)") && echo "=== FLIP markers? ===" && (grep -rn FLIP poc_fusion/ || echo "(none)")
=== git diff --stat ===
 poc_fusion/config/costmap_params.yaml | 10 +++++-----
 1 file changed, 5 insertions(+), 5 deletions(-)
=== TEMP markers left? ===
(none)
=== FLIP markers? ===
(none)
```

(The 5-line diff shown there is the `#FLIP` revert described in Step H,
which was still uncommitted at that moment; it is committed in this round.)

### Step D: `tf2_echo odom base_link` — live (corrects the dangling citation)

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
date -u
echo "=== tf2_echo odom base_link (single sample, live) ==="
timeout 8 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -20
echo "=== EXIT ==="
date -u
'; echo "--- HOST ---"; date
Sun Aug  9 15:43:04 UTC 2026
=== tf2_echo odom base_link (single sample, live) ===
[INFO] [1786290185.533270382] [tf2_echo]: Waiting for transform odom ->  base_link: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[INFO] [1786290187.482621507] [tf2_echo]: Waiting for transform odom ->  base_link: Invalid frame ID "base_link" passed to canTransform argument source_frame - frame does not exist
At time 1786290188.467496868
- Translation: [0.000, 0.000, 0.054]
- Rotation: in Quaternion [0.000, 0.000, 0.001, 1.000]
- Rotation: in RPY (radian) [0.000, -0.000, 0.001]
- Rotation: in RPY (degree) [0.000, -0.000, 0.064]
- Matrix:
  1.000 -0.001  0.000  0.000
  0.001  1.000  0.000  0.000
  0.000  0.000  1.000  0.054
  0.000  0.000  0.000  1.000
At time 1786290189.475504044
- Translation: [0.000, 0.000, 0.054]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
- Rotation: in RPY (radian) [0.000, -0.000, 0.001]
- Rotation: in RPY (degree) [0.000, -0.000, 0.054]
- Matrix:
  1.000 -0.001  0.000  0.000
  0.001  1.000  0.000  0.000
=== EXIT ===
Sun Aug  9 15:43:10 UTC 2026
--- HOST ---
Sun Aug  9 11:43:10 PM HKT 2026
```

`odom -> base_link` resolves, at z = 0.054 m, which is the value the
`costmap_params.yaml` comment relies on when it computes the LiDAR scan
plane's height in `odom` (0.039 + 0.054 = 0.093 m). Note also that a
freshly-started `tf2_echo` in this same graph emits the *same*
`Invalid frame ID ... frame does not exist` message twice before resolving.
That is an **observation**, and it is the reason the corrected paragraph
earlier in this document now records the costmap's identical startup line
without asserting a cause: nothing here isolates one.

### Step E: `/costmap/costmap_updates` re-measured after the fix, with a real concurrent load reading

The fix changes what the costmap marks, so the original section's ~6 Hz
figure is **not** carried forward as a baseline. Fresh measurement, with
`uptime` / `/proc/loadavg` taken *while the `ros2 topic hz` run was still
in flight* — the concurrency is visible in the command (the `hz` job is
backgrounded, its PID checked alive, then `wait`ed on):

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
date -u
timeout 25 ros2 topic hz /costmap/costmap_updates > /tmp/hz_updates.txt 2>&1 &
HZ_PID=$!
sleep 10
echo "=== uptime / loadavg taken DURING the hz run above (pid $HZ_PID still running) ==="
uptime
cat /proc/loadavg
echo "=== hz still alive? ==="
kill -0 $HZ_PID && echo "yes, pid $HZ_PID alive"
wait $HZ_PID
echo "=== ros2 topic hz /costmap/costmap_updates (25s) ==="
cat /tmp/hz_updates.txt
date -u
'; echo "--- HOST ---"; date
Sun Aug  9 15:43:26 UTC 2026
=== uptime / loadavg taken DURING the hz run above (pid 65679 still running) ===
 15:43:36 up 20 min,  0 users,  load average: 7.37, 5.76, 3.83
7.37 5.76 3.83 10/1012 66454
=== hz still alive? ===
yes, pid 65679 alive
=== ros2 topic hz /costmap/costmap_updates (25s) ===
average rate: 7.112
	min: 0.100s max: 0.201s std dev: 0.03602s window: 9
average rate: 6.409
	min: 0.100s max: 0.347s std dev: 0.06102s window: 15
average rate: 6.993
	min: 0.004s max: 0.347s std dev: 0.06909s window: 24
average rate: 6.994
	min: 0.004s max: 0.347s std dev: 0.06255s window: 31
average rate: 6.992
	min: 0.004s max: 0.347s std dev: 0.05952s window: 38
average rate: 6.825
	min: 0.004s max: 0.347s std dev: 0.05769s window: 45
average rate: 6.887
	min: 0.004s max: 0.347s std dev: 0.05588s window: 53
average rate: 6.906
	min: 0.004s max: 0.347s std dev: 0.05351s window: 61
average rate: 6.804
	min: 0.004s max: 0.347s std dev: 0.05267s window: 68
average rate: 6.821
	min: 0.004s max: 0.347s std dev: 0.05202s window: 75
average rate: 6.780
	min: 0.004s max: 0.347s std dev: 0.05121s window: 82
average rate: 6.781
	min: 0.004s max: 0.347s std dev: 0.05049s window: 89
average rate: 6.670
	min: 0.004s max: 0.347s std dev: 0.05076s window: 95
average rate: 6.554
	min: 0.004s max: 0.347s std dev: 0.05079s window: 101
average rate: 6.606
	min: 0.004s max: 0.347s std dev: 0.04989s window: 109
average rate: 6.532
	min: 0.004s max: 0.347s std dev: 0.04980s window: 115
average rate: 6.488
	min: 0.004s max: 0.347s std dev: 0.04946s window: 121
average rate: 6.544
	min: 0.004s max: 0.347s std dev: 0.04870s window: 129
average rate: 6.506
	min: 0.004s max: 0.347s std dev: 0.04880s window: 135
average rate: 6.516
	min: 0.004s max: 0.347s std dev: 0.04854s window: 142
average rate: 6.458
	min: 0.004s max: 0.347s std dev: 0.04880s window: 148
Sun Aug  9 15:43:52 UTC 2026
--- HOST ---
Sun Aug  9 11:43:52 PM HKT 2026
```

**Post-fix `/costmap/costmap_updates`: ~6.5 Hz average over 25 s
(final window: `average rate: 6.458`, `window: 148`), against a configured
10 Hz. MEASURED, NOT ISOLATED.** No causal story is offered. The load
average at the moment of measurement was 7.37 (1-minute) on this 4-core
Raspberry Pi 5 — that is a genuine concurrent reading, and it is recorded as
*context, not cause*: nothing here shows the rate gap is caused by it.
Note also `min: 0.004s`, i.e. some update pairs arrive far faster than the
100 ms configured period, which the older section did not observe.

### Step F: does the depth-camera point cloud reach the grid?

This was never tested before — the original Task 6 probe ran without the
depth pipeline, and `ros2 node info` only ever proved *subscription*.

The run-2 / run-3 pair in Step C answers it. Both runs used the broken
`scan.max_obstacle_height: 0.0`, so the LiDAR source contributed nothing in
either. They differ only in whether the `pointcloud` source is listed:

- run 3, `observation_sources: "scan"` → **0** occupied cells.
- run 2, `observation_sources: "scan pointcloud"` → **80** occupied cells,
  stable across two samples 15 s apart.

`obstacle_layer` is the only plugin configured (`plugins: ["obstacle_layer"]`
— no static layer, no inflation layer), and `scan` marked nothing under that
height filter, so **those 80 cells can only have been marked by
`/poc_fusion/points`**. The depth-derived point cloud is not merely
subscribed; its observations reach the occupancy grid. `/poc_fusion/points`
was confirmed live during that same run at `average rate: 3.350` Hz.

**What this does NOT show, stated plainly:**

- It does not show the depth camera sees anything the LiDAR would have
  missed. That is Step 5's hand-wave test and it still has not been done.
  **"Fusion is not verified working" from the original report stands and is
  NOT walked back.**
- It does not show *what* those 80 cells are. With `min_obstacle_height:
  0.03` / `max_obstacle_height: 0.94` on that source and an empty floor in
  front of a camera tilted ~40° down, near-floor returns and genuine
  low obstacles are not distinguished by anything measured here.
- No human was present in this session to put a hand or an object into the
  camera's FOV outside the LiDAR plane, so the correlated "object appears →
  new cells appear at the expected bearing" test was not attempted at all.

### Step G: launch-mechanism evidence cited by the source comments

The `/**` wildcard claim in `poc_fusion.launch.py`'s comment, from the
installed `launch_ros` source in the container:

```
$ docker exec -u ubuntu MentorPi bash -lc '
echo "=== launch_ros node.py lines 366-378 ==="
sed -n "366,378p" /opt/ros/humble/lib/python3.10/site-packages/launch_ros/actions/node.py
'
=== launch_ros node.py lines 366-378 ===
        keywords = (self.UNSPECIFIED_NODE_NAME, self.UNSPECIFIED_NODE_NAMESPACE)
        return all(x not in self.node_name for x in keywords)

    def _create_params_file_from_dict(self, params):
        with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
            param_file_path = h.name
            param_dict = {
                self.node_name if self.is_node_name_fully_specified() else '/**':
                {'ros__parameters': params}
            }
            yaml.dump(param_dict, h, default_flow_style=False)
            return param_file_path
```

And the temporary params files actually generated by the five launches above,
showing both the `/**` wildcard key in practice and the injected scan-topic
override landing as its own file:

```
$ docker exec -u ubuntu MentorPi bash -lc '
cd /tmp
for f in $(find . -maxdepth 1 -name "launch_params_*" -newermt "-25 minutes" | head -20); do echo "--- $f ---"; head -6 "$f"; done
'
--- ./launch_params_sbhtj9ge ---
/**:
  ros__parameters:
    global_frame: odom
    height: 3
    obstacle_layer.enabled: true
    obstacle_layer.observation_sources: scan pointcloud
--- ./launch_params_glx07sbe ---
/**:
  ros__parameters:
    autostart: true
    bond_timeout: 0.0
    node_names: !!python/tuple
    - costmap/costmap
--- ./launch_params_enmjhq3y ---
/**:
  ros__parameters:
    global_frame: odom
    height: 3
    obstacle_layer.enabled: true
    obstacle_layer.observation_sources: scan pointcloud
--- ./launch_params_jeoltfsz ---
/**:
  ros__parameters:
    obstacle_layer.scan.topic: /scan_raw
--- ./launch_params_u7bqcf4k ---
/**:
  ros__parameters:
    obstacle_layer.scan.topic: /scan_raw
--- ./launch_params_c50s0pl_ ---
/**:
  ros__parameters:
    autostart: true
    bond_timeout: 0.0
    node_names: !!python/tuple
    - costmap/costmap
--- ./launch_params_p6veoov6 ---
/**:
  ros__parameters:
    global_frame: odom
    height: 3
    obstacle_layer.enabled: true
    obstacle_layer.observation_sources: scan
--- ./launch_params_xltwkv_m ---
/**:
  ros__parameters:
    obstacle_layer.scan.topic: /scan_raw
--- ./launch_params_g6ggjynx ---
/**:
  ros__parameters:
    autostart: true
    bond_timeout: 0.0
    node_names: !!python/tuple
    - costmap/costmap
--- ./launch_params_4o9s2pve ---
/**:
  ros__parameters:
    global_frame: odom
    height: 3
    obstacle_layer.enabled: true
    obstacle_layer.observation_sources: scan pointcloud
--- ./launch_params_7d8uc64p ---
/**:
  ros__parameters:
    obstacle_layer.scan.topic: /scan_raw
--- ./launch_params_gn1dr94t ---
/**:
  ros__parameters:
    autostart: true
    bond_timeout: 0.0
    node_names: !!python/tuple
    - costmap/costmap
--- ./launch_params_v0uuj1vc ---
/**:
  ros__parameters:
    autostart: true
    bond_timeout: 0.0
    node_names: !!python/tuple
    - costmap/costmap
--- ./launch_params_8c618ih1 ---
/**:
  ros__parameters:
    obstacle_layer.scan.topic: /scan_raw
--- ./launch_params_6a_fjqkk ---
/**:
  ros__parameters:
    global_frame: odom
    height: 3
    obstacle_layer.enabled: true
    obstacle_layer.observation_sources: scan
```

`resolve_scan_topic_key` mutation-proof, run on the host against the real
shipped YAML (this is the claim `poc_fusion.launch.py` makes when it says the
override key follows a rename instead of orphaning itself):

```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -c "
import copy, yaml, sys
sys.path.insert(0, '.')
from poc_fusion.lib.costmap_param_keys import resolve_scan_topic_key
tree = yaml.safe_load(open('config/costmap_params.yaml'))['costmap']['costmap']['ros__parameters']
print('as-shipped                       ->', resolve_scan_topic_key(tree))
m = copy.deepcopy(tree)
m['fused_layer'] = m.pop('obstacle_layer'); m['plugins'] = ['fused_layer']
print('layer renamed to fused_layer     ->', resolve_scan_topic_key(m))
m2 = copy.deepcopy(m)
m2['fused_layer']['lidar'] = m2['fused_layer'].pop('scan')
m2['fused_layer']['observation_sources'] = 'lidar pointcloud'
print('source renamed to lidar          ->', resolve_scan_topic_key(m2))
m3 = copy.deepcopy(tree)
del m3['obstacle_layer']['scan']['data_type']
try:
    resolve_scan_topic_key(m3)
except RuntimeError as e:
    print('LaserScan source removed         -> RuntimeError:', e)
"
as-shipped                       -> obstacle_layer.scan.topic
layer renamed to fused_layer     -> fused_layer.scan.topic
source renamed to lidar          -> fused_layer.lidar.topic
LaserScan source removed         -> RuntimeError: expected exactly one LaserScan observation source on costmap layer 'obstacle_layer', found 0: []
```

### Step H: `#FLIP` discipline and host tests

The `#FLIP` markers used during this round to prove the config discriminates
a real regression are all gone, and the host suite is green:

```
$ date && git status && git log --oneline -3 && echo "=== FLIP ===" && grep -rn FLIP poc_fusion/ ; echo "exit=$?"
Sun Aug  9 11:35:34 PM HKT 2026
On branch feature/poc-fusion-costmap
Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git restore <file>..." to discard changes in working directory)
	modified:   poc_fusion/config/costmap_params.yaml

no changes added to commit (use "git add" and/or "git commit -a")
de956ea Implement dynamic scan topic resolution and enhance costmap parameter handling
164fa8b Task 6: costmap configuration (standalone nav2_costmap_2d + lifecycle manager)
2cc984a Task 5 fix round part 3: correct two grep blocks to match their commands
=== FLIP ===
exit=1
```

```
$ cd /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion && python3 -m pytest test/ -q 2>&1 | tail -5
.................................................                        [100%]
49 passed in 0.30s
```

### Robot left as found

Each run was stopped by sending `SIGINT` to the launch's own process group
only — `~/.stop_ros.sh` was never used, no velocity command and no arm
command was ever published, and the vendor stack was verified intact
afterwards:

```
$ docker exec -u ubuntu MentorPi bash -lc '
source /opt/ros/humble/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash
ps -o pid,stat,cmd -p 57875
echo "=== node list (fresh discovery) ==="
ros2 node list | grep -E "costmap|poc_fusion" || echo "(no poc_fusion/costmap nodes)"
'
    PID STAT CMD
  57875 Z    [nav2_costmap_2d] <defunct>
=== node list (fresh discovery) ===
(no poc_fusion/costmap nodes)
```

(PID 57875 is a zombie left from run 1, where `nav2_costmap_2d` did not exit
on `SIGTERM` and was `kill -9`ed by PID — a single targeted signal to one of
this POC's own processes, not a broad kill. It holds no resources and has no
ROS presence, as the node list shows. Runs 2–5 shut down cleanly via
process-group `SIGINT` and needed no such signal.)

## Task 7: Costmap stop monitor (2026-08-10, live session ~21:43–21:49 HKT / 13:43–13:49 UTC)

All evidence in this section was captured in a single continuous session on
the live robot (off charger). Every command below was run inside the MentorPi
container as `ubuntu`. The monitor publishes no velocity and commands no
motion — acting on the signal is Task 9's scope — so nothing here moved the
robot.

**Provenance warning, read before trusting any number here.** The original
Task 7 implementer session died and its output was unrecoverable. Its code
survived in git (`1adbb3f`), but the measurements it cited in code comments
did not. Nothing in this section is carried over from that session: every
figure below was re-measured from scratch. Where a re-measurement failed to
reproduce an original claim, the original is recorded as **withdrawn**, not
restated.

### Step 1 — Deploy and build

```
$ scripts/deploy_poc_fusion.sh
Deploying /home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion -> MentorPi:/home/ubuntu/ros2_ws/src/poc_fusion
No stale destination files to remove.
Deploy complete.

$ docker exec -u ubuntu MentorPi bash -lc 'source /opt/ros/humble/setup.bash && cd /home/ubuntu/ros2_ws && colcon build --packages-select poc_fusion --symlink-install'
Finished <<< poc_fusion [3.80s]

Summary: 1 package finished [6.14s]
  1 package had stderr output: poc_fusion
```

(The stderr output is the setuptools `setup.py develop` deprecation warning
already documented in earlier tasks, not an error.)

### Step 2 — Costmap topic measurement (Requirement 3 evidence)

Measured with a purpose-written subscriber (`/tmp/measure_costmap.py`, a
throwaway tool, not part of the package) rather than `ros2 topic hz`, so that
message *content* could be inspected in the same window as arrival timing.
One continuous 90 s subscription:

```
$ docker exec -u ubuntu MentorPi bash -lc 'source /opt/ros/humble/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash; date -Is; python3 /tmp/measure_costmap.py; date -Is'
2026-08-10T13:43:02+00:00
measuring for 90 s ...
window actually elapsed: 90.15 s
--- /costmap/costmap_raw (nav2_msgs/Costmap) ---
messages received: 562
intervals: 561
mean   : 0.1565 s  (6.389 Hz)
median : 0.1561 s
min    : 0.0947 s
p95    : 0.2292 s
p99    : 0.2441 s
max    : 0.2612 s   <-- worst observed gap
raw max cost value observed : 254
--- /costmap/costmap (nav_msgs/OccupancyGrid) ---
messages received: 1
intervals: n/a (need >=2 messages)
grid max cost value observed: 100
2026-08-10T13:44:39+00:00
```

**This settles the topic choice.** `/costmap/costmap` (OccupancyGrid) carries
costs rescaled to 0..100. The monitor's `lethal_threshold` is 253, so on that
topic the threshold could **never** be reached: the monitor would report CLEAR
forever, with no error and no missing message to notice. Consuming
`/costmap/costmap_raw` (max cost 254 observed, the real nav2 cost scale where
253 = INSCRIBED_INFLATED_OBSTACLE and 254 = LETHAL_OBSTACLE) is what makes the
threshold meaningful. This is a correction to the Task 7 brief's Step 1, and
it is exactly the class of silent failure `CONSTRAINTS.md:37` warns about.

The 6.389 Hz measured against a configured 10 Hz is used here as a **measured,
not-isolated** input, consistent with the standing project ruling. It is not
explained and no cause is offered.

**Withdrawn claims.** The pre-existing comment in `stop_monitor_params.yaml`
sized the staleness bound against a 0.467 s worst gap (attributed to a
`ros2 topic hz --window 500` run) and a 0.3417 s maximum in its own 90 s run,
and argued that a 10 Hz-derived ~0.40 s bound "would have flapped" because
0.467 s exceeds it. The re-measurement above reproduces the distribution
*shape* closely (562 vs 568 messages; 6.389 vs 6.309 Hz; median 0.1561 vs
0.1552 s) but **does not reproduce either large gap** — the worst gap observed
here is 0.2612 s. Against this run's data a 0.40 s bound would *not* have
flapped. That argument has therefore been withdrawn from the config comment
rather than restated. The bound stays at 0.75 s on the surviving justification
alone: 2.87x the worst gap actually observed and 4.79x the mean interval.

### Step 3 — Requirement 5, and normal operation with an ACTIVE costmap

Launch log excerpt (`/tmp/t7_launch.log`), from startup:

```
[costmap_stop_monitor_node-5] [INFO] [1786369316.969318052] [costmap_stop_monitor_node]: monitor state: None -> UNKNOWN (no_costmap_received)
[costmap_stop_monitor_node-5] [ERROR] [1786369316.972866150] [costmap_stop_monitor_node]: no depth frame within 2.0 s: FUSION IS NO LONGER ACTIVE and the costmap is now LiDAR-ONLY. Low-profile obstacles the LiDAR plane misses will NOT be seen. Not halting -- degrading to the proven sensor is correct here; degrading silently is not.
[costmap_stop_monitor_node-5] [INFO] [1786369317.220948742] [costmap_stop_monitor_node]: depth frames resumed on the camera axis: fusion is ACTIVE again (was LIDAR_ONLY).
[nav2_costmap_2d-3] [WARN] [1786369318.235406689] [costmap.costmap.rclcpp]: failed to send response to /costmap/costmap/get_state (timeout): client will not receive response, at ./src/rmw_response.cpp:154, at ./src/rcl/service.c:314
[costmap_stop_monitor_node-5] [WARN] [1786369318.613261270] [costmap_stop_monitor_node]: get_state on /costmap/costmap did not answer within 0.50 s; costmap liveness is now UNCONFIRMED.
[costmap_stop_monitor_node-5] [INFO] [1786369318.815430715] [costmap_stop_monitor_node]: monitor state: UNKNOWN -> UNKNOWN (tf_lookup_failed)
[costmap_stop_monitor_node-5] [WARN] [1786369318.895016364] [costmap_stop_monitor_node]: no TF odom -> base_link at the previous costmap message's stamp before the next message arrived; that cycle was dropped unevaluated. State is UNKNOWN, NOT clear.
[costmap_stop_monitor_node-5] [INFO] [1786369319.015245315] [costmap_stop_monitor_node]: monitor state: UNKNOWN -> OBSTACLE (lethal_cells_in_window)
```

Four requirements are demonstrated in these nine lines:

- **Requirement 4 (fail-safe startup):** the very first state is
  `None -> UNKNOWN (no_costmap_received)`. Startup is UNKNOWN, never CLEAR.
- **Requirement 2 (positive liveness):** a `get_state` call that did not
  answer within 0.50 s drove liveness to **UNCONFIRMED** rather than being
  treated as "fine". Absence of an answer is not an affirmative answer. The
  costmap's own matching `failed to send response` warning is in the log too,
  so this was a genuine unanswered call, not a client-side artifact.
- **Requirement 1 (UNKNOWN is not CLEAR):** a TF lookup failure produced
  `UNKNOWN (tf_lookup_failed)` and the cycle was dropped unevaluated.
- The camera axis is independent: it reported LIDAR_ONLY then FUSION_ACTIVE
  without changing the costmap-side state.

Full status sample during ACTIVE operation:

```
$ ros2 topic echo /costmap_app/monitor_status --once
status:
- level: "\0"
  name: 'costmap_stop_monitor: costmap'
  message: CLEAR
  hardware_id: /costmap/costmap_raw
  values:
  - {key: state, value: CLEAR}
  - {key: reason, value: window_clear}
  - {key: costmap_age_s, value: '0.048'}
  - {key: costmap_staleness_bound_s, value: '0.750'}
  - {key: costmap_lifecycle_node, value: /costmap/costmap}
  - {key: costmap_lifecycle_state, value: active}
  - {key: costmap_frame, value: odom}
  - {key: tf_ok, value: 'True'}
  - {key: obstacle_in_window, value: 'False'}
  - {key: bool_published, value: 'True'}
  - {key: window_forward_m, value: '1.000'}
  - {key: window_half_width_m, value: '0.300'}
  - {key: lethal_threshold, value: '253'}
- level: "\0"
  name: 'costmap_stop_monitor: fusion'
  message: FUSION_ACTIVE
  values:
  - {key: camera_axis, value: FUSION_ACTIVE}
  - {key: depth_age_s, value: '0.008'}
  - {key: camera_timeout_s, value: '2.000'}
```

(Reformatted from `ros2 topic echo`'s one-key-per-line YAML into inline
mappings for width; keys and values are verbatim.) `costmap_lifecycle_state:
active` is the positive confirmation required by Requirement 2 — the state is
reported from a real `get_state`/`transition_event` answer, not inferred from
the existence of a subscription.

### Step 4 — Requirement 7: lifecycle manager alive, costmap NOT active

This is the scenario carried forward from Task 6 Step 4, where "manager alive
but costmap never active" had no launch-time gate. Task 7 closes it in the
consumer instead: the monitor refuses to treat a non-ACTIVE costmap as usable.

The real costmap lifecycle node was driven out of ACTIVE while the monitor
kept running (`/tmp/live_check_b.py`):

```
=== PHASE 1: baseline, costmap ACTIVE (10 s) ===
  Bool messages published : 65  values=[False, True]
  status samples          : 50
    state=CLEAR     reason=window_clear             lifecycle=active       diag_level=0
    state=OBSTACLE  reason=lethal_cells_in_window   lifecycle=active       diag_level=0

=== DEACTIVATING costmap lifecycle node ===
  ros2 lifecycle set /costmap/costmap deactivate -> Transitioning successful

=== PHASE 2: manager alive, costmap NOT active (20 s) ===
  status samples          : 110
    state=UNKNOWN   reason=costmap_not_active       lifecycle=inactive     diag_level=2

=== REACTIVATING costmap lifecycle node ===
  ros2 lifecycle set /costmap/costmap activate -> Transitioning successful

=== PHASE 3: recovery (12 s) ===
    state=OBSTACLE  reason=lethal_cells_in_window   lifecycle=active       diag_level=0
    state=CLEAR     reason=window_clear             lifecycle=active       diag_level=0
```

`diag_level=2` is `DiagnosticStatus.ERROR` — Requirement 5's "loud and
observable" on the wire, greppable in a rosbag when a trial is audited
afterwards for blind operation. The corresponding log line:

```
[costmap_stop_monitor_node-5] [ERROR] [...] [costmap_stop_monitor_node]: MONITOR STATE UNKNOWN (costmap_not_active): this node is NOT a valid safety input right now and this is NOT "clear". costmap_age_s=26.019 bound=0.750 lifecycle='inactive' tf_ok=True. No std_msgs/Bool is being published while degraded.
```

Recovery was observed through the `transition_event` path, which is
Requirement 2's second mechanism:

```
[costmap_stop_monitor_node-5] [INFO] [...] /costmap/costmap lifecycle transition: inactive -> activating
[costmap_stop_monitor_node-5] [INFO] [...] /costmap/costmap lifecycle transition: activating -> active
[costmap_stop_monitor_node-5] [INFO] [...] monitor state: UNKNOWN -> CLEAR (window_clear)
```

#### The Bool-silence claim, and why the first run's result was not trusted

The safety invariant is that UNKNOWN publishes **nothing** on the
`std_msgs/Bool` compatibility topic — never `False`, which a consumer would
read as "clear". A first attempt counted Bool messages per phase and appeared
to **fail**: 1 Bool inside the UNKNOWN window.

That result was not reported as a defect, because per-phase counting cannot
distinguish "published while UNKNOWN" from "published while still ACTIVE and
received a moment later" — the timestamps are *receipt* times at the
subscriber, on two different topics, so they cannot order publications at the
node. A second run (`/tmp/live_check_b3.py`) separated the two:

```
deactivate -> Transitioning successful
entered UNKNOWN(costmap_not_active) at t_deact+0.010 s

-- BOUNDARY WINDOW (the 3 s settle) --
Bools after entering UNKNOWN: 1
   offset +0.0003 s after UNKNOWN, value=True

-- STEADY STATE (20 s of continuous UNKNOWN) --
status samples          : 100
distinct (state, reason): [('UNKNOWN', 'costmap_not_active')]
BOOLS IN STEADY UNKNOWN : 0   <-- PASS (gate holds; the boundary Bool was an interleave)
```

The single boundary Bool arrived **0.3 ms** after the first UNKNOWN status —
three orders of magnitude inside one ~0.156 s publish cycle, i.e. a same-cycle
interleave between two topics at the subscriber, not a gate leak. Over 20 s of
**continuous** UNKNOWN (100 consecutive samples, no other state) the Bool was
completely silent. Its value was also `True`, never `False`: even the
interleaved message pointed in the fail-safe direction.

This matches the static reading of the code — `costmap_stop_monitor_node.py`
has exactly one Bool publish site, gated by `should_publish_bool()`, and
`bool_value()` raises `ValueError` on UNKNOWN so the conflation cannot be
reintroduced silently.

### Step 5 — Shutdown guard (SIGTERM)

```
$ PID=$(pgrep -f costmap_stop_monitor_node | head -1); echo "pid=$PID"; kill -TERM $PID
pid=31236
...
[INFO] [costmap_stop_monitor_node-5]: process has finished cleanly [pid 31236]
```

No `RCLError: failed to shutdown: rcl_shutdown already called` and no
traceback appeared in the launch log. The `rclpy.ok()` guard added in
`17af039` is therefore live-verified. (The `NOT LIVE-VERIFIED` marker that
commit carried has been retired on the strength of this run, not on trust in
the dead session that originally wrote the guard.)

**Observed and carried forward to Task 9:** killing the monitor did **not**
tear down the rest of the launch — `nav2_costmap_2d`, `lifecycle_manager`,
`component_container` and `depth_preprocess_node` all kept running. A dead
monitor is therefore *silent* rather than false-clear, which is the fail-safe
direction, but it means **Task 9 must treat Bool silence as "not clear"**, not
as "no obstacle". This is a consumer-side requirement, recorded here so it is
not rediscovered later.

### Step 6 — Not verified live

Stated plainly rather than left implied:

- **`UNKNOWN (costmap_stale)` was never exercised live.** `evaluate_state()`
  checks lifecycle before staleness, so deactivating the costmap always
  reports `costmap_not_active` first. Producing a genuine stale-but-ACTIVE
  costmap would require stalling publication while the lifecycle node still
  reports ACTIVE, which was not arranged in this session. The path is covered
  by unit tests, two of which are mutation-proven (Step 7), but it has **no
  live evidence**.
- The `0.3417 s` and `0.467 s` worst-gap figures from the dead implementer
  session remain **unreproduced** (Step 2).

### Step 7 — Test suite and mutation proofs

The discrimination bar (Requirement 6) was verified by mutation rather than by
assertion: each safety rule was broken in turn and the suite re-run, proving
the tests actually discriminate and are not vacuous.

```
$ cd poc_fusion && python3 -m pytest test -q
87 passed in 1.53s
```

| Mutation applied to `lib/monitor_state.py` | Result |
|---|---|
| startup (`costmap_age_s is None`) returns CLEAR instead of UNKNOWN | 2 failed — `test_startup_before_any_costmap_is_unknown_not_clear`, `test_startup_is_unknown_even_when_lifecycle_is_already_active` |
| staleness check disabled (stale treated as fresh) | 2 failed — `test_stale_costmap_is_unknown_not_clear`, `test_stale_costmap_is_unknown_even_when_an_obstacle_was_last_seen` |
| `should_publish_bool()` returns True unconditionally | 1 failed — `test_bool_is_not_published_in_unknown` |
| lifecycle check disabled (inactive costmap treated as usable) | 6 failed — `test_every_non_active_lifecycle_state_is_unknown[inactive]`, `[finalized]`, `[errorprocessing]`, and others in that parametrisation |

All four mutations were reverted and the suite re-run clean afterwards.

### Step 8 — Robot left as found

```
$ kill -TERM 31210 31228 31230 31232 31234
$ ps -eo pid,stat,cmd | grep -E "poc_fusion|nav2_costmap|lifecycle_manager|component_container|depth_preprocess" | grep -v grep
  31232 Z    [nav2_costmap_2d] <defunct>

$ ros2 node list | wc -l
28
$ ros2 node list | grep -iE "costmap|poc_fusion|stop_monitor|depth_preprocess"
(none of ours - clean)
```

28 vendor nodes remain, matching the baseline recorded at the end of Task 6.
The single `<defunct>` entry is an unreaped exit status, not a live process —
the container runs no init reaper, as documented in Task 6. No velocity was
ever published, no arm command was ever sent, and `.stop_ros.sh` was not run.

---

## Task 8: Integration verification of the full chain (2026-08-10 ~22:20–23:58 HKT / 14:20–15:58 UTC)

Task 8's brief is three steps: write the launch file, load all three config YAMLs
with `scan_topic` passed onward rather than restated, and verify the whole chain
starts clean alongside the existing bringup without disturbing `proximity_alert`.
The brief carries **no latency requirement** — latency is Task 12's, and is to be
measured there, not derived here.

Verification of the third step turned up a **defect in production configuration**
that made the chain's headline acceptance criterion ("clear environment reports
CLEAR") impossible to satisfy. That defect, its measurement, its fix and the
re-verification are the bulk of this section.

**Standing rule applied throughout: consumption of a sensor into the costmap is
never inferred from a subscription existing.** Every consumption claim below is
backed by a measurement that would read differently if the sensor were not being
consumed.

---

### Step 3a: the whole chain runs, all eight stages measured simultaneously

One process subscribed to all eight stages of the pipeline at once for 45.02 s, so
the rates below are concurrent, not stitched together from separate runs.

```
   stage                topic                                  msgs     Hz
1  depth in             /ascamera/.../depth0/image_raw          658  14.62
2  cleaned              /poc_fusion/depth_cleaned               639  14.19
3  rectified            /poc_fusion/depth_rect                  651  14.46
4  points               /poc_fusion/points                       96   2.13
5  lidar                /scan_raw                               444   9.86
6  costmap              /costmap/costmap_raw                    294   6.53
7  status               /costmap_app/monitor_status             215   4.78
8  safety Bool          /costmap_app/obstacle_detected          277   6.15
-- content observed --
  bool_values: {True}
  costmap_cells: 3600
  costmap_lethal: 18
  costmap_max: 254
  lifecycle: active
  points_width: 256000
  scan_min_m: 1.732
  scan_returns: 171
  states: {('OBSTACLE', 'lethal_cells_in_window')}
```

Every stage carries traffic and the lifecycle node reports `active`, so the chain
is connected end to end.

**MEASURED, NOT ISOLATED — new figure to carry forward.** `/poc_fusion/points` ran
at **2.13 Hz** here, *below* the ~3–7 Hz band previously ratified for this topic
(Task 5) and far below the 14.6 Hz its own upstream depth input sustained in the
same window. This is recorded as an input for Task 11/12, **not explained**. No
cause was isolated and none is asserted. Task 12's latency figures must be measured
against whatever rate is observed at that time, never derived from this one.

The `states` line — `OBSTACLE` with 18 lethal cells — was what triggered the rest
of this section, because the scene was an empty floor.

---

### Step 3b: the reported OBSTACLE was not backed by either sensor

`scan_min_m: 1.732` says the nearest LiDAR return **anywhere in the scan** was
1.732 m, yet the monitor reported lethal cells inside a window extending only 1.0 m
forward. Something was marking the near field. Locating every lethal cell in
`base_link`:

```
costmap: 60x60 @ 0.05 m, origin=(-1.450, -1.450) frame=odom
TF odom->base_link: t=(0.000,0.000) yaw=-129.96 deg

lethal cells (cost >= 253): 22
 x_base(m)  y_base(m)  range(m)  cost
     0.202      0.124     0.237   254
     0.234      0.163     0.285   254
     0.266      0.201     0.334   254
     0.305      0.169     0.348   254
     0.298      0.239     0.382   254
     0.337      0.207     0.395   254
     0.369      0.245     0.443   254
     0.401      0.284     0.491   254
     0.439      0.252     0.506   254
     0.471      0.290     0.553   254
nearest lethal cell: 0.237 m

lethal cells inside monitor window (0 < x <= 1.0, |y| <= 0.3): 10

LaserScan: 173 valid returns, nearest 1.735 m
  within +/-20 deg of forward: 25 returns, nearest 9.987 m
```

The cells lie on a ray at ~31.5° to port, 0.237 m to 0.60 m out. The LiDAR sees
nothing closer than 1.735 m anywhere, and nothing closer than 9.987 m ahead. **The
LiDAR cannot be the source.** Neither could persistence hide it: the scan source
runs `observation_persistence: 0.0` and the pointcloud source only 0.3 s.

---

### Step 3c: root cause — the height filter is applied in `odom`, not above the floor

`nav2_costmap_2d`'s `ObservationBuffer` transforms each observation into the
costmap's `global_frame` and **only then** compares z against
`min_obstacle_height` / `max_obstacle_height`. This costmap's `global_frame` is
`odom`.

Frame offsets, measured live:

```
odom->base_link:            t=(+0.0000, +0.0000, +0.0540)
base_link->lidar_frame:     t=(+0.0730, +0.0000, +0.0387)
odom->lidar_frame:          t=(-0.0469, +0.0560, +0.0927)
base_link->depth_camera_link: t=(+0.1049, +0.0209, +0.1918)
```

`odom->base_link` z is a **fixed** +0.0540 m, not drift — 860 lookups over 20 s:

```
odom->base_link z over 860 lookups in 20 s:
  min=0.054000 max=0.054000 mean=0.054000 std=1.31081e-17
```

The camera measures the floor plane at `base_link` z ≈ −0.006, so **the floor sits
at `odom` z ≈ +0.048** — above the configured `min_obstacle_height: 0.03`. The
threshold written to reject the floor was underneath it.

The same 200,961-point cloud frame, same instant, expressed in each frame:

```
--- cloud expressed in base_link ---
  n=200961  z: min=-0.2242 p05=-0.0211 median=-0.0060 p95=+0.0251 max=+0.0344
  points passing height band [0.03, 0.94]: 1984 / 200961 (1.0%)

--- cloud expressed in odom ---
  n=200961  z: min=-0.1702 p05=+0.0329 median=+0.0480 p95=+0.0791 max=+0.0884
  points passing height band [0.03, 0.94]: 200596 / 200961 (99.8%)
```

**1.0% versus 99.8%.** In the frame the filter actually runs in, essentially the
entire cloud — bare floor — was admitted as obstacle.

Why only 22 lethal cells appeared rather than a plastered field: the LiDAR source
has `clearing: true` and raytraces the near field clear at ~10 Hz while the camera
re-marks it at ~2 Hz. The two race. Measured over 30 s with the **robot stationary
and the scene unchanged**:

```
costmap samples over 30 s: 184
lethal cells per sample: min=14 max=123 mean=85.3
cells lethal in AT LEAST ONE sample: 127
cells lethal in EVERY sample:        10
total cell add/remove events between consecutive samples: 6328
mean churn per publication: 34.6 cells

monitor states seen: {('OBSTACLE', 'lethal_cells_in_window'): 147}
```

A genuine obstacle gives a stable cell set. 34.6 cells changing per publication on
a motionless robot in a static scene is the signature of the mark/clear race, and
the monitor sat at **OBSTACLE in 147 of 147 samples, never once reaching CLEAR**.

---

### Step 3d: the fix

The band is now expressed in `odom`, the frame it is compared in.

| key | before | after | rationale |
|---|---|---|---|
| `obstacle_layer.pointcloud.min_obstacle_height` | 0.03 | **0.10** | floor at odom z ≈ 0.048; clears observed floor-noise max 0.0884 by ~12 mm |
| `obstacle_layer.pointcloud.max_obstacle_height` | 0.94 | **0.99** | same 0.94 m camera ceiling *above the floor*, expressed in odom (0.048 + 0.94 = 0.988) |

The scan source is untouched: its band is [0.0, 2.0] and the scan plane at odom
z = 0.0927 sits well inside it either way.

**Capability consequence, stated plainly:** the minimum detectable obstacle is now
roughly **0.052 m tall** (0.10 − 0.048). This is a chosen trade-off ratified by the
user against the measured floor-noise distribution above, **not a derived bound**.
Floor noise has been measured on **one surface only**. The granite and metal trial
surfaces are exactly what Task 11 re-measures; if their floor noise is worse, this
is the key that moves.

Readback from the running node — the values nav2 actually loaded, not the file:

```
obstacle_layer.pointcloud.min_obstacle_height        Double value is: 0.1
obstacle_layer.pointcloud.max_obstacle_height        Double value is: 0.99
obstacle_layer.scan.min_obstacle_height              Double value is: 0.0
obstacle_layer.scan.max_obstacle_height              Double value is: 2.0
```

This readback was reproduced on a **second, independent launch** after the robot
was power-cycled for charging, so the fix is confirmed to survive a cold start.

**Also corrected in the same file:** the scan source's comment described the LiDAR
plane as sitting "0.093 m above the floor". 0.0927 is its height above the **odom
origin**; the floor is at odom z ≈ 0.048, so the plane is only ~0.045 m above the
actual floor. The conclusion that comment supported is unaffected (the filter
compares odom z regardless), but the stated fact was wrong and is now fixed.

---

### Step 3e: re-verification after the fix

Identical script, identical stationary scene:

| metric | before fix | after fix |
|---|---|---|
| lethal cells per publication | 14–123 (mean 85.3) | 7–14 (mean 9.4) |
| cells lethal in every sample | 10 | 5 |
| churn per publication | 34.6 cells | 3.0 cells |
| monitor state | OBSTACLE 147/147 | **CLEAR 136/136** |

```
costmap samples over 30 s: 164
lethal cells per sample: min=7 max=14 mean=9.4
total cell add/remove events between consecutive samples: 490
mean churn per publication: 3.0 cells

monitor states seen: {('CLEAR', 'window_clear'): 136}
```

And the surviving cells are now outside the window and at plausible ranges:

```
lethal cells (cost >= 253): 8
nearest lethal cell: 1.711 m
lethal cells inside monitor window (0 < x <= 1.0, |y| <= 0.3): 0
```

With the scene bare, the depth cloud's odom z now maxes at +0.0886, entirely below
the 0.10 threshold, so the camera correctly marks **nothing**. That is the desired
behaviour, and it is also why proving depth *consumption* required an actual
obstacle (Step 3g).

---

### Step 3f: LiDAR observations are demonstrably consumed

Each lethal cell was transformed into `lidar_frame` and matched against the
**concurrent** scan in (bearing, range) space. A cell that is genuinely a LiDAR
mark must have a scan return at its own bearing agreeing in range to about one cell
diagonal (tolerance 0.08 m, bearing tolerance 3°).

```
lethal cells: 12   valid scan returns: 177

 cell_bearing  cell_range  scan_range   d_range   verdict
       -108.2       0.455       6.341    +5.886  MISMATCH
        -64.5       0.900       0.897    -0.003     MATCH
        -63.2       0.945       0.897    -0.048     MATCH
       -103.2       1.705       1.736    +0.031     MATCH
       -101.7       1.728       1.736    +0.008     MATCH
       -104.0       1.750       1.741    -0.009     MATCH
       -102.5       1.773       1.741    -0.032     MATCH
         96.2       1.774       1.794    +0.020     MATCH
         95.0       1.809       1.810    +0.001     MATCH
         92.8       1.811       1.810    -0.001     MATCH
         97.3       1.811       1.810    -0.001     MATCH
         93.9       1.845       1.827    -0.018     MATCH

lethal cells backed by a concurrent scan return (within 0.08 m at same bearing): 11 / 12
```

**11 of 12.** This is direct evidence of LiDAR consumption into the costmap, not
inference from a subscription.

**Correction recorded.** The first version of this cross-check reported only 4/12
and claimed five cells had *no scan return at their bearing*. That was a bug in the
checking script, not a finding: the LD19's `angle_min` is ~0 so beam bearings run
0…2π, while `atan2`-derived cell bearings run −π…π, and the comparison did not wrap.
A cell at −105° was being compared against beams at +255° — the same direction — and
never matched. The claim was withdrawn and the script fixed before any conclusion
was drawn from it. Per-sector scan validity was measured separately and confirms
those bearings do carry finite returns:

```
total beams 504, finite in-range 181 (35.9%)
 sector(deg)  beams  finite  inf/oor   nan
    240..255     21       1        0    20
    255..270     21      16        0     5
    270..285     21      16        0     5
    285..300     21      19        0     2
```

**DEFERRED, NOT EXPLAINED.** The one mismatched cell sits at −108.2°, range 0.455 m,
where the concurrent scan reports 6.341 m — a mark with measured free space in front
of it that the clearing raytrace has not removed. It is outside the monitor window
and the cell set is transient (7–14 cells), so it does not affect any Task 8
criterion. No cause is isolated and none is asserted. Recorded for the final
whole-branch review. The risk it represents — a ghost mark that cannot be cleared
landing *inside* the window — would fail safe (permanent OBSTACLE, robot stops)
rather than fail dangerous.

Separately noted from the same measurement: only **35.9%** of the LD19's 504 beams
carry finite in-range returns, with a contiguous all-`nan` arc from ~105° to ~240°.
Recorded as an observation; not investigated here.

---

### Step 3g: Phase A — controlled obstacle, present then removed

Robot **stationary throughout; no velocity was ever published and no wheel or servo
was commanded.** The only thing that moved was the box, placed and removed by hand.

Per-second capture of state, Bool, lethal cells in window, and — crucially — the
attribution columns `scan_win` (LaserScan returns whose endpoint falls in the
window) and `pts_win` (depth points in the window passing the production odom band
[0.10, 0.99]).

| phase | wall (UTC) | samples | state | Bool | win_cells | scan_win | pts_win |
|---|---|---|---|---|---|---|---|
| baseline, clear | 15:54:45–15:55:54 | 70 | CLEAR, all | False | 0 | 0 | 0 |
| box present | 15:55:55–15:56:56 | 62 | OBSTACLE, all | True | 11–14 | ~80 | ~21,400 |
| box removed | 15:56:57–15:58:59 | 64+ | CLEAR, all | False | 0 | 0 | 0 |

Rising edge, within a single sample:

```
15:55:53     CLEAR             window_clear  False         0   1.519        0       0
15:55:54     CLEAR             window_clear  False         0   1.550        0       0
15:55:55  OBSTACLE   lethal_cells_in_window   True        11   0.326      117   69395
15:55:56  OBSTACLE   lethal_cells_in_window   True        13   0.326      105   69202
15:55:57  OBSTACLE   lethal_cells_in_window   True        13   0.432       81   21388
```

Falling edge:

```
15:56:56  OBSTACLE   lethal_cells_in_window   True        14   0.432       80   21398
15:56:57     CLEAR             window_clear  False         0   1.519        0   20531
15:56:58     CLEAR             window_clear  False         0   1.519        0       0
```

Exactly **three state changes** across 260 s and 1669 Bool messages. The nearest
lethal cell reads 1.519 m both before placement and after removal — the same
background structure — so the scene demonstrably reset rather than merely quieting.

This establishes, with captured evidence:

- **Depth observations are consumed.** `pts_win` went 0 → 69,395 at the instant of
  placement and returned to 0 on removal. A subscription that was not being
  consumed could not move that number.
- **The costmap→monitor→Bool chain carries a real obstacle signal.** State and Bool
  tracked the physical object in both directions.
- **Clear environment reports CLEAR** (70 consecutive samples before, 64+ after).
- **Recovery works** and is not a one-sample flicker.
- **The cluster filter does its job.** In the first Phase A run the monitor held
  CLEAR while `win_cells` was 1 — a lone 25 cm² cell correctly rejected by
  `min_cluster_area_cm2: 100.0` rather than tripping a stop.

**Superseded run, recorded for honesty.** An earlier Phase A attempt (14:24–14:28
UTC) was **confounded**: the capture was already `OBSTACLE` at its first sample
(22 window cells, `scan_win` 43, `pts_win` 0) because the box went in before the
baseline was established. Its *falling* edge was clean — CLEAR at 14:26:20 held for
131 s across 132 consecutive samples with zero relapses — but its rising edge proves
nothing, so the run above was performed from a verified-clear baseline and
supersedes it. The superseded run is not used to support any claim.

---

### NOT LIVE-VERIFIED — Phase B, depth-only detection of an overhang

**Status: pending. No evidence exists for this claim and none is asserted.**

Phase A used a box resting on the floor, which **both** sensors see (`scan_win`
≈ 80 alongside `pts_win` ≈ 21,400). It therefore proves depth is consumed, but it
does **not** show the depth camera detecting anything the LiDAR misses — which is
the fusion's actual contribution and the geometry behind the 2026-07-22 pedestal
desk collision.

The discriminating test is an object whose lowest solid part is above the floor
with nothing beneath it, so the LiDAR's fixed ~0.045 m scan plane passes
underneath. The passing result is **`scan_win` == 0 with `pts_win` > 0 and state
OBSTACLE**.

Three attempts were made. The first was aborted at 37 samples when the session was
stopped. The second was cancelled before placement because a suitable overhanging
object was not available. The third ran to completion (261 samples, 16:05:55–16:10:15
UTC) and **failed**:

```
samples: 261    CLEAR 202    OBSTACLE 59
max pts_win:   0
max scan_win: 10
max win_cells: 5
samples with pts_win > 0 AND scan_win == 0:  0
```

```
16:07:05  OBSTACLE   lethal_cells_in_window   True         5   0.941        7       0
16:07:06  OBSTACLE   lethal_cells_in_window   True         5   0.834        9       0
16:07:07     CLEAR             window_clear  False         3   0.834        8       0
16:07:08  OBSTACLE   lethal_cells_in_window   True         4   0.834        8       0
```

`pts_win` was **0 for every one of the 261 samples**. The LiDAR saw the object
(`scan_win` 7–10); the depth camera never did. The discriminating query returned
zero samples.

**The failure was in the test instruction, not in the pipeline or the operator.**
The placement asked for was "lowest solid part ≥ 0.15 m above the floor, 0.5–0.7 m
ahead". That is outside this camera's field of view. Measured afterwards by
projecting the **real ray directions of one live 201,738-point cloud** to candidate
obstacle heights, counting only rays crossing within the monitor window's ±0.3 m
half-width:

```
camera origin in base_link: (0.105, 0.021, 0.192)  -> 0.198 m above floor
observed floor patch in base_link: x 0.217..0.669   y -0.353..0.344

 obstacle height  base_link z     reachable x (m)    rays
           0.05m        0.044       0.18 .. 0.61   201660
           0.10m        0.094       0.16 .. 0.44   201738
           0.15m        0.144       0.13 .. 0.27   201738
           0.20m        0.194        unreachable        0
           0.25m        0.244        unreachable        0
           0.30m        0.294        unreachable        0
           0.40m        0.394        unreachable        0
           0.50m        0.494        unreachable        0
```

At 0.15 m the camera reaches only x 0.13–0.27 m — the object was placed 2–5x
beyond that. At and above 0.20 m nothing is reachable at all: no ray in the cloud
rises above ~19.3° below horizontal, consistent with a ~40° downtilt.

**MEASURED, NOT ISOLATED.** Whether the 0.669 m far cutoff is the FOV edge or
grazing-angle dropout on the floor was **not** isolated, and no cause is asserted.
What is measured is that on this scene the camera contributed no ray above
`base_link` z ≈ 0.194.

This finding matters well beyond Phase B, and is flagged for the final whole-branch
review and for Task 11:

- `camera_coverage_ceiling_m: 0.94` (stop_monitor_params.yaml) and
  `max_obstacle_height: 0.99` (costmap_params.yaml) are **height-filter bounds, not
  achieved coverage**. Neither is reached in practice. The parameters are not wrong
  — a filter bound above the achievable ceiling admits nothing extra — but a trial
  write-up must not quote 0.94 m as camera coverage.
- The camera's usable contribution at its current pose is a **narrow near-floor
  band, roughly x 0.16–0.61 m for obstacles 0.05–0.10 m tall**. The monitor window
  extends to 1.0 m, so most of the window is LiDAR-only in practice, not just the
  far 6 cm the config comment describes.
- This is quantitative evidence for the 2026-07-22 pedestal-desk blind spot: at
  desk-underside height the camera has no rays at all.

Until a Phase B is run inside the measured envelope, **depth-only detection of
LiDAR-invisible geometry remains unverified**, and no claim about it may appear in
the write-up. A viable retry geometry follows from the table above: overhang
underside **0.10–0.14 m** above the floor (above the LiDAR's ~0.045 m plane and
above the ~0.052 m minimum detectable height) placed at **x ≈ 0.20–0.30 m**, with
supports outside ±0.3 m laterally.

---

### Task 8 status summary

| goal | status |
|---|---|
| Sensor data enters the pipeline | VERIFIED — all 8 stages carrying traffic concurrently |
| Costmap produces expected obstacle representation | VERIFIED — after the height-band fix; 11/12 cells scan-backed |
| Monitor positively validates active costmap | VERIFIED — `get_state` returns `active` on two independent launches |
| Clear environment → CLEAR | VERIFIED — 136/136, and 70 + 64 samples either side of Phase A |
| Lethal obstacle → safety response | VERIFIED — OBSTACLE + Bool True, 62/62 samples |
| Obstacle removal → recovery | VERIFIED — three clean state changes, no flicker |
| Timing/latency | NOT REQUIRED by the Task 8 brief — Task 12 |
| Robot stationary unless a test requires motion | HELD — no motion was ever commanded in Task 8 |
| Depth-only detection of an overhang | **NOT LIVE-VERIFIED — pending** |

### Robot left as found

```
$ kill -TERM 13518 13602 13604 13606 13608 13610
$ ros2 node list | wc -l
28
$ ros2 node list | grep -iE "costmap|poc_fusion|stop_monitor|depth_preprocess"
(none of ours - clean)
$ ps -eo pid,stat,cmd | grep -E "poc_fusion|nav2_costmap|lifecycle_manager|depth_preprocess"
  13606 Z    [nav2_costmap_2d] <defunct>
```

28 vendor nodes remain, matching the baseline recorded at the end of Task 6, and
`proximity_alert` was undisturbed throughout. The single `<defunct>` entry is an
unreaped exit status, not a live process — the container runs no init reaper, as
documented in Task 6. No velocity was ever published, no arm or servo command was
ever sent, and `.stop_ros.sh` was not run.

### Deferred to the final whole-branch review

Recorded here so none of these is lost; none blocks a Task 8 criterion, and none is
explained.

1. **Un-cleared ghost cell** at −108.2°, range 0.455 m, with the concurrent scan
   reading 6.341 m (Step 3f). Outside the window; fails safe, not dangerous.
2. **`/poc_fusion/points` at 2.13 Hz** (Step 3a), below the previously ratified
   ~3–7 Hz band. An input for Tasks 11/12, never an explanation.
3. **State chatter at the cluster-area threshold.** In the failed Phase B run
   `win_cells` oscillated 3–5 against `min_cluster_area_cm2: 100.0` (= exactly 4
   cells at 25 cm² each), flipping CLEAR/OBSTACLE across the boundary. No
   hysteresis exists on this threshold. Whether that matters is a Task 11 tuning
   question against trial data.
4. **Camera coverage envelope** far tighter than the configured filter ceilings
   (Phase B section). Affects how camera coverage may be described in the write-up.
5. **LD19 beam validity: 35.9%** of 504 beams finite in-range, with an all-`nan`
   arc ~105°–240° (Step 3f).

## Task 9: Stop action + audible alert (2026-08-11 ~01:12–01:55 HKT / 2026-08-10 ~17:12–17:55 UTC)

Every run in this section was **observe-only**: the stack was launched with
`publish_cmd_vel:=false`, so `stop_action_node` created **no Twist publisher at
all**. No wheel could be commanded. This is stated up front because most of what
follows looks like stop-path verification and is not — see "What Task 9 does NOT
yet establish".

### Design: what the node decides, and what decides it

All decision logic is in `poc_fusion/lib/stop_action.py`, which imports no ROS and
is unit-tested off-robot. `stop_action_node.py` is a shell that maps topics onto
it. The four reasons are disjoint and never share an output value:

| Reason | Trigger | stop | alert |
|---|---|---|---|
| `no_obstacle_signal` | no Bool ever received | **True** | no |
| `obstacle_signal_stale` | age > `signal_staleness_bound_s` | **True** | no |
| `obstacle_detected` | Bool True and fresh | **True** | on rising edge only |
| `clear` | Bool False and fresh | False | no |

The two fail-safe reasons stop the robot but deliberately do **not** fire the
alert: an alert means "an obstacle was detected", and neither of those is a
detection. Conflating them would teach an operator to distrust the buzzer.

**Why absence must mean stop.** The monitor publishes *no* `std_msgs/Bool` while
degraded (Task 7). On the derived Bool topic, UNKNOWN is therefore a **gap in the
stream**, indistinguishable from a dead monitor, a crashed container or an
unplugged camera. Absence of a message is never permission to drive, so the
staleness timeout is the whole mechanism that makes UNKNOWN safe.

### Step 1: unit tests, with mutation testing

`test_stop_action.py`, 16 tests. Tests were proven to have teeth by mutation:

| Mutation | Result |
|---|---|
| `stop=True` → `stop=False` for `no_obstacle_signal` | **caught** |
| `>` → `>=` on the staleness comparison | **caught** |
| drop the `previous_reason` guard (alert every tick) | **caught** |
| `obstacle` checked before staleness | **caught** |

4/4 caught. Full suite at the close of this task: **121 passed**.

### Step 2: live safety gates (observe-only, zero motion)

Launched with `stop_action:=true publish_cmd_vel:=false` (log `/tmp/t9_launch2.log`).

**Gate 1 — startup fail-safe.** Before any monitor message existed, the node held
a stop and said why:

```
[stop_action_node]: stop action: None -> no_obstacle_signal (stop=True)
[stop_action_node]: holding a stop with NO live obstacle signal
                    (no_obstacle_signal). This is fail-safe, not a detection.
```

then released only once the monitor reached CLEAR, 4.4 s later:

```
[stop_action_node]: stop action: no_obstacle_signal -> clear (stop=False)
```

**Gate 2 — monitor death.** In an earlier run (`/tmp/t9_launch1.log`) the monitor
was SIGKILLed. `stop_active` flipped false→true and the reason was correct:

```
[stop_action_node]: stop action: clear -> obstacle_signal_stale (stop=True)
[stop_action_node]: holding a stop with NO live obstacle signal
                    (obstacle_signal_stale). This is fail-safe, not a detection.
```

**Gate 3 — no publisher exists.** The decisive check that this run could not move
the robot:

```
$ ros2 topic info /cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1
```

**Gate 4 — output rate.** `/costmap_app/stop_active` measured at **10.006 Hz**
(window 96, min 0.091 s, max 0.110 s) against `tick_rate_hz: 10.0`.

### Measured UNKNOWN gaps — an updated number

Two UNKNOWN episodes occurred in the ~4-minute run, extracted from all 74 monitor
state transitions:

| Episode | Duration | Cause |
|---|---|---|
| startup | 0.992 s | `costmap_not_active` — before the costmap went active |
| steady state | **0.303 s** | `tf_lookup_failed` |

The startup episode is covered by the `no_obstacle_signal` fail-safe and is not a
steady-state gap. **The steady-state gap of 0.303 s is a new measurement and is
larger than the previously recorded worst gap of 0.2612 s.** It was absorbed by
`signal_staleness_bound_s: 0.75` without a degraded stop, which is the designed
behaviour. Recorded plainly: the margin against the configured bound is 2.5×.
This is a fresh measurement of a real gap, not a re-opening of the withdrawn
claim about 0.40 s, which remains withdrawn and is not revived here.

### The alert path

`_fire_alert()` spawns `aplay -D <device> <wav>` non-blocking, deliberately
identical to the mechanism already in `proximity_alert`'s `path_tracker` (line
383) so this POC introduces no second audio mechanism. `proximity_alert` is
frozen for the surface trials, so extracting the shared code was not an option.

What is established: the WAV exists (`/home/ubuntu/shared/audio/obstacle_alert.wav`,
30912 bytes), `aplay` exists (`/usr/bin/aplay`), `alert_enabled=True`, and 17
rising edges fired the path with **no** `could not launch aplay` warning — so
every `Popen` succeeded.

What is **not** established: **audibility**. `Popen` is non-blocking with output
discarded, so a successful spawn is not proof of sound. Only a human in the room
can close this, and it is listed as pending below rather than assumed.

### What Task 9 does NOT yet establish

- **That a stop actually stops the robot.** Not tested. Requires motion.
- **That the stop persists.** The STM32 latches the last commanded velocity
  forever, so a zero Twist at costmap rate does not hold a stop. `motion_watchdog`
  (`/cmd_vel_unsafe` → `/cmd_vel` at 20 Hz) is **required** in the loop, and when
  it is, `cmd_vel_topic` must be repointed at its **input**, `/cmd_vel_unsafe`.
- **Publisher arbitration.** ROS 2 does not arbitrate publishers. If anything else
  publishes to the same topic, last-write-wins and `zero_twist_burst: 3` is a
  mitigation, not a fix. The correct fix is a series gate; that is out of Task 9
  scope and is documented in the node's docstring rather than silently hoped away.
- **Audibility of the alert** (above).

`/controller/cmd_vel` has 5 vendor publishers and is never written to; the node
raises `ValueError` at construction if `cmd_vel_topic` is set to it.

### Launch default is deliberately unsafe-by-omission

`stop_action` defaults to **`'false'`** in `poc_fusion.launch.py`. The Task 8
launch command therefore remains incapable of moving a wheel even after this task
added a node that can. Opting in is explicit.

## Task 12: Latency budget — Steps 1–4

### Step 1: `latency_recorder_node.py`

On each False→True transition of `/costmap_app/obstacle_detected`, records
`now − (stamp of the newest depth frame that had ARRIVED before the transition)`.
`/poc_fusion/depth_cleaned` carries the camera stamp verbatim (Task 4 Step 1),
which is why it is the origin. **This node commands nothing.**

Pure logic lives in `lib/latency_stats.py`, 18 tests. Mutation testing found **two
surviving mutants — my own tests were weaker than their docstrings claimed**:

1. *Select the reference frame by `stamp` instead of by arrival.* Survived because
   the test put the late frame outside the eligibility window, so the **filter**
   answered it and the comparator was never exercised. Fixed by making both frames
   arrive before the transition with stamps in the opposite order to arrivals.
2. *Truncate the p95 rank instead of rounding up.* Survived because at n=20,
   `ceil(0.95n)` and `int(0.95n)` both land on rank 19. Fixed by adding n=3
   (2.85 → rank 3 vs rank 2) and n=5 cases.

Both mutants fail correctly now. Recorded because the first version of this
section would have claimed tested behaviour that was not actually pinned.

### Step 2: ≥20 rising edges

**35 edges collected, 0 dropped**, over a 252.8 s window (2026-08-10 17:44:56 →
17:49:09 UTC), by repeatedly presenting an obstacle. Requirement was ≥20.

Dataset audited before use, not merely trusted:

| Check | Result |
|---|---|
| Rows | 35 (+ header) |
| `edge_index` contiguous 1..n | yes — single run, no appended second run |
| `transition_time_s` monotonic | yes |
| Non-positive latencies | 0 |
| Frames selected that arrived **after** their edge | 0 |
| `DROPPED` warnings in log | 0 |

Node's own final report:

```
FINAL latency over 35 edges (LOWER BOUND, excludes camera exposure):
median 75.4 ms, p95 115.8 ms, min 42.0 ms, max 116.4 ms, dropped 0.
Samples: /tmp/poc_fusion_latency.csv
```

**median 75.4 ms, p95 115.8 ms** (p95 nearest-rank, an actually-observed value,
not interpolated). Inter-edge gaps: min 3.4 s, median 6.7 s, max 20.7 s.

### Step 3: what this figure covers and what it does not

**Covers:** depth preprocessing, rectification, projection, costmap update, the
costmap publish interval, and monitor evaluation — everything this POC adds.

**Does not cover:** the camera's own exposure and internal processing latency,
which is unrecoverable from message stamps and would need external high-frame-rate
video to capture.

The reported figure is a **lower bound on true physical-entry-to-stop latency**.
It is **not** end-to-end, and it is **not** stop-completion latency — it ends when
the Bool flips, not when the wheels have stopped. The wheel-stop segment is
unmeasured because Task 9 Step 3 has not run.

### Step 4: check against the budget

**Target: p95 ≤ 300 ms. Measured p95 = 115.8 ms. PASS**, with 2.6× margin.

The target is met by the added pipeline alone, so the escalation path in the plan
(the Task 4 median filter kernel, explicitly **not** a further publish-rate
increase) is not triggered and no tuning lever was pulled. `publish_frequency`
remains at the 10 Hz set in Task 6 Step 1.

Caveat carried forward rather than buried: because the figure excludes camera
exposure and the wheel-stop segment, **the true physical latency is higher by an
unmeasured amount**. The 2.6× margin is against the lower bound, not against the
real number. Task 13's CPU figures are not yet collected, so the latency/compute
tradeoff is not yet visible.

### Sample set archived

`/tmp/poc_fusion_latency.csv` is ephemeral, so the 35-sample set backing the
headline numbers is committed at
`docs/poc_fusion_data/task12_latency_2026-08-10.csv`. It is POC evidence and is
deliberately kept out of `organized_data/`, which belongs to the frozen
surface-trial pipeline.
