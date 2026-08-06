# LiDAR + Depth Camera Costmap Fusion POC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fuse the arm-mounted Aurora 930 Pro depth camera with the LD19 LiDAR into a single Nav2 local costmap, and demonstrate that the fused representation detects obstacle classes the LiDAR alone misses.

**Architecture:** Additive-only ROS 2 package `poc_fusion`. Depth image → OpenCV preprocessing (invalid + self-arm masking) → `depth_image_proc` point cloud → standalone `nav2_costmap_2d` local costmap (also consuming `/scan_raw`) → stop-monitor node → buzzer + zero-velocity stop.

**Tech Stack:** ROS 2 Humble, Python 3.10, `nav2_costmap_2d`, `depth_image_proc`, `cv_bridge`, OpenCV 4.10, numpy, scipy 1.8.

**Design doc:** [2026-08-05-lidar-depth-costmap-fusion-design.md](../specs/2026-08-05-lidar-depth-costmap-fusion-design.md)

---

## Environment Assumptions

**Read this once. It is not repeated per task — every command in this plan assumes it.**

| | |
|---|---|
| Robot | Hiwonder **LanderPi** (Raspberry Pi 5) |
| Container | Docker container named **`MentorPi`**, image `ros:humble`, **host** network mode |
| ROS distro | Humble |
| Workspace | **`/home/ubuntu/ros2_ws`** — *inside the container*. `proximity_alert` already lives in `src/`. |
| Build user | **`ubuntu`**, never `root` |

**"LanderPi" is the robot. "MentorPi" is only the container's name.** Use `MentorPi` in `docker exec` commands and nowhere else.

### Every command runs inside the container, as `ubuntu`

```bash
docker exec -u ubuntu MentorPi bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ubuntu/ros2_ws/install/setup.bash
  <command>
'
```

- **`-u ubuntu` is mandatory.** Running as `root` breaks FastRTPS shared-memory transport
  *silently* — nodes start, publish nothing discoverable, and every diagnostic reports
  "no publishers." This has already cost this project a full night of false diagnoses.
- **Never `colcon build` as root.** It leaves root-owned artifacts in
  `build/`/`install/`/`log/` that subsequent `ubuntu` builds cannot overwrite, and the
  resulting errors do not name the real cause.
- **Source both setup files.** `/opt/ros/humble/setup.bash` alone does not expose
  workspace packages or Hiwonder message types (`ros_robot_controller_msgs`,
  `servo_controller_msgs`) and they will appear "unknown."
- **`need_compile` must be set for Hiwonder launch files.** They read
  `os.environ['need_compile']` at import and crash with `KeyError` if it is unset in a
  fresh `docker exec` shell. Pass `-e need_compile=False`. (Value read from the running
  bringup process, `/proc/<pid>/environ`.)

### Paths that do not exist — do not use them

- **`~/ros_ws`** — does not exist anywhere.
- **Host `~/ros2_ws`** — exists, but is a **stale clone**. Editing it changes nothing
  the robot runs. The live workspace is inside the container.

### Motion safety

- **The robot has no motion watchdog by default.** A published velocity persists until
  something overrides it. A single uncommanded spin already ran for 8 minutes.
  **Always follow any velocity publication with an explicit zero-`Twist` stop.**
- This POC's stop path depends on this project's existing `motion_watchdog` node being
  in the loop (Task 9).

### Verified topic and hardware facts

Confirmed live on 2026-08-06. Do not re-derive; do re-check if something behaves oddly.

| | |
|---|---|
| LiDAR | **LD19 driver**, node `/LD19` — but the platform is provisioned `LIDAR_TYPE=MS200`. See "LiDAR model discrepancy" below. |
| Scan topic | **`/scan_raw`** — **`/scan` does not exist**, and the reason is known (below) |
| Scan rate | 9.87 Hz |
| Depth image | `/ascamera/camera_publisher/depth0/image_raw`, 640×400, **14.7 Hz** |
| Depth camera frame | **`depth_camera_link`** (URDF also defines `depth_cam_link` — different frame, not the one topics use) |
| `camera_info` | `/ascamera/camera_publisher/depth0/camera_info` — **valid** |
| Driver cloud | `/ascamera/camera_publisher/depth0/points` — already published |
| Velocity topic (ours) | **`/cmd_vel`** |
| Velocity topic (vendor) | `/controller/cmd_vel` — 5 vendor publishers, **do not use** |
| `depth_image_proc` | **NOT installed** — see Task 0 |

### LiDAR model discrepancy

The Hiwonder manual says **MS200**. The robot runs an **LD19** driver. The vendor's own
launch file explains it — `peripherals/launch/lidar.launch.py` reads `LIDAR_TYPE` from
the environment, and its `MS200` branch has the MS200 driver **commented out** and
force-routed to `ldlidar_LD19.launch.py`. The running bringup has
**`LIDAR_TYPE=MS200`**. So the unit is provisioned as MS200 and driven as LD19.

Code against LD19. **And note in the paper's Methodology** — the scan rate does not
reveal the mismatch (manual quotes 7–15 Hz for MS200; measured 9.87 Hz is inside that
band), so a reviewer comparing against the spec sheet will flag "LD19" as an error
unless it is disclosed. Task 16 Step 3 covers this.

### Why `/scan` does not exist

The same launch file builds a `laser_filters` `scan_to_scan_filter_chain` node that
would remap `scan_raw` → `scan`, then **leaves it out of the returned
`LaunchDescription`** (commented out). Consequences: `/scan_raw` is the only scan topic,
and **no laser filtering is applied at all** — no range clipping, shadow filter, or
speckle removal. Do not describe the LiDAR input as filtered anywhere in the writeup.

---

## Global Constraints

- **Additive only.** No edits to `proximity_alert/`, its launch files, or anything in
  the trial logging path. New package `poc_fusion` only. The surface-trial dataset must
  remain reproducible from unchanged code.
- **The scan topic is a parameter, never a literal.** It is declared once in
  `config/costmap_params.yaml` as `scan_topic: /scan_raw` and referenced from there.
  No file may hard-code `/scan` or `/scan_raw`. Rationale: a wrong scan topic makes
  `nav2_costmap_2d` publish a **valid, all-free** costmap with no error — the single
  most expensive silent failure available here.
- **Frames.** `base_link` is REP-103: +x forward, +y left, +z up. Forward is never
  determined empirically.
- **Costmap `global_frame` stays `odom`.** This matches Nav2's shipped default and is
  required for `observation_persistence` to be correct under rotation. See the design
  doc §5.3 — this was reviewed and the `base_link` alternative was rejected on
  technical grounds, not overlooked.
- **Pure logic is unit-testable and tested off-robot.** Geometry, masking, and
  thresholding live in dependency-free modules under `poc_fusion/lib/` taking arrays
  and numbers, not ROS messages.
- **Every on-robot verification step records its output** into
  `docs/poc_fusion_verification.md`. "It looked right in RViz" is not a result.

## File Structure

```
poc_fusion/
├── package.xml
├── setup.py
├── config/
│   ├── costmap_params.yaml          # includes scan_topic — single source of truth
│   ├── depth_preprocess_params.yaml # self-arm ROI, filter params
│   └── stop_monitor_params.yaml
├── launch/
│   └── poc_fusion.launch.py
├── poc_fusion/
│   ├── depth_preprocess_node.py
│   ├── costmap_stop_monitor_node.py
│   ├── latency_recorder_node.py
│   └── lib/
│       ├── window_geometry.py       # pure: cell mask, area conversion
│       └── obstacle_detection.py    # pure: occupancy fraction, cluster area
└── test/
    ├── test_window_geometry.py
    └── test_obstacle_detection.py

docs/poc_fusion_verification.md      # running record of on-robot measurements
```

---

## Task 0: Environment preflight

Most of this is already verified (see Environment Assumptions). This task records it and
closes the one real gap.

- [ ] **Step 1: Create `docs/poc_fusion_verification.md`** and paste in the verified
      facts table above as its first section, dated, with the command used for each.
- [ ] **Step 2: Confirm the scan topic is still `/scan_raw` and record the rate.**
      ```
      ros2 topic list | grep -i scan
      ros2 topic hz /scan_raw
      ```
      Write the result into `config/costmap_params.yaml` as `scan_topic`. **Every other
      file reads it from there.** If it is ever not `/scan_raw`, this one line changes
      and nothing else does.
- [ ] **Step 2a: Record the LiDAR model discrepancy** in the verification doc, with the
      `lidar.launch.py` excerpt and the `LIDAR_TYPE=MS200` process-env evidence, and the
      fact that the `laser_filters` chain is commented out of the launch description.
      Both feed Task 16 Steps 3a/3b. Confirm `LIDAR_TYPE` is still `MS200` and the node
      is still `/LD19` — if either flips, the driver changed and rates must be re-measured.
- [ ] **Step 3: Install `depth_image_proc` and `image_proc`.** `depth_image_proc` is
      **not present** on this robot — Task 5 cannot start without it. `image_proc`
      provides the rectify node added in Task 5 Step 2.
      ```
      docker exec -u root MentorPi bash -lc 'apt-get update && apt-get install -y ros-humble-depth-image-proc ros-humble-image-proc'
      ```
      (Package install is the one legitimate `-u root` use; it touches the system image,
      not the workspace. Nothing is *built* as root.)
      Verify as `ubuntu`: `ros2 pkg prefix depth_image_proc`.
- [ ] **Step 4: Confirm the rest of the dependency set** (already verified present, so
      this is a regression check, not discovery): `nav2_costmap_2d`,
      `nav2_lifecycle_manager`, `rclcpp_components`, `cv_bridge`, `tf2_tools`,
      scipy 1.8.0, OpenCV 4.10.0. Note that `ros2 pkg executables nav2_costmap_2d`
      lists the executable as plain `nav2_costmap_2d`.
- [ ] **Step 5: Record the velocity-topic decision** in the verification doc:
      `/cmd_vel` is the POC's output (0 publishers, 1 subscriber `odom_publisher`,
      and this project's own `path_tracker` publishes there);
      `/controller/cmd_vel` has 5 vendor publishers and is off-limits.
      **Resolve this now, not during debugging** — publishing into the wrong one
      produces a robot that ignores stop commands with no error anywhere.

## Task 1: TF chain verification (record only)

The chain was verified live on 2026-08-06 and resolves. This task documents it and
establishes the pitch-measurement procedure that Task 10 will use.

- [ ] **Step 1: Record the chain**
      `base_footprint → base_link → link1..link4 → camera_connect_link →
      depth_cam_link → depth_camera_link`, plus `odom → base_link` resolving.
      Commands: `ros2 run tf2_tools view_frames`,
      `ros2 run tf2_ros tf2_echo base_link depth_camera_link`.
      **Note the trap in the doc:** the URDF defines both `depth_cam_link` and
      `depth_camera_link`; topics are stamped `depth_camera_link`. Using the wrong one
      shifts the whole cloud.
- [ ] **Step 2: Confirm the transform is genuinely arm-driven,** not a static fudge —
      the URDF puts `camera_connect_joint`'s parent at `link4`, which moves with
      revolute `joint4`. Record that this is why the arm must be locked for every run.
- [ ] **Step 3: Record the pitch-measurement procedure and current value.**
      From `tf2_echo base_link depth_camera_link`, take the optical **forward** axis in
      `base_link` and read its z-component; downward pitch = `asin(−z)`.
      Current measured: forward axis `(0.766, −0.005, −0.642)` → **39.9° down**.
      This is the check Task 10 must satisfy at 20°.

## Task 2: Package scaffolding

- [ ] **Step 1: Create the `poc_fusion` package** in `/home/ubuntu/ros2_ws/src/`
      (`ament_python`), with the file structure above and empty config files.
- [ ] **Step 2: Declare dependencies** in `package.xml`: `rclpy`, `sensor_msgs`,
      `nav_msgs`, `geometry_msgs`, `std_msgs`, `cv_bridge`, `tf2_ros`,
      `nav2_costmap_2d`, `nav2_lifecycle_manager`, `depth_image_proc`, `image_proc`.
- [ ] **Step 3: Register entry points** in `setup.py`: `depth_preprocess_node`,
      `costmap_stop_monitor_node`, `latency_recorder_node`. Add `config/` and `launch/`
      to `data_files`.
- [ ] **Step 4: Build and confirm the package is discoverable.**
      `colcon build --packages-select poc_fusion` as `ubuntu`, then
      `ros2 pkg executables poc_fusion`.

## Task 3: Pure geometry and detection logic (off-robot)

Write and test these before touching hardware. Everything here is arrays and numbers.

- [ ] **Step 1: `lib/window_geometry.py` — `window_mask(...)`.**
      Given costmap `width`, `height`, `resolution`, `origin_x`, `origin_y`, and the
      robot pose in `odom` (`rx`, `ry`, `ryaw`), plus `forward_m` and `half_width_m`,
      return a `(height, width)` boolean array marking cells whose centres fall inside
      the body-aligned rectangle:
      ```
      X = origin_x + (col + 0.5)*res;  Y = origin_y + (row + 0.5)*res
      bx =  cos(ryaw)*(X-rx) + sin(ryaw)*(Y-ry)
      by = -sin(ryaw)*(X-rx) + cos(ryaw)*(Y-ry)
      mask = (bx >= 0) & (bx <= forward_m) & (abs(by) <= half_width_m)
      ```
      Vectorized with numpy; a 60×60 grid is 3600 points.
      **No `forward_sign` parameter. No "robot is at grid centre" assumption.**
      Forward is +x by REP-103, and `origin` is used directly.
- [ ] **Step 2: `lib/window_geometry.py` — `area_cm2_to_cells(area_cm2, resolution)`.**
      Physical area threshold → cell count, computed from the live resolution so the
      threshold means the same thing if resolution changes.
- [ ] **Step 3: `lib/obstacle_detection.py`.**
      - `occupancy_fraction(values, lethal_threshold)` — `values` is the 1-D array of
        masked cell costs.
      - `largest_cluster_cells(grid, mask, lethal_threshold)` — `scipy.ndimage.label`
        on `(grid >= lethal) & mask` with **8-connectivity**, returns the largest
        component's size. 8-connectivity because point-cloud marks arrive as speckled
        blobs that frequently touch only diagonally; 4-connectivity fragments them
        below threshold and the trigger never fires.
      - `should_stop(...)` — OR of the fraction and area conditions.
- [ ] **Step 4: Unit tests.** Empty grid → no stop. Fully-lethal window → stop. A
      diagonal-only chain → one cluster under 8-connectivity. Sub-threshold speckle →
      no stop. Unknown (`-1`) cells treated as not-lethal, explicitly asserted.
- [ ] **Step 5: Rotation test for `window_mask` — the important one.**
      Place a synthetic lethal cell 1m directly ahead of the robot in `odom`. Sweep
      `ryaw` through 0, ±45°, ±90°, 180°, re-deriving the cell's `odom` position each
      time so it stays physically ahead: assert it is always in the mask. Then hold the
      cell fixed in `odom` and rotate the robot away: assert it leaves the mask.
      **This is the regression test for the bug the earlier row/column-slice design
      would have shipped** — a window that silently swings to the robot's side and
      then its back as it turns, while still returning plausible numbers.
- [ ] **Step 6: Run the full suite off-robot.** `colcon test --packages-select poc_fusion`.

## Task 4: Depth preprocessing node

- [ ] **Step 1: Node skeleton.** Subscribe to the raw depth image, `cv_bridge` to numpy,
      republish unchanged to `/poc_fusion/depth_cleaned`. **Preserve `header` exactly** —
      stamp and `frame_id` must pass through verbatim. The stamp is the camera frame
      time and Task 12's latency measurement depends on it surviving this node; the
      `frame_id` must remain `depth_camera_link` or the costmap transforms the cloud
      from the wrong frame.
- [ ] **Step 2: Invalid-pixel masking.** Zero and NaN depth → invalid. Log the invalid
      fraction periodically; a sudden jump is the first sign of a camera fault.
- [ ] **Step 3: Self-arm ROI masking from YAML.** Read the ROI from
      `config/depth_preprocess_params.yaml` — **not hard-coded**, because it must be
      re-derived whenever the arm pose changes (Task 10 changes it). Start with a
      permissive placeholder; Task 11 measures the real one.
- [ ] **Step 4: Spatial denoising.** Median filter, kernel size a parameter. Keep it
      cheap — this runs at 14.7 Hz on a Pi 5 and Task 13 measures the cost.
- [ ] **Step 5: Debug overlay.** Publish `/poc_fusion/debug_image` with the masked
      region drawn. This is the image Task 11 reads the ROI off.
- [ ] **Step 6: Verify on-robot.** Confirm `/poc_fusion/depth_cleaned` publishes at
      ~14.7 Hz and that its header stamps match the source topic's.

## Task 5: Point cloud generation

**Requires `depth_image_proc` from Task 0 Step 3. Confirm `ros2 pkg prefix
depth_image_proc` succeeds before starting.**

- [ ] **Step 1: Add `depth_image_proc::PointCloudXyzNode`** to the launch file as a
      composable node.
- [ ] **Step 2: Insert an `image_proc` rectify node** between
      `/poc_fusion/depth_cleaned` and the projection, publishing
      `/poc_fusion/depth_rect`.
      **Set interpolation to nearest-neighbour, not the linear default.** Linear
      interpolation across a depth discontinuity synthesizes pixels at depths where no
      surface exists ("flying pixels"), which project into the cloud as free-floating
      obstacle marks — a worse artifact for a stop trigger than the distortion being
      corrected. Verify the setting took effect by checking for speckle at depth edges
      in RViz before and after.
- [ ] **Step 3: Remap** `image_rect` ← `/poc_fusion/depth_rect`,
      `camera_info` ← `/ascamera/camera_publisher/depth0/camera_info`,
      `points` → `/poc_fusion/points`.
      Rectifying is what makes Task 11's frame-edge sweep interpretable: without it,
      an edge miss cannot be attributed between structured-light falloff and distortion.
      Record the measured distortion magnitude in the verification doc for the record —
      worst case **0.64 cm at 1m** at the horizontal edge (2.71 px), against a **5 cm**
      costmap cell, i.e. ~⅛ cell and far below quantization. It was never large enough
      to invalidate an edge result; rectifying removes the argument rather than the
      error. Do **not** describe edge failures as inconclusive — they are now
      attributable, and illumination falloff is the expected dominant effect.
- [ ] **Step 4: Do not add an approximate-intrinsics fallback.** The driver's
      `camera_info` is verified valid (`k = [423.92, 0, 319.30; 0, 424.83, 190.86]`).
      The datasheet estimate (fx 449, fy 471, cy 200) is 6–11% off, which is exactly
      why it must never be a silent default. Instead: **log a visible warning if
      `camera_info` is not received within 5s of startup**, since its absence makes
      `point_cloud_xyz` produce no points and the costmap runs LiDAR-only with no error.
- [ ] **Step 5: Verify in RViz.** Cloud appears in `depth_camera_link`, geometry
      matches the scene, self-arm region is absent. Record a screenshot reference.

## Task 6: Costmap configuration

- [ ] **Step 1: Write `config/costmap_params.yaml`.**
      `global_frame: odom`, `robot_base_frame: base_link`, `rolling_window: true`,
      3m × 3m, `resolution: 0.05`, `update_frequency: 10.0`,
      **`publish_frequency: 10.0`**, `transform_tolerance: 0.3`.
      Comment why publish matches update: the stop monitor is driven by publications,
      so the publish interval is a hard latency floor. At 5 Hz an obstacle arriving just
      after a publish waits up to 200 ms before the monitor can see it — which consumed
      the entire Task 12 budget before any processing ran. 10 Hz halves that to 100 ms.
      The extra publish cost is measured in Task 13, not assumed free.
      Add a comment recording *why* `odom`: it is Nav2's shipped default
      (`nav2_bringup/params/nav2_params.yaml`) and `ObservationBuffer` transforms
      observations *into* `global_frame` at observation time, so a body-fixed global
      frame would corrupt persisted camera marks under rotation.
- [ ] **Step 2: Configure two observation sources** on the obstacle layer.
      `scan` — `LaserScan`, topic from the `scan_topic` key, `observation_persistence: 0.0`.
      `pointcloud` — `PointCloud2` on `/poc_fusion/points`,
      **`observation_persistence: 0.3`** (≈4 frames at the measured 14.7 Hz), reduced
      from 0.5s (≈7 frames — itself a correction of an earlier draft's "2–3 frames").
      **Comment why it was reduced:** persisted marks live in `odom`, so they are only
      as good as odometry, and this platform's mecanum wheel slip is the very thing the
      surface trials measure. On metal and granite, drift during a sharp holonomic
      rotation smears persisted marks into free space, and the cluster-area trigger
      cannot tell a ghost from an obstacle. 0.3 is a starting point; Task 11 Step 5a
      derives the real bound from the trial CSVs.
- [ ] **Step 3: Height-filter the point cloud source** (`min_obstacle_height`,
      `max_obstacle_height`) to exclude ground returns. Initial values are a starting
      point; Task 11 tunes them and they are the most likely thing to need iteration.
- [ ] **Step 4: Add `nav2_lifecycle_manager`** to the launch, autostarting the costmap.
      **Fail the launch loudly if the costmap does not reach `active`** rather than
      letting the stop monitor subscribe to a dead topic.
- [ ] **Step 5: Verify the costmap publishes** and shows both LiDAR and camera marks in
      RViz. Wave a hand in front of the camera only (outside the LiDAR plane) and
      confirm marks appear — this is the first real evidence fusion is doing anything.

## Task 7: Stop monitor node

- [ ] **Step 1: Subscribe to the costmap** `OccupancyGrid` and hold a `tf2_ros` buffer
      and listener.
- [ ] **Step 2: Look up `odom → base_link` at the costmap message's stamp.** On
      lookup failure, log at throttled `warn` and **skip the cycle without publishing** —
      do not fall back to the last pose, which would evaluate a stale window.
- [ ] **Step 3: Assert `info.origin.orientation` is (near) identity** and warn if not.
      `nav2_costmap_2d` never rotates its grid, so this holds; the assertion documents
      the one remaining assumption in `window_mask` rather than leaving it implicit.
- [ ] **Step 4: Build the mask** via `window_geometry.window_mask(...)` and evaluate
      `obstacle_detection.should_stop(...)`. Parameters from
      `config/stop_monitor_params.yaml`: `window_forward_m` (start 1.0),
      `window_half_width_m` (start 0.3), `lethal_threshold` (start 253),
      `min_occupancy_fraction` (start 0.30), `min_cluster_area_cm2` (start 100).
- [ ] **Step 5: Publish `/costmap_app/obstacle_detected`** (`std_msgs/Bool`).
- [ ] **Step 6: Publish the window as a `visualization_msgs/Marker`** — the four
      rectangle corners transformed into `odom`. A rotated rectangle drawn in RViz is
      the fastest way to see the mask is wrong if it ever is.
- [ ] **Step 7: Camera-health watchdog.** If no frame arrives on
      `/poc_fusion/depth_cleaned` for `camera_timeout_s` (2.0), log an explicit error
      stating fusion is no longer active and the system is LiDAR-only; log a recovery
      message when frames resume. **Do not halt** — degrading to the proven sensor is
      the right behaviour for a stop-trigger POC. Failing *silently* is not.

## Task 8: Launch integration

- [ ] **Step 1: Write `launch/poc_fusion.launch.py`** bringing up, in order: the depth
      preprocess node, the `point_cloud_xyz` composable node, the costmap +
      lifecycle manager, and the stop monitor.
- [ ] **Step 2: Load all three config YAMLs.** `scan_topic` is read from
      `costmap_params.yaml` and passed onward — it is not restated in the launch file.
- [ ] **Step 3: Verify the whole chain starts clean** with the existing bringup already
      running, and that nothing in `proximity_alert` is disturbed.

## Task 9: Stop action

- [ ] **Step 1: On rising edge of `obstacle_detected`, trigger the buzzer** using the
      existing buzzer path (do not duplicate its code).
- [ ] **Step 2: Publish zero `Twist` to `/cmd_vel`** — topic name from config,
      defaulting to `/cmd_vel`. Never `/controller/cmd_vel`.
- [ ] **Step 3: Require `motion_watchdog` in the loop and document it.** A single zero
      `Twist` per costmap update (5 Hz) does not *hold* a stop on this robot. Use the
      existing `proximity_alert` `motion_watchdog` node rather than reimplementing
      stop-hold here. Verify a stop actually persists before calling this task done.
- [ ] **Step 4: No avoidance maneuver.** Stop only — consuming the costmap in
      `avoidance.py` is deferred to protect the trial dataset (design doc §11).

## Task 10: Set the camera to 20° downward

**Sequenced here deliberately.** It sits after the pipeline is built because the
ground-plane tuning (Task 11) depends on the final pose, and because the arm control
path is currently unreliable — making it an entry gate would have blocked everything
behind it.

**Known blocker, read before starting.** On 2026-08-06 the `servo_controller` nodes
(`arm_controller`, `gripper_controller`, `controller_manager`) dropped out of the ROS
graph after a `FollowJointTrajectory` goal and had to be relaunched. Recovery command
that worked:

```bash
docker exec -d -u ubuntu -e need_compile=False MentorPi bash -lc \
  'source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && \
   ros2 launch servo_controller servo_controller.launch.py base_frame:=base_footprint'
```

**Correction to an earlier revision of this task.** It stated that `/joint_states`
reports all-zero after a restart and instructed "verify pose through TF, not
`/joint_states`." Both halves were wrong:

1. The all-zero reading was **transient**, not a standing property. Re-checked live,
   `/joint_states` reports `joint2 = 0.9634`, `joint3 = −1.5499`, `joint4 = −1.6755` —
   the correct physical pose.
2. More importantly, **TF is not an independent check on `/joint_states`** —
   `robot_state_publisher` computes TF *from* it. If `/joint_states` is stale or reset,
   `tf2_echo` reports the pose of the zero configuration with full confidence while the
   arm sits somewhere else, and nothing in the graph contradicts it. The original
   instruction would have "verified" the command, not the hardware.

Independent checks that do work: **the camera image itself** (what is actually in
frame), and true bus-servo feedback via `/ros_robot_controller/bus_servo/get_state`.

- [ ] **Step 0: Confirm `/joint_states` is live before trusting anything downstream.**
      Nudge a joint ≤0.05 rad and confirm both `/joint_states` **and**
      `tf2_echo base_link depth_camera_link` change. If TF moves but the arm does not,
      or `/joint_states` reads all-zero while the arm is visibly posed, the chain is
      stale — relaunch `servo_controller` and re-check. Do not proceed on a stale chain.
- [ ] **Step 1: Read the pose from TF, cross-checked against the camera image.**
      `ros2 run tf2_ros tf2_echo base_link depth_camera_link` → the optical forward
      axis is the **third column** of the rotation matrix; downward pitch = `asin(−z)`.
      Cross-check every TF reading against a live snapshot
      (`http://localhost:8080/snapshot?topic=/ascamera/camera_publisher/rgb0/image`):
      at 40° the frame is nearly all floor with the gripper at the bottom edge; at 20°
      noticeably more of the scene ahead should be visible. **Agreement between TF and
      the image is the actual verification** — TF alone is not.
- [ ] **Step 2: Determine the sign of the `joint4` correction empirically** with a
      small nudge (≤0.1 rad) and re-read TF. The direction is not known a priori.
      Baseline recorded 2026-08-06: `joint2 = 0.9634`, `joint3 = −1.5499`,
      `joint4 = −1.6755` rad, giving 39.9° down.
- [ ] **Step 3: Command the pose** so the optical forward axis z-component reads
      **`−0.342 ± 0.03`** (18.3°–21.7° below horizontal). Move in small increments,
      re-reading TF each time.
- [ ] **Step 3a: Contingency if the ROS servo path keeps failing — use the WonderPi
      app to position the arm, but do NOT kill ROS.**
      The arm uses HX-06L serial bus servos; positioning it from the vendor mobile app's
      Robot Control interface bypasses the flaky `FollowJointTrajectory` path entirely.
      That part is sound and is the right fallback.

      **Two corrections to the obvious version of this contingency, both verified:**

      - **`~/.stop_ros.sh` is not on the host.** It exists only *inside* the container
        (`/home/ubuntu/.stop_ros.sh`); running it from a host terminal fails. The host
        has only `~/start.sh`, which is already known broken (it restarts
        `start_node.service`, a unit that does not exist on this machine).
      - **Do not run it.** Its entire contents are:
        ```
        ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
        ```
        That is an indiscriminate `kill -9` of every ROS process — including
        `robot_state_publisher`, the Aurora camera driver, and the LiDAR driver.
        **This POC needs all three.** Killing ROS destroys the
        `base_link → depth_camera_link` TF chain, which is the thing being verified,
        and takes both observation sources with it. There would be nothing left to
        launch the POC against.

      It also does not do what stopping ROS is assumed to do: **bus servos hold
      position from their own internal loop while torque-enabled and powered**, which
      is independent of whether ROS is running. Simply not sending arm commands has the
      same holding effect at none of the cost.

      **So: position with the app, leave the ROS stack up, send no arm commands, and
      verify with Step 1's TF + image cross-check.** If `/joint_states` has gone stale
      after app-driven motion (likely — the app talks to the servo bus directly, and
      `/joint_states` echoes ROS commands), then TF is stale too and the camera image
      plus `/ros_robot_controller/bus_servo/get_state` are the only trustworthy
      verification. Record which one was used.
- [ ] **Step 4: Record the final joint values and TF output** in the verification doc.
      This is the pose every subsequent run must be restored to; without true servo
      feedback, the recorded numbers are the only reference.
- [ ] **Step 5: Re-derive the self-arm ROI** (Task 4 Step 3) at the new pose and update
      `depth_preprocess_params.yaml`. The old ROI is invalid — a stale mask either
      blinds the camera or produces constant false triggers from the gripper.
- [ ] **Step 6: Note in the doc that 20° is expected to change.** It is a compromise
      (design doc §4a): level makes the camera redundant with the LiDAR, 40° buries the
      frame in floor. Re-deriving it is a pose change plus a re-measured ROI, not a code
      change.

## Task 11: Validation and tuning

- [ ] **Step 1: Ground-plane check.** Bare floor ahead, robot stationary, at the Task 10
      pose. Confirm no cells marked occupied. Tune `min_obstacle_height` until clean.
      Record the final value and how many iterations it took.
- [ ] **Step 2: Self-mask check.** Confirm the gripper is excluded and that real
      obstacles near frame edges are **not** over-masked.
- [ ] **Step 3: Detection sweep.** Object at 30cm, 60cm, 1m, 2m; plus 1m at the left and
      right horizontal FOV edges. Record detection outcome per position.
      **Target: >95% detection rate.**
- [ ] **Step 4: False-positive count.** ≥10 stationary bare-floor samples.
      **Target: zero.**
- [ ] **Step 5: Rotation correctness on hardware.** Obstacle fixed in place; rotate the
      robot through ±90° in place. Detection must be True only while the obstacle is
      genuinely ahead. This is the hardware counterpart to Task 3 Step 5 and the direct
      test of the `odom`-grid / `base_link`-window design.
- [ ] **Step 5a: Persistence ghosting under wheel slip.**
      First derive the bound from data this project already has: read per-surface
      odometry-vs-ground-truth slippage out of the trial CSVs and pick
      `observation_persistence` such that worst-case drift over the window stays under
      one costmap cell (5 cm). Then test it: rapid in-place rotation and strafe with
      clear space ahead, on **metal and granite** (high slip), counting false triggers;
      repeat on wood and concrete as a low-slip control.
      **Target: zero ghosting false positives.** If false triggers appear only on the
      slippery surfaces, that is odometry drift rather than sensor noise, and
      persistence — not the detection thresholds — is the correct knob. Record the
      derived value, the measured drift it came from, and the per-surface counts.
- [ ] **Step 6: Threshold tuning.** Adjust `min_occupancy_fraction` and
      `min_cluster_area_cm2` against the sweep data. Record before/after values and
      what changed.
- [ ] **Step 7: Dynamic-obstacle clearing.** Person walks through frame and leaves;
      confirm marks clear rather than persisting. Nav2's obstacle layer should do this;
      confirm rather than assume.

## Task 12: Latency budget

The defining metric for a stop system, and the one the earlier draft asserted a target
for without any way to measure it.

- [ ] **Step 1: Write `latency_recorder_node.py`.** Subscribe to
      `/poc_fusion/depth_cleaned` (carries the camera stamp verbatim, per Task 4 Step 1)
      and `/costmap_app/obstacle_detected`. On each False→True transition, record
      `now − (stamp of the newest depth frame preceding the transition)`.
- [ ] **Step 2: Collect ≥20 rising edges** by repeatedly presenting an obstacle. Report
      **median and p95**, not a single sample.
- [ ] **Step 3: Record what this covers and what it does not.**
      Covers: preprocessing, projection, costmap update, publish interval, monitor
      evaluation — everything this POC adds.
      Does not cover: the camera's own exposure and internal processing latency, which
      is unrecoverable from message stamps and would need external high-frame-rate
      video to capture. **State the reported figure as a lower bound on true
      physical-entry-to-stop latency, and do not claim otherwise.**
- [ ] **Step 4: Check against the budget.** `publish_frequency` is already raised to
      10 Hz (Task 6 Step 1), which halves the queueing floor from 200 ms to 100 ms and
      makes the target reachable rather than arithmetically excluded — at 5 Hz the
      publish interval alone consumed the entire budget before any processing ran.
      **Target: p95 ≤ 300 ms.** If the measurement misses it, the next lever is the
      Task 4 median filter kernel, **not** a further publish-rate increase; record the
      CPU cost from Task 13 alongside the latency figure so the tradeoff is visible.

## Task 13: Compute budget

- [ ] **Step 1: Record baseline CPU/RAM** with the stock stack running and `poc_fusion`
      stopped.
- [ ] **Step 2: Record fused CPU/RAM** with the full pipeline running.
- [ ] **Step 3: Report the two separately** so the cost attributable to this POC is
      visible rather than inferred. This is also where the design doc §5.2 decision
      (regenerating the cloud so the self-arm mask can run in image space, before
      projection) gets its actual price tag. **Target: sustained average below 80%.**

## Task 14: Graceful degradation

- [ ] **Step 1: Disconnect the camera mid-run.** Confirm the costmap keeps updating from
      the LiDAR, camera marks age out within 0.5s, and the watchdog logs an explicit
      error naming the loss of fusion.
- [ ] **Step 2: Reconnect.** Confirm fusion resumes and a recovery message is logged.
- [ ] **Step 3: Confirm no silent degradation** — the failure must be visible in the log
      without inspecting topics.

## Task 15: Fusion benefit A/B measurement

Tasks 0–14 prove the system works. They do **not** prove the camera helped — LiDAR alone
already stops for obstacles.

- [ ] **Step 1: Create a LiDAR-only control config** differing from the fused config in
      exactly one line (`observation_sources`). Diff the two files and confirm.
- [ ] **Step 2: Run four obstacle classes at 1m, 5 trials each, both configs:**
      1. **Tall box (~30cm)** — control; both sensors should see it.
      2. **Low-profile object (~8–12cm)** — likely below the LD19 scan plane.
      3. **Overhanging object** (pedestal-style form) — **the class that caused the real
         2026-07-22 collision**, and the strongest available motivation.
      4. **Thin vertical obstacle** (chair leg) — LiDAR's strength; included to check
         the camera path does not *degrade* anything.
- [ ] **Step 3: Record false-positive counts per condition** over bare-floor runs.
      Fusion that catches more obstacles but also stops for phantoms is not a win.
- [ ] **Step 4: Write up results honestly, including null results.** A class where
      fusion shows no benefit is legitimate and publishable, and far better than a
      reviewer finding the gap later.

## Task 16: Documentation

- [ ] **Step 1: Finalize `docs/poc_fusion_verification.md`** with every measured value:
      environment facts, TF chain, final camera pose and joint values, tuned thresholds,
      sweep results, latency median/p95, CPU baseline vs fused, A/B results.
- [ ] **Step 2: Write `poc_fusion/README.md`** — how to launch, what each config key
      does, the environment assumptions, and the two known approximations (unrectified
      depth; pipeline latency as a lower bound).
- [ ] **Step 3: State the framing correctly in the writeup.** This POC improves
      **perception for an avoidance planner that already exists** — `avoidance.py`
      already implements mecanum STRAFE, gap finding, and abort-and-replan, fed today
      by LiDAR sectors alone. It is **not** a step toward building lateral avoidance,
      and any text implying otherwise (including the original spec's §11) is wrong.
      Note explicitly that the trials use `proximity_alert`, **not** Hiwonder's stock
      `lidar_app`, which merely happens to be running on the robot.
- [ ] **Step 3a: Disclose the LiDAR model discrepancy in Methodology.** State that the
      LanderPi manual specifies an MS200, that this unit is provisioned
      `LIDAR_TYPE=MS200` but the vendor launch file routes that branch to the LD19
      driver (MS200 driver commented out), and that all sensor characterisation
      therefore cites the LD19. Say plainly that the scan rate does not expose the
      mismatch — 9.87 Hz measured sits inside the manual's 7–15 Hz MS200 band — which
      is exactly why it needs stating rather than leaving a reviewer to find it.
- [ ] **Step 3b: Disclose that the scan is unfiltered.** The vendor launch file omits
      the `laser_filters` chain, so `/scan_raw` is the only scan topic and no range
      clipping, shadow filtering, or speckle removal is applied — to this POC or to any
      trial already collected. Do not describe the LiDAR input as filtered.
- [ ] **Step 4: Record the deferred follow-on** — replacing the sector-clearance input
      to `avoidance.py`'s `_assess()` with a costmap query, behind a flag, on a branch,
      after the dataset is complete.

---

## Deferred / not in this plan

- Feeding the fused costmap into `avoidance.py` (protects the trial dataset).
- Formal checkerboard intrinsic calibration and depth rectification.
- Full Nav2 navigation (global costmap, map server, planner, behavior tree).
- Root-causing the `servo_controller` dropout, and finding a true bus-servo feedback
  path — candidate identified but unverified: service
  `/ros_robot_controller/bus_servo/get_state`
  (`ros_robot_controller_msgs/srv/GetBusServoState`). Task 10 works around this by
  verifying pose through TF instead.
