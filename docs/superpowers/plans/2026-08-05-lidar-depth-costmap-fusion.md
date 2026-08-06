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

### Where source code lives, and why it is not where you would guess

**Verified 2026-08-07: the host project repo is NOT bind-mounted into the container.**
`docker inspect MentorPi` shows only `/dev`, `/home/pi/docker/tmp → /home/ubuntu/shared`,
pulse, X11, and dbus. Worse, `/home/ubuntu/ros2_ws` **is itself a git repo — Hiwonder's
vendor one** (`git log`: "update command", "update .typerc"). Anything written to
`/home/ubuntu/ros2_ws/src/` lands in the vendor's history, not this project's, and is
invisible to code review.

So there is no single path that is both git-committable and ROS-runnable. The split:

| | |
|---|---|
| **Author and commit here** | `/home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion/` — host, top level, beside the existing `proximity_alert/` package |
| **Build and run here** | `/home/ubuntu/ros2_ws/src/poc_fusion/` — container, populated by the deploy script, never hand-edited |

- **The host repo is the source of truth.** Every edit, every commit, every review diff
  happens there. The container copy is a build artifact.
- **Deploy before any on-robot step** via `scripts/deploy_poc_fusion.sh` (Task 2 Step 5).
  If you edit in the container and forget to copy back, the next deploy silently
  overwrites your work — the sync is one-directional by design.
- **Pure-logic tests need no container at all.** Confirmed on the host:
  `cd poc_fusion && python3 -m pytest test/ -q` — this is how the existing
  `proximity_alert` suite runs (49 tests passing). Task 3 is entirely off-robot.

### Paths that do not exist — do not use them

- **`~/ros_ws`** — does not exist anywhere.
- **Host `~/ros2_ws`** — exists, but is a **stale clone**, and is *not* the project repo
  either. Editing it changes nothing the robot runs and nothing that gets committed.

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
| Depth encoding | **`mono16`** — *not* `16UC1`. Must be relabelled before `depth_image_proc`, see Task 4 Step 1a |
| Camera pose (operating) | ~**40°** below horizontal, 0.246 m above floor. Coverage ceiling ~**0.94 m**; nothing above 0.246 m ever in frame (Task 10). **Every task except 10a assumes this pose.** |
| Camera pose (overhang test only) | 5.4° **up**, 0.349 m above floor. Blind below ~0.97 m. Used only for Task 15 class 3, then reverted (Task 10a) |
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
- **Task 3 is strict TDD — test first, no exceptions.** Every function in
  `lib/window_geometry.py` and `lib/obstacle_detection.py` gets its failing test
  written and *observed failing* before any implementation exists. This is the math
  that decides whether the robot stops; a test written after the code passes
  immediately and proves nothing about whether it can catch the bug. The step order in
  Task 3 encodes this and must not be reordered for convenience.
- **Author on the host, deploy to the container.** No task writes source directly into
  `/home/ubuntu/ros2_ws/src/`. See "Where source code lives" above.
- **Every on-robot verification step records its output** into
  `docs/poc_fusion_verification.md`. "It looked right in RViz" is not a result.

## File Structure

All paths relative to the host repo root, `/home/pi/Desktop/LanderPi-Proximity-Alert/`.

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

scripts/deploy_poc_fusion.sh         # host poc_fusion/ -> container src/, one-directional
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
      This is the same procedure Task 10 uses to confirm the 40° pose each session,
      and Task 10a uses to confirm the overhang-test pose (where the z-component flips
      sign, to +0.094).

## Task 2: Package scaffolding

- [ ] **Step 1: Create the `poc_fusion` package at the host repo root**
      (`/home/pi/Desktop/LanderPi-Proximity-Alert/poc_fusion/`, `ament_python`), with the
      file structure above and empty config files. **Not** in
      `/home/ubuntu/ros2_ws/src/` — see "Where source code lives". Mirror the layout and
      `setup.py`/`setup.cfg` conventions of the sibling `proximity_alert/` package rather
      than inventing new ones.
- [ ] **Step 2: Declare dependencies** in `package.xml` — **all of them, including the
      ones installed by hand in Task 0.** Task 0's `apt-get` makes them present on
      *this* container only; `package.xml` is what makes `rosdep install` reproduce the
      environment when Dr. Mu or Haotian rebuild on a fresh container. A missing entry
      is skipped silently and surfaces later as a runtime failure with no obvious cause.
      ```xml
      <exec_depend>rclpy</exec_depend>
      <exec_depend>sensor_msgs</exec_depend>
      <exec_depend>nav_msgs</exec_depend>
      <exec_depend>geometry_msgs</exec_depend>
      <exec_depend>std_msgs</exec_depend>
      <exec_depend>visualization_msgs</exec_depend>
      <exec_depend>cv_bridge</exec_depend>
      <exec_depend>tf2_ros</exec_depend>
      <exec_depend>nav2_costmap_2d</exec_depend>
      <exec_depend>nav2_lifecycle_manager</exec_depend>
      <exec_depend>depth_image_proc</exec_depend>
      <exec_depend>image_proc</exec_depend>
      <exec_depend>python3-scipy</exec_depend>
      <exec_depend>python3-numpy</exec_depend>
      ```
- [ ] **Step 2a: Verify the declaration is actually complete** — `rosdep check
      --from-paths src/poc_fusion` should report no missing keys. Do not treat "it runs
      on my container" as evidence; that is exactly the case this step guards against.
- [ ] **Step 3: Register entry points** in `setup.py`: `depth_preprocess_node`,
      `costmap_stop_monitor_node`, `latency_recorder_node`. Add `config/` and `launch/`
      to `data_files`.
- [ ] **Step 4: Write `scripts/deploy_poc_fusion.sh`.** Copies the host
      `poc_fusion/` into `/home/ubuntu/ros2_ws/src/poc_fusion/` in the `MentorPi`
      container. Requirements:
      - **One-directional, host → container.** Never copies back. State this in a
        comment at the top of the script, because the failure it prevents (editing in
        the container, then losing it to the next deploy) is silent.
      - Deletes files in the destination that no longer exist in the source, so a
        renamed or removed module does not linger and get imported.
      - Leaves `build/`, `install/`, `log/`, and `__pycache__` alone.
      - Ends by `chown`ing the destination to `ubuntu:ubuntu` — a `docker cp` from the
        host lands root-owned, and a subsequent `colcon build` as `ubuntu` then fails
        with an error that does not name the real cause.
      Verify by round-trip: deploy, then `docker exec -u ubuntu MentorPi diff -r` the
      two trees and confirm no differences.
- [ ] **Step 5: Build and confirm the package is discoverable.** Deploy first
      (Step 4), then inside the container as `ubuntu`:
      `colcon build --packages-select poc_fusion`, then
      `ros2 pkg executables poc_fusion`. **Never `colcon build` as root.**

## Task 3: Pure geometry and detection logic (off-robot, strict TDD)

Everything here is arrays and numbers — no ROS, no hardware, no container. Run with
`cd poc_fusion && python3 -m pytest test/ -q` on the host.

> **This task is strict test-driven development and the step order below is the
> requirement, not a suggestion.** Steps 1–3 write tests against functions that do not
> exist yet and watch them fail; Steps 4–6 make them pass. This is the arithmetic that
> decides whether the robot stops for an obstacle, and it is the one place in this plan
> where a silent error produces confident, plausible, wrong numbers rather than a
> crash — the row/column-slice design this replaced would have shipped a detection
> window that silently swung to the robot's back as it turned, while still returning
> sensible-looking values. A test written after the implementation passes on first run
> and demonstrates nothing about its ability to catch that.
>
> **Required of every function here:** the failing test exists first, you run it, and
> you confirm it fails *for the intended reason* (function missing / wrong value) and
> not from a typo or import error. Record the observed failure output in the task
> report. A test that passed the first time you ran it is not evidence — delete the
> implementation and start that function over.

### Steps 1–3: RED — write the failing tests first

- [ ] **Step 1: `test/test_window_geometry.py` — tests for `window_mask(...)`,**
      written against the signature below before the module exists. Cover: a cell
      directly ahead is in the mask; a cell behind is not; a cell beyond `forward_m`
      is not; a cell outside `half_width_m` is not; a non-centred `origin` is honoured
      (do **not** assume the robot sits at grid centre). Run them; confirm they fail on
      the missing module.
- [ ] **Step 2: `test/test_window_geometry.py` — the rotation test. The important one.**
      Place a synthetic lethal cell 1 m directly ahead of the robot in `odom`. Sweep
      `ryaw` through 0, ±45°, ±90°, 180°, **re-deriving the cell's `odom` position each
      time so it stays physically ahead**: assert it is always in the mask. Then hold
      the cell fixed in `odom` and rotate the robot away: assert it leaves the mask.
      This is the regression test for the bug the earlier design would have shipped.
      Also: a test for `area_cm2_to_cells(area_cm2, resolution)` asserting the same
      physical area yields different cell counts at 0.05 and 0.10 resolution.
- [ ] **Step 3: `test/test_obstacle_detection.py` — tests before the module exists.**
      Empty grid → no stop. Fully-lethal window → stop. **A diagonal-only chain forms
      one cluster under 8-connectivity** (this is the assertion that pins the
      connectivity choice — under 4-connectivity it fragments and the trigger never
      fires). Sub-threshold speckle → no stop. **Unknown (`-1`) cells are treated as
      not-lethal — assert this explicitly**, since `-1 >= lethal` is false but the
      intent must be pinned against a future change to unsigned handling.
      Run all three files; confirm every test fails, and that each fails on the missing
      import rather than on a mistake in the test.

### Steps 4–6: GREEN — minimal implementations

- [ ] **Step 4: `lib/window_geometry.py` — `window_mask(...)`.**
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
      Also `area_cm2_to_cells(area_cm2, resolution)` — physical area threshold → cell
      count, computed from the live resolution so the threshold means the same thing
      if resolution changes.
- [ ] **Step 5: `lib/obstacle_detection.py`.**
      - `occupancy_fraction(values, lethal_threshold)` — `values` is the 1-D array of
        masked cell costs.
      - `largest_cluster_cells(grid, mask, lethal_threshold)` — `scipy.ndimage.label`
        on `(grid >= lethal) & mask` with **8-connectivity**, returns the largest
        component's size. 8-connectivity because point-cloud marks arrive as speckled
        blobs that frequently touch only diagonally; 4-connectivity fragments them
        below threshold and the trigger never fires.
      - `should_stop(...)` — OR of the fraction and area conditions.
      Write the minimum that turns the Step 1–3 tests green. No extra parameters, no
      configurability the tests do not demand.
- [ ] **Step 6: Run the full suite and confirm it is green and clean.**
      `cd poc_fusion && python3 -m pytest test/ -q` on the host — no container needed.
      Output must be pristine: no warnings, no errors, no skips.
      **In the task report, record for each function: the failing output observed at
      RED and the passing output at GREEN.** A report that shows only the green run has
      not demonstrated TDD and the task is not complete.

## Task 4: Depth preprocessing node

- [ ] **Step 1: Node skeleton.** Subscribe to the raw depth image, `cv_bridge` to numpy,
      republish unchanged to `/poc_fusion/depth_cleaned`. **Preserve `header` exactly** —
      stamp and `frame_id` must pass through verbatim. The stamp is the camera frame
      time and Task 12's latency measurement depends on it surviving this node; the
      `frame_id` must remain `depth_camera_link` or the costmap transforms the cloud
      from the wrong frame.
- [ ] **Step 1a: Re-encode `mono16` → `16UC1`. This is a blocker, not a nicety.**
      The driver publishes `encoding: mono16` (verified: 640×400, `step: 1280`, so
      16-bit single channel). **`depth_image_proc::PointCloudXyzNode` accepts only
      `16UC1` (millimetres) and `32FC1` (metres)** — it compares the encoding string
      and throws "Depth image has unsupported encoding" on anything else. `mono16` is a
      distinct string from `16UC1` even though the buffer layout is identical, so
      feeding the driver topic through unchanged makes Task 5 fail immediately.
      Since this node already republishes, set the outgoing `encoding` to `16UC1`.
      No pixel data changes — it is a relabel of an identical buffer.
      **Verify before moving on:** `ros2 topic echo /poc_fusion/depth_cleaned --no-arr`
      shows `encoding: 16UC1`, and confirm the units really are millimetres by holding
      an object at a tape-measured distance and reading the raw pixel value.
- [ ] **Step 2: Invalid-pixel masking — integer semantics, no NaN check.**
      `mono16` is an **unsigned integer** type, so NaN cannot occur; invalid pixels are
      exactly **0**. An earlier revision of this step said "zero and NaN" — the NaN
      branch is unreachable dead code for this encoding and must not be written as
      though it guards anything. If the driver is ever reconfigured to `32FC1`, both
      `0.0` and `NaN` become possible and this step changes: **assert the encoding at
      startup and fail loudly rather than silently mis-masking.**
      Log the invalid fraction periodically; a sudden jump is the first sign of a
      camera fault.
- [ ] **Step 3: Self-arm ROI masking from YAML.** Read the ROI from
      `config/depth_preprocess_params.yaml` — **not hard-coded**, because it must be
      re-derived whenever the arm pose changes (Task 10a needs a second profile for
      the overhang-test pose; keep them separate). Start with a
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
      Note `window_forward_m: 1.0` slightly exceeds the camera's 0.94 m ceiling, so the
      far ~6 cm is LiDAR-only. Intentional — the window is sized against stopping
      distance, not camera coverage — but record it so a trigger at ~1 m is never
      written up as a camera detection.
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
      `Twist` per costmap update (10 Hz) does not *hold* a stop on this robot. Use the
      existing `proximity_alert` `motion_watchdog` node rather than reimplementing
      stop-hold here. Verify a stop actually persists before calling this task done.
- [ ] **Step 4: No avoidance maneuver.** Stop only — consuming the costmap in
      `avoidance.py` is deferred to protect the trial dataset (design doc §11).

## Task 10: Record camera coverage bounds at the as-is 40° pose

**Decision (2026-08-06): the camera stays at its current ~40° pitch for this POC.**
Re-pitching would mean pairing a phone to the robot for the WonderPi app (the ROS servo
path being unreliable), and the robot is worked on over SSH. That is disproportionate
effort for a proof of concept. This task no longer moves the arm — it records what the
existing pose can and cannot see, so the validation tasks test reachable things.

- [ ] **Step 1: Record the live pose.** `ros2 run tf2_ros tf2_echo base_footprint
      depth_camera_link`. Expected: translation `[0.105, 0.021, 0.246]`, optical forward
      axis (third column of the rotation matrix) `(0.766, −0.005, −0.642)` → **39.9°
      below horizontal**, camera **0.246 m** above the floor. Re-check at the start of
      every session; a drift beyond ±1° means the arm has sagged and prior measurements
      need re-taking.
- [ ] **Step 2: Record the derived coverage envelope** in the verification doc, from the
      verified intrinsics (V-FOV 50.4°, H-FOV 74.1°) and the pose above:
      - top ray **14.7°** below horizontal, bottom ray 65.1°
      - **floor visible from 0.114 m to 0.938 m**
      - heights in frame: 0.3 m → 0–0.167 m · 0.5 m → 0–0.115 m · 0.7 m → 0–0.062 m ·
        0.9 m → 0–0.010 m · **≥1.0 m → nothing (entirely below floor level)**
      **Two hard limits follow, and every later task depends on them:**
      1. **Max camera range ≈ 0.94 m.** Tests at 1 m or 2 m measure the LiDAR only.
      2. **Nothing above 0.246 m is ever in frame, at any distance** — the top ray
         descends, so the highest visible point is at the camera itself. Floor-standing
         objects are still detected via their base; true overhangs are not detectable
         at all.
- [ ] **Step 3: Verify the envelope empirically before trusting it.** Place an object at
      0.5 m and at 1.2 m. Confirm the first produces cloud points and the second produces
      none. If the 1.2 m object *does* appear, the pose or intrinsics assumption is wrong
      and Step 2 must be recomputed before proceeding.
- [ ] **Step 4: Confirm the self-arm ROI matches this pose.** The ROI in
      `depth_preprocess_params.yaml` is pose-specific; since the pose is unchanged from
      when it was measured, it stays valid. Re-derive it only if Step 1 shows sag.
- [ ] **Step 5: Record the consequences for the A/B study** so they are not rediscovered
      mid-experiment: Task 15 classes 1/2/4 move from 1 m to **0.6 m** and stay at this
      pose; **class 3 (overhanging obstacle) is not runnable at this pitch** and moves
      to the separate pose in Task 10a — see Task 15 Step 2.

## Task 10a: Command and verify the overhang-test pose (design doc §4d)

**This is a second, test-only pose used solely to make Task 15's class 3 runnable.** It
does not replace the ~40° operating pose. The arm returns to 40° immediately afterwards,
and every other task in this plan runs at 40°.

Superseded plan text, for the record: an earlier revision deferred this as a "20° pitch
change" to be done via the WonderPi app. Design doc §4d supersedes that — once the arm
is repositioned through ROS rather than a paired phone, 20° is no longer the right
target (it gains only 2.7 cm of height over the current pose), and a fuller
reconfiguration is chosen instead.

**Target pose**, from the forward-kinematics model validated against the live
current-pose transform (computed pitch 39.94° vs. measured 39.9°; forward axis matched
to three decimals):

| | |
|---|---|
| Joints | `joint2 ≈ 0.3°`, `joint3 ≈ 0.3°`, `joint4 ≈ −84.7°` (grid-search resolution 5°) |
| Camera height | **0.349 m** above floor (vs 0.246 m at 40°) |
| Pitch | **5.4° up** from horizontal (vs 39.9° down) |
| Top ray | **+30.6°** above horizontal |
| Bottom ray | 19.8° below horizontal |

- [ ] **Step 1: Refine the joint target to ~0.5° resolution** before generating the
      trajectory goal. The 5° grid search above located the pose; it did not optimise
      it. Re-run the same FK sweep at finer resolution around the listed values and
      record the refined triple. This changes nothing structural — if the refined pose
      differs from the table by more than ~2° in resulting pitch, stop and re-check the
      FK model rather than trusting the new number.
- [ ] **Step 2: Command the pose via `FollowJointTrajectory`** through the
      `servo_controller` action path. **Known-flaky (Task 4 environment notes / design
      §4): the `arm_controller`, `gripper_controller`, and `controller_manager` nodes
      dropped out of the ROS graph once after a goal was sent and needed manual
      relaunch.** If that happens, relaunch them manually and re-send. Expect it; it is
      not a new bug.
      **Do NOT run `.stop_ros.sh` at any point.** It exists only inside the container
      (`/home/ubuntu/.stop_ros.sh`) and its entire contents are
      `ps aux | grep ros | ... | kill -9` — an indiscriminate kill of every ROS process,
      including `robot_state_publisher` (which would destroy the very
      `base_link → depth_camera_link` chain this task verifies), the Aurora driver, and
      the LiDAR driver. It also does not do what it is assumed to: HX-06L bus servos
      hold position from their own internal control loop while powered and
      torque-enabled, regardless of ROS. **To hold a pose, simply stop sending arm
      commands.**
- [ ] **Step 3: Verify the achieved pose — and not with TF alone.**
      `robot_state_publisher` computes TF *from* `/joint_states`, so they are one source
      of truth, not two: if `/joint_states` goes stale or resets, `tf2_echo` will
      confidently report a pose the arm is not in, and nothing in the graph will
      disagree. Use all three:
      1. `ros2 run tf2_ros tf2_echo base_footprint depth_camera_link` — expect height
         **0.349 m ± 0.02**, forward-axis z-component **+0.094 ± 0.03** (5.4° up;
         note the **sign flip** from the 40° pose's −0.642 — the camera now points
         above horizontal, and a negative z here means the move did not take).
      2. The camera image itself — the test object must be visibly in frame.
      3. `/ros_robot_controller/bus_servo/get_state`
         (`ros_robot_controller_msgs/srv/GetBusServoState`) for true servo feedback if
         it is available; this service is identified but not yet verified to work.
- [ ] **Step 4: Re-derive the self-arm ROI for this pose.** The ROI in
      `depth_preprocess_params.yaml` is **pose-specific and the 40° value is invalid
      here** — the arm is in a completely different configuration, so the gripper
      occupies a different image region. Read the new ROI off `/poc_fusion/debug_image`
      and record it as a *separate* config profile; do not overwrite the 40° one, since
      every other task needs it back.
- [ ] **Step 5: Record the coverage envelope for this pose** in the verification doc:
      floor not visible until **~0.97 m** (bottom ray is below horizontal but shallow),
      and height ceiling **growing** with distance — 0.65 m at 0.5 m, 0.94 m at 1.0 m,
      1.53 m at 2.0 m.
      **Record the near-field blind zone as a hard constraint:** below ~0.97 m this pose
      sees nothing at floor level, a real regression against the 40° pose. **No
      near-field or low-obstacle test is ever run at this pose** — not Task 11's sweep,
      not Task 15 classes 1/2/4. Running them here would attribute a pose limitation to
      the sensor.
- [ ] **Step 6: Record the class-3 visible window.** For a ~0.75 m test object at
      0.349 m camera height, the object is in frame only at range **≥ 0.68 m** — closer
      than that, the required look-up angle exceeds the +30.6° top ray and the object
      scrolls out of the top of frame. **This is expected geometry, not a fault to
      debug.** Task 15 places the object at 1.0–1.5 m accordingly.
- [ ] **Step 7: Return the arm to the 40° pose and re-verify** via Task 10 Step 1
      (expect forward-axis z back to −0.642, height back to 0.246 m), and restore the
      40° self-arm ROI. **Do not leave the robot in the test pose** — every other task
      in this plan assumes 40°, and a silently-changed pose invalidates their results
      without any error appearing.


## Task 11: Validation and tuning

- [ ] **Step 1: Ground-plane check.** Bare floor ahead, robot stationary, at the 40°
      pose. Confirm no cells marked occupied. Tune `min_obstacle_height` until clean.
      Record the final value and how many iterations it took.
- [ ] **Step 1a: If floor false positives form a gradient toward the far edge, suspect
      arm sag before touching `min_obstacle_height`.** The floor's projection into
      `odom` depends entirely on the accuracy of the `depth_camera_link` pitch, so a
      small physical sag lifts the apparent floor with distance and looks exactly like
      a threshold that is set too low. Raising the threshold to compensate would blind
      the camera to genuinely low obstacles — trading a cosmetic fix for the detection
      class the POC exists to demonstrate.
      **Diagnostic:** a *uniform* offset across the window is a threshold problem; a
      *gradient growing with distance* is a pitch problem. Re-read Task 10 Step 1 and
      compare against the recorded 39.9°.
      Magnitudes at this pose (max floor range 0.94 m): **1° sag → 1.6 cm lift,
      2° → 3.3 cm** — both under one 5 cm cell, so 40° is a forgiving pose for this.
      It would matter far more at the Task 10a pose, where the floor is visible to
      several metres and the same 2° becomes ~7 cm at 2 m — another reason that pose is
      test-only. Record the check either way.
- [ ] **Step 2: Self-mask check.** Confirm the gripper is excluded and that real
      obstacles near frame edges are **not** over-masked.
- [ ] **Step 3: Detection sweep — ranges bounded by Task 10's coverage envelope.**
      Object at **0.3 m, 0.5 m, 0.7 m, 0.9 m**, plus **0.6 m** at the left and right
      horizontal FOV edges. Record detection outcome per position.
      **Do not test at 1 m or 2 m.** At the 40° pose the FOV is entirely below floor
      level beyond ~0.94 m, so those points are outside the camera's line of sight and
      would record a geometric impossibility as a sensor miss.
      Also record how much of the object's height is in frame at each distance
      (0.3 m → 16.7 cm, 0.5 m → 11.5 cm, 0.7 m → 6.2 cm, 0.9 m → 1.0 cm). A mark built
      from a 1 cm sliver at 0.9 m is a far weaker detection than one from 16.7 cm at
      0.3 m, and `min_cluster_area_cm2` must accommodate the weakest case that still
      needs to trigger — expect this to be the binding constraint in Step 6.
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
      the LiDAR, camera marks age out within **`observation_persistence` (0.3 s per
      Task 6 Step 2, or whatever value Task 11 Step 5a derived)**, and the watchdog logs
      an explicit error naming the loss of fusion.
- [ ] **Step 2: Reconnect.** Confirm fusion resumes and a recovery message is logged.
- [ ] **Step 3: Confirm no silent degradation** — the failure must be visible in the log
      without inspecting topics.

## Task 15: Fusion benefit A/B measurement

Tasks 0–14 prove the system works. They do **not** prove the camera helped — LiDAR alone
already stops for obstacles.

- [ ] **Step 1: Create a LiDAR-only control config** differing from the fused config in
      exactly one line (`observation_sources`). Diff the two files and confirm.
- [ ] **Step 2: Run the obstacle classes at 0.6 m, 5 trials each, both configs.**
      **0.6 m, not 1 m** — 1 m is beyond the camera's 0.94 m coverage ceiling at the 40°
      pose (Task 10 Step 2), so the "fused" and "LiDAR-only" configs would be comparing
      LiDAR against LiDAR and the study would return a guaranteed null by construction.
      1. **Tall box (~30cm)** — control; both sensors should see it. The camera sees its
         bottom ~9 cm at 0.6 m, which is ample to mark.
      2. **Low-profile object (~8–12cm)** — likely below the LD19 scan plane and fully
         in frame at 0.6 m. **This class carries the POC at the 40° pose.**
      3. **Overhanging object** (pedestal-style form) — the class that caused the real
         2026-07-22 collision, and the strongest available motivation.
         **⚠ Not runnable at the 40° pose, and not run at 0.6 m.** Nothing above the
         camera's 0.246 m height enters the frame at any distance there (Task 10
         Step 2). **Run this class on its own, at the Task 10a overhang-test pose, with
         the object at 1.0–1.5 m** (inside the 0.68 m visible-window bound from Task 10a
         Step 6), then return the arm to 40° per Task 10a Step 7 before anything else.
         **Do not run classes 1/2/4 at the 10a pose** — its sub-0.97 m blind zone would
         make the camera miss them for reasons that have nothing to do with fusion.
         The distances are deliberately not comparable across classes: each is chosen
         against its own pose's coverage ceiling, and the writeup must say so rather
         than presenting a single distance for the study.
      4. **Thin vertical obstacle** (chair leg) — LiDAR's strength; included to check
         the camera path does not *degrade* anything.
- [ ] **Step 3: Record false-positive counts per condition** over bare-floor runs.
      Fusion that catches more obstacles but also stops for phantoms is not a win.
- [ ] **Step 4: Write up results honestly, including null results.** A class where
      fusion shows no benefit is legitimate and publishable, and far better than a
      reviewer finding the gap later.
- [ ] **Step 5: State the two-pose structure explicitly in the writeup.** Classes 1/2/4
      ran at the stock ~40° fall-prevention pitch, which bounds their coverage to
      ~0.94 m and to objects below 0.246 m. Class 3 required a **second, test-only pose**
      (Task 10a) to be observable at all, with its own narrower visible window
      (≥0.68 m) and a sub-0.97 m near-field blind zone that rules that pose out for
      general use. **The overhang result is therefore pose-specific and is not evidence
      that the operating pose sees overhangs.** Presenting the two together as one
      uniform experiment, or quietly omitting which pose produced which class, is the
      specific failure mode to avoid.

## Task 16: Documentation

- [ ] **Step 1: Finalize `docs/poc_fusion_verification.md`** with every measured value:
      environment facts, TF chain, **both poses** (the as-is 40° operating pose and the
      Task 10a overhang-test pose) each with its derived coverage envelope, tuned
      thresholds, sweep results, latency median/p95, CPU baseline vs fused, and A/B
      results **labelled with which pose produced each class**.
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
- Root-causing the `servo_controller` dropout. Task 10a works around it by expecting the
  dropout and relaunching manually.
- Verifying the true bus-servo feedback path — service
  `/ros_robot_controller/bus_servo/get_state`
  (`ros_robot_controller_msgs/srv/GetBusServoState`) is identified but has not been
  confirmed to work. Task 10a Step 3 uses it opportunistically and does not depend on
  it, cross-checking against the camera image instead. **Note TF is not an independent
  check** — `robot_state_publisher` derives it from `/joint_states`.
- Adopting the Task 10a pose for general operation. Its sub-0.97 m near-field blind zone
  makes it unsuitable; it exists only to make Task 15 class 3 observable.
