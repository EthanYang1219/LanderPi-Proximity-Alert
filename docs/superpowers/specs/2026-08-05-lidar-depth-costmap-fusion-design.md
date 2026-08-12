# LanderPi Depth Camera + LiDAR Costmap Fusion — Proof of Concept Design

**Date:** 2026-08-05 (revised 2026-08-06 against live-robot verification)
**Author:** Ethan (with Claude)
**Status:** Revised per second review — environment claims now verified on hardware

## 0. Verified environment facts

Everything in this section was measured on the live robot on 2026-08-06, not
taken from a datasheet or a prior document. Where an earlier draft of this spec
asserted something different, the earlier draft was wrong.

| Fact | Verified value | How |
|---|---|---|
| LiDAR model | **LD19 driver**, though the platform is configured `LIDAR_TYPE=MS200` — see §0a | `lidar.launch.py`, running process env |
| LiDAR topic | **`/scan_raw`** (no `/scan` exists) — see §0b | `ros2 topic list` |
| LiDAR rate | **9.87 Hz** | `ros2 topic hz /scan_raw` |
| Depth topic | `/ascamera/camera_publisher/depth0/image_raw` | `ros2 topic list` |
| Depth rate | **14.7 Hz** | `ros2 topic hz` |
| Depth resolution | 640×400 | `camera_info` |
| `camera_info` | **valid**, `k = [423.92, 0, 319.30; 0, 424.83, 190.86; 0,0,1]` | `ros2 topic echo` |
| Distortion | `plumb_bob`, `d = [0.0572, -0.0914, 0.0006, -0.0002, 0.0321]` — **non-zero** | `camera_info` |
| Camera frame | **`depth_camera_link`** | topic headers |
| TF chain | `base_link → link1..link4 → camera_connect_link → depth_cam_link → depth_camera_link` — live, arm-driven | `tf2_echo`, `/tf_static`, URDF |
| `odom → base_link` | resolves | `tf2_echo` |
| Driver point cloud | `/ascamera/camera_publisher/depth0/points` **already published** | `ros2 topic list` |
| `depth_image_proc` | **NOT installed** | `ros2 pkg prefix` → not found |
| `nav2_costmap_2d` | installed, executable name `nav2_costmap_2d` | `ros2 pkg executables` |
| scipy / OpenCV | 1.8.0 / 4.10.0 | `python3 -c import` |
| Runtime | ROS 2 Humble inside Docker container **`MentorPi`**, workspace `/home/ubuntu/ros2_ws` | `docker ps` |

### 0a. LiDAR model discrepancy — must be disclosed in the paper's Methodology

The Hiwonder LanderPi manual lists the onboard LiDAR as an **MS200**. This robot runs an
**LD19** driver. That is not an observation error, and it is not a case of the manual
being loosely worded — the vendor's own launch file resolves it:

```python
# peripherals/launch/lidar.launch.py
lidar_type = os.environ['LIDAR_TYPE']
if lidar_type == 'MS200':
    # lidar_launch_path = os.path.join(..., 'launch/include/ms200_scan.launch.py')
    lidar_launch_path = os.path.join(..., 'launch/include/ldlidar_LD19.launch.py')
elif lidar_type == 'LD19':
    lidar_launch_path = os.path.join(..., 'launch/include/ldlidar_LD19.launch.py')
```

The running bringup process has **`LIDAR_TYPE=MS200`** in its environment. The `MS200`
branch has its own driver **commented out** and force-routed to the LD19 driver. Both
`ms200_scan.launch.py` and `ldlidar_LD19.launch.py` ship in the image; only the LD19 one
is reachable. The node in the graph is named `/LD19`.

So: **the unit is provisioned as an MS200 and driven as an LD19** — a hardware revision
absorbed in software without a config change. Code against LD19 behaviour, because that
is what runs.

**Why the paper must state this.** A reviewer checking the LanderPi spec sheet will
expect MS200 characteristics and will flag "LD19" as an error. Worse, the discrepancy is
not detectable from the scan rate: the manual quotes 7–15 Hz for the MS200, and the
measured 9.87 Hz sits inside that band, so rate agreement is not evidence of model
agreement. Any sensor characterisation in the paper must cite the LD19, with the
provisioning mismatch and the launch-file mechanism above stated in Methodology.

### 0b. Why `/scan` does not exist

Same launch file. It constructs a `laser_filters` `scan_to_scan_filter_chain` node that
would remap `scan_raw` → `scan`, then **omits it from the returned `LaunchDescription`**
(the entry is commented out). So the raw driver topic `/scan_raw` is the only scan topic,
and **no laser filtering of any kind is applied** — no range clipping, no shadow filter,
no speckle removal.

Two consequences. The costmap must consume `/scan_raw` (§5.3). And the paper should not
describe the LiDAR input as filtered, because it isn't — unfiltered scan is the input to
every trial already collected.

**Frame convention.** `base_link` follows REP-103: **+x forward, +y left, +z up**.
"Forward" is therefore not something to be discovered empirically — it is fixed by
convention. Any design that needs to measure which way is forward has an indexing
bug, not a calibration problem (see §5.4).

## 1. Background

LanderPi's LiDAR-based proximity alert system (LD19 LiDAR + I2C buzzer) is already
integrated and is the trigger used for the surface-material transit trials (granite,
concrete, wood, metal) currently being analyzed by Haotian. This spec covers a separate,
standalone proof of concept: fusing the arm-mounted Aurora 930 Pro depth camera with the
LiDAR through a Nav2 local costmap, to demonstrate multi-sensor obstacle **perception**.

**This POC explicitly does not feed into, replace, or modify the trial dataset or the
`proximity_alert` code path used to collect it.** It is a separate demonstration to
document as a working extension (e.g. in a future-work or supplementary section of the
paper), not a new data-collection method.

### 1a. Relationship to the existing avoidance stack

**Correction to the previous draft.** An earlier version of this spec described the
trial runs as using "the simple `lidar_app` obstacle-avoidance node (straight-line drive,
turn away from obstacle, no map)." That is wrong on two counts:

1. `lidar_app` is Hiwonder's stock demo application, which happens to be running on the
   robot. It is not this project's code and the trials do not use it.
2. The trials run on **this project's own `proximity_alert` package** —
   `path_tracker.py` driving a state machine in `avoidance.py` — which already
   implements mecanum **STRAFE** (lateral sidestep), gap finding, cross-track
   return-to-line correction, and abort-and-replan when a committed flank closes
   mid-maneuver.

This materially changes what the POC contributes. The robot **already has** holonomic
lateral avoidance and a return-to-path controller. What it does not have is a
perception layer good enough to feed them reliably — the LD19's fixed 2D scan plane
missed a pedestal desk entirely on 2026-07-22 and the robot made contact with it.

So the honest framing is:

> This POC improves **perception** for an avoidance planner that already exists.
> It is not a step toward building that planner.

The POC's own response to a detection is still a full stop plus buzzer, because
wiring fused perception into the live avoidance state machine would mean touching
trial code (§3). Producing the fused costmap and demonstrating that it sees what the
LiDAR alone misses is the deliverable; consuming it is the follow-on.

## 2. Goals

- Demonstrate improved obstacle **perception** through multi-sensor fusion — LiDAR and
  depth-camera observations combined into a single occupancy representation (a Nav2 local
  costmap).
- Quantify what the camera actually adds over LiDAR alone (§8, and the A/B measurement
  it gates), since "it stops for obstacles" is already true without the camera.
- Do this without requiring a pre-built map of the room and without adopting Nav2's
  global planner/behavior-tree navigation stack.
- Keep the build isolated from all existing trial code, launch files, and logging.

## 3. Non-goals

- Not a replacement for the LiDAR-only trigger used in surface trials.
- Not full Nav2 goal-directed navigation (no map server, no global costmap, no path
  planner, no behavior tree).
- Not collecting comparative data for the paper's core dataset.
- Not modifying `avoidance.py` / `path_tracker.py` to consume the fused costmap. That
  is the follow-on phase (§11) and it is deferred purely to protect the dataset.

## 4. Background constraints

- Camera: Deptrum Aurora 930 Pro, mounted on the arm (URDF: `link4 →
  camera_connect_link → depth_cam_link → depth_camera_link`). Structured-light depth,
  working range 30–300cm, FOV H71°×V46°, 640×400 @ 14.7Hz measured. Driver runs
  natively on the Pi 5 and publishes depth, `camera_info`, and a point cloud.
- **Because the camera is arm-mounted, the arm must be fully locked for any run using
  this POC:**
  - Command the arm to one fixed, known joint configuration before the run starts.
  - Verify the resulting camera pitch against TF (§4a) immediately before collection.
  - No commanded arm motion for the run's duration. A *sustained* deviation beyond
    tolerance invalidates that run's TF assumption; discard the run.
- **Known blocker on the arm control path (2026-08-06).** During environment probing,
  the `servo_controller` package's nodes (`arm_controller`, `gripper_controller`,
  `controller_manager`) dropped out of the ROS graph after a `FollowJointTrajectory`
  goal was sent, and had to be relaunched manually. **Immediately after the relaunch**
  `/joint_states` reported all-zero arm positions while the arm was physically unmoved.

  **Correction to an earlier draft of this section:** that all-zero state was
  transient, not standing. Re-checked live, `/joint_states` now reports
  `joint2 = 0.9634`, `joint3 = −1.5499`, `joint4 = −1.6755` — the correct physical
  pose. The earlier text described a restart artifact as a permanent property of the
  topic and overstated the blocker.

  **The real limitation, which survives that correction:** `/joint_states` on this
  platform echoes commanded state, not bus-servo feedback. And because
  `robot_state_publisher` computes TF *from* `/joint_states`, **TF is not an
  independent check on it** — if `/joint_states` goes stale or resets, `tf2_echo`
  will confidently report the pose of the zero configuration while the arm sits
  somewhere else entirely, and nothing in the graph will disagree. Any pose
  verification that relies only on TF is therefore verifying the command, not the
  hardware. Genuinely independent checks:
  - the camera image itself (what is actually in frame), and
  - true bus-servo feedback via `/ros_robot_controller/bus_servo/get_state`
    (`ros_robot_controller_msgs/srv/GetBusServoState`) — identified, not yet verified.

  This is why the camera-pose task is sequenced late in the plan rather than used as
  an entry gate.
- The existing depth-camera code on this robot (fall-prevention, bridge-crossing) reads
  narrow, pre-calibrated ground-plane ROIs — there is no existing general obstacle-marking
  or camera→costmap integration to build on.

### 4a. Camera pose: 40° downward (as-is) — POC decision

**Decision (2026-08-06): keep the current ~40° pose. Do not re-pitch for this POC.**
The robot is accessed over SSH; the practical route to re-posing the arm (the WonderPi
mobile app, given the flaky `servo_controller` path) means pairing a phone to it, which
is disproportionate effort for a proof of concept. The pitch change is deferred, not
cancelled — see §4c.

This is a legitimate call, and the pipeline is pitch-independent: nothing in §5 changes.
What *does* change is the volume the camera can observe, and that has to be stated
rather than discovered during validation.

### 4b. What 40° actually covers — measured, not assumed

Computed from the verified intrinsics (`fy = 424.83` → **V-FOV 50.4°**, `fx = 423.92` →
H-FOV 74.1°) and the live `base_footprint → depth_camera_link` transform (camera height
**0.246 m** above the floor, optical axis 39.9° below horizontal):

- Top ray: **14.7° below horizontal.** Bottom ray: 65.1°.
- **Floor is visible from 0.114 m to 0.938 m.**

| Distance ahead | Heights in frame |
|---|---|
| 0.3 m | 0 – 0.167 m |
| 0.5 m | 0 – 0.115 m |
| 0.7 m | 0 – 0.062 m |
| 0.9 m | 0 – 0.010 m |
| **≥ 1.0 m** | **entirely below floor level — nothing visible** |

Two consequences that the validation plan must respect:

1. **Maximum camera detection range is ~0.94 m.** Because the top ray descends, it
   meets the floor at 0.94 m and there is no line of sight beyond that at or above
   ground level. Any test at 1 m or 2 m measures the LiDAR only, and would have
   recorded a camera "miss" that is geometric, not a sensor failure. §8's sweep and
   §12's A/B distances are re-scoped accordingly.
2. **Nothing above camera height (0.246 m) is ever in frame, at any distance.** The
   highest point in the FOV is at the camera itself and decreases with distance.
   Floor-standing objects are still detected — a tall box at 0.5 m is seen from 0 to
   0.115 m, i.e. its base, which is enough to mark — but a genuine **overhang with no
   visible support in the near field cannot be detected at this pitch**.

**The second point costs the POC its strongest motivating case.** §12 class 3
(overhanging obstacle, the pedestal-desk form that caused the real 2026-07-22 collision)
is **not testable at 40°**. That class is the clearest demonstration of what fusion adds
over the LD19, and at this pitch the depth camera cannot see it either. The POC can
still demonstrate fusion working and can still show a benefit on low-profile obstacles
in the near field — but the headline result is deferred with the pitch change, and the
writeup must not imply otherwise.

An upside worth recording: the shallow effective range makes the pose far less sensitive
to arm sag than a level pose would be. A 2° sag lifts the apparent floor by only **3.3 cm**
at maximum range (1° → 1.6 cm), i.e. under one 5 cm costmap cell. Floor-filtering at 40°
is a comparatively forgiving problem.

### 4c. Deferred: camera pose at 20° downward

The camera's pitch is set by the arm and is the single biggest lever on this POC's
false-positive rate.

Deferred for this POC (§4a), and the reason to revisit it is now quantified rather than
aesthetic: at 20° the top ray points 5.2° *above* horizontal, so the FOV no longer
terminates on the floor and both the ~0.94 m range ceiling and the "nothing above
0.246 m" limit disappear. That is what unlocks §12's overhang class.

- **Current pose: ~40° below horizontal** (optical forward axis in `base_link` =
  `(0.766, -0.005, -0.642)`; downward pitch = `asin(0.642)` = 39.9°) — the stock
  fall-prevention pose.
- **Future target: 20° below horizontal.** A deliberate compromise:
  - Level (0°) maximizes range and minimizes floor returns, but the camera then sees
    roughly what the LiDAR already sees, which undercuts the whole point of §12's A/B
    measurement.
  - Steeply down (40°, current) guarantees the ground plane dominates every frame,
    making §8's zero-floor-false-positive criterion very hard to hit, and gives almost
    no forward look-ahead.
  - 20° down keeps the low-profile and overhanging obstacle classes — the ones the
    LD19 provably misses — inside the frame while leaving the upper half of the FOV
    on useful forward volume.
- **Acceptance criterion (checkable, no reliance on `/joint_states`):** the optical
  forward axis's z-component in `base_link`, read from
  `ros2 run tf2_ros tf2_echo base_link depth_camera_link`, is
  `-sin(20°) = -0.342 ± 0.03` (i.e. 18.3°–21.7° down).
- **Expected to change.** This value is a starting point tied to the obstacle classes
  in §12. Re-deriving it is a pose change plus a re-measured arm mask (§7), not a code
  change.

### 4d. Overhang-test pose (Option C) — test-only, not a replacement for §4a

**Decision (2026-08-07): add a second, test-only arm pose used exclusively to run the
§12 class 3 overhang test.** This does not replace the 40°-down operating pose (§4a) —
every other trial in §8 and §12 still runs at 40°. The arm returns to the 40° pose
before and after this one test.

**Why not just re-pitch to 20° (§4c) instead.** §4c was deferred because re-posing via
the WonderPi app requires phone pairing over what's currently an SSH-only session — too
much friction for a POC. This pose is instead commanded through the same ROS
`FollowJointTrajectory` path already used for normal operation (§4, known-flaky
blocker), which sidesteps the phone-pairing problem entirely. Once a ROS-commanded
reposing is accepted as in scope, there's no reason to settle for 20°'s marginal gain
(§4c) when a fuller reconfiguration gets meaningfully more height and a genuinely
positive top-ray angle — see the comparison below.

**Target pose**, from a forward-kinematics model built against the URDF and validated
by reproducing the live-measured current-pose transform exactly (computed pitch 39.94°
vs. measured 39.9°, forward axis (0.766, −0.005, −0.642) matched):

| Config | Joint values | Camera height | Pitch | Top ray | Bottom ray |
|---|---|---|---|---|---|
| Current (§4a) | j2=0.9634, j3=−1.5499, j4=−1.6755 rad | 0.246m | 40° down | −14.7° | 65.1° down |
| Wrist-only 20° (§4c) | j2, j3 unchanged, j4 only | 0.273m | 20° down | +5.6° | 44.8° down |
| **This pose (C)** | j2≈0.3°, j3≈0.3°, j4≈−84.7° | **0.349m** | **5.4° up** | **+30.6°** | 19.8° down |

(j2/j3/j4 target found by a 5° grid search over each joint's ±2.09 rad range, filtered
to forward-pointing configurations; refine to ~0.5° resolution before generating the
actual trajectory goal, which changes nothing about this design.)

**What this buys.** With a +30.6° top ray, the frame's height ceiling grows with
distance instead of shrinking: 0.65m at 0.5m range, 0.94m at 1m, 1.53m at 2m — enough
to plausibly see a desk-height (~0.75m) test object, which none of the other poses can.

**Cost: a near-field blind zone.** Bottom ray is 19.8° *below* horizontal, so the floor
doesn't enter frame until ~0.97m out — everything closer than that is invisible to the
camera at this pose, a regression against §4a's near-field coverage. This is why the
pose is test-scoped rather than adopted generally: no near-field or low-profile-obstacle
trial (§8 step 5, §12 classes 1/2/4) is ever run at this pose.

**Visible window for the class-3 target.** For a ~0.75m test object at this camera
height, the object is only in frame for range ≥ **0.68m** (below that, the required
look-up angle exceeds the +30.6° top ray and the object scrolls out of the top of frame
— this is an expected consequence of the fixed FOV, not a failure to diagnose). Test
protocol: place the object at ~1.0–1.5m (inside the window with margin), confirm
detection, then approach and confirm it exits frame near the ~0.68m boundary as
predicted.

**Motion and verification — same discipline as §4.** Commanded via
`FollowJointTrajectory`; the known controller flakiness (§4) applies here too, with the
same manual-relaunch contingency, not `.stop_ros.sh`. Verify the resulting pose against
the camera image itself and `/joint_states`/TF together, not TF alone (§4's correction
on why TF is not independent of `/joint_states`).

## 5. Architecture

Four new components, additive only — no edits to `proximity_alert`, trial launch files,
or logging used for the dataset:

### 5.1 Depth preprocessing (OpenCV) — new

A small node, `depth_preprocess_node.py`, sits between the raw Aurora depth topic and
point cloud generation:

- Uses `cv_bridge` to pull the raw depth image into a numpy/OpenCV array.
- Masks out invalid (0/NaN) depth pixels.
- Masks out an image region corresponding to the robot's own arm/gripper in frame,
  defined as a **configurable ROI in YAML, not hard-coded**, so re-deriving it for a
  different arm pose is a parameter change.
- Applies a lightweight spatial denoising filter (median initially; filter type and
  kernel are parameters, decided empirically).
- Publishes a debug image with the masked self-region drawn on it.

**Scope discipline:** this node exists because self-arm masking is specific to this
platform and has no stock ROS equivalent. Keep it to that plus debug visualization.

### 5.2 Point cloud generation

`depth_image_proc`'s stock `point_cloud_xyz` composable node, consuming the *cleaned*
depth image from §5.1 plus `camera_info`, publishing `/poc_fusion/points`.

**Why regenerate a cloud the driver already publishes.** The Aurora driver already
publishes `/ascamera/camera_publisher/depth0/points` at ~15Hz. Re-deriving a cloud
looks redundant and a reviewer will ask about it. The reason is ordering:

> **The self-arm mask and invalid-pixel rejection are image-space operations. They must
> run before projection, because once the arm's pixels are projected into 3D they are
> indistinguishable from a real obstacle at the same position.**

Masking the driver's existing cloud instead would mean expressing "the robot's own
gripper" as a 3D volume in the camera frame and culling points inside it — which is
both harder to author and harder to re-derive after a pose change than reading a
rectangle off a debug image. The image-space mask is the cheaper, more maintainable
formulation, and it is what forces §5.2 to exist.

The CPU cost of this choice is real and is measured explicitly in §8, so the tradeoff
is documented rather than assumed.

**Camera intrinsics.** The driver publishes valid `camera_info` (§0), so the
hand-authored approximate-intrinsics fallback discussed in earlier drafts is **dead
code and is removed from the plan**. For the record, the datasheet-derived estimate
(fx 449, fy 471, cy 200) was 6–11% off the measured values — which is exactly why it
was never allowed to be a silent default.

**Rectification.** `point_cloud_xyz` consumes a topic named `image_rect` and assumes a
*rectified* image. The Aurora driver publishes an unrectified depth image with non-zero
`plumb_bob` coefficients (`d = [0.0572, -0.0914, 0.0006, -0.0002, 0.0321]`).

An earlier draft accepted this as an approximation while also planning to test detection
at the horizontal FOV edges (§8 step 5) — where radial distortion is worst. That is a
confounded methodology: an edge miss could not be attributed between structured-light
falloff and distortion pushing the projection out of the detection window. The objection
is correct. It is resolved two ways.

**First, the magnitude is bounded — it was assumed, now it is computed.** Applying the
published coefficients against the verified intrinsics:

| Image position | Pixel error | Lateral error @1m | @2m |
|---|---|---|---|
| Centre | 0.00 px | 0.00 cm | 0.00 cm |
| Horizontal edge, mid-height | 2.71 px | **0.64 cm** | 1.28 cm |
| Corner | 1.50 px | 0.35 cm | 0.71 cm |

The costmap resolution is **5 cm**. Worst-case distortion displacement at 1m is
**0.64 cm — roughly one eighth of a cell**, an order of magnitude below grid
quantization, against a detection window 60 cm wide. Distortion cannot move an obstacle
out of the window at POC ranges. So the confound is real in principle but too small to
explain any edge failure, and edge results do **not** need to be treated as
inconclusive.

**Second, rectify anyway, because it is nearly free** — an `image_proc` rectify node in
the existing pipeline. This removes the variable rather than arguing about it.

> **Required if rectifying: nearest-neighbour interpolation.** `image_proc`'s default is
> linear, which is correct for colour images and wrong for depth — linear interpolation
> across a depth discontinuity synthesizes pixels at intermediate depths that correspond
> to no real surface ("flying pixels"), which project into the cloud as free-floating
> obstacle marks. Those artifacts are considerably worse for a stop trigger than the
> 0.64 cm they would be correcting.

The dominant edge effect is expected to be structured-light illumination falloff, which
rectification does not address; §8 step 5 is measuring that, and now measures it
cleanly.

### 5.3 Local costmap

A standalone `nav2_costmap_2d` local costmap instance. No map server, no global costmap,
no AMCL. Obstacle layer configured with two observation sources: `scan` (LiDAR, topic
**`/scan_raw`**) and `pointcloud` (from §5.2), the latter height-filtered to exclude
ground-plane returns.

**Coordinate frames — `global_frame: odom`, and why.** This matches Nav2's own default
for local costmaps (verified in the installed
`/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml`: `global_frame: odom`,
`robot_base_frame: base_link`, `rolling_window: true`).

An earlier review recommended switching `global_frame` to `base_link` to simplify the
detection-window indexing. **That recommendation was wrong and is rejected.** Nav2's
`ObservationBuffer` transforms every incoming observation *into `global_frame` at
observation time* (`observation_buffer.hpp`: "The frame to transform PointClouds into").
With `global_frame: base_link`, an observation buffered at time *t* is stored in the
body frame as it was at *t*; once the robot rotates, those persisted points are at the
wrong bearing, and `nav2_costmap_2d`'s rolling-window update only translates the grid —
it never rotates it. The camera source's 0.5s persistence (~7 frames) would smear under
any turn. `odom` is both the Nav2 convention and the semantically correct choice.

The indexing problem the earlier recommendation was trying to solve is real, and is
solved properly in §5.4 instead.

**Dimensions:** a 3m × 3m rolling window at 0.05m resolution. Sized by the LD19's
effective indoor range and the robot's estimated stopping distance (~0.4m) plus margin.
Note the camera contributes to only the near portion of it: the sensor's datasheet range
is 30cm–3m, but at the 40° pose its *geometric* coverage is **0.114–0.938 m** (§4b), so
beyond ~0.94 m the grid is LiDAR-only. That is expected, not a defect — but it means the
fused and LiDAR-only costmaps are identical in the outer two thirds of the window, and
any comparison drawn there is vacuous. 0.05m resolves small obstacles
(chair/table legs) while keeping the grid at 60×60 cells.

**Observation sources and rates:** LiDAR at 9.87Hz measured, camera-derived cloud at
14.7Hz measured. **The LiDAR, not the camera, is the rate-limiting sensor** — this
reverses an assumption in the previous draft. Costmap `update_frequency` **10Hz**;
`publish_frequency` **10Hz** (raised from 5Hz).

**Why `publish_frequency` is 10Hz, not 5Hz.** The stop monitor is driven by costmap
publications, so the publish interval is a hard latency floor: at 5Hz an obstacle
arriving just after a publish waits up to **200ms** before the monitor can see it at
all — before any preprocessing, projection, or evaluation. That alone consumed the
entire latency target (§8a), making it unreachable by construction. Matching
`publish_frequency` to `update_frequency` removes the self-inflicted floor and drops
the worst-case queueing contribution to 100ms. The extra publish cost is measured in
§8's compute check rather than assumed negligible.

**Observation persistence:** `0.0` for LiDAR (each scan replaces the last).
**`0.3s` for the camera** (≈4 frames at the measured 14.7Hz), reduced from 0.5s
(≈7 frames — itself a correction of an earlier draft's "2–3 frames" claim).

**Why persistence was reduced.** Persisted marks live in `odom`, so their positions
are only as good as odometry. This platform is mecanum-driven, and **wheel slip is
literally this project's dependent variable** — the surface trials exist to measure it
on granite, concrete, wood, and metal. On the high-slip surfaces, odometry drift during
a sharp holonomic rotation will smear persisted camera marks into free space, and the
stop monitor's cluster-area trigger cannot distinguish a ghost from an obstacle. Longer
persistence buys robustness to dropped frames and pays for it in false positives on
exactly the surfaces this robot is studied on.

0.3s is a starting point, not the answer. **The right value is derivable from data this
project already has:** the trial CSVs record odometry-vs-ground-truth slippage per
surface, so persistence can be set such that worst-case measured drift over the
persistence window stays below one costmap cell (5cm). §8 does that calculation and
tests ghosting directly on the high-slip surfaces rather than accepting a guess.

**Synchronization / TF tolerance:** both sources carry their own timestamps and the
costmap looks up each observation's transform at that timestamp. There is no
cross-sensor message synchronization and none is needed — the costmap fuses by
accumulating marks into a shared grid, not by pairing frames. `transform_tolerance` is
0.3s, roughly 3× the update period. If lookups still time out at that margin, that
indicates the TF chain isn't publishing fast enough and needs direct investigation
rather than a larger tolerance.

### 5.4 Stop monitor node

`costmap_stop_monitor_node.py` reads cost values within a forward-facing detection
window.

**Window definition — metric, TF-based, rotation-correct.** The window is a rectangle
defined in **`base_link`**:

- `x ∈ [0, window_forward_m]` — forward, per REP-103
- `y ∈ [−window_half_width_m, +window_half_width_m]`

Because the costmap grid is `odom`-aligned (§5.3) while the window is body-aligned, the
two differ by the robot's yaw. The monitor therefore does **not** slice the grid by row
and column ranges. It looks up `odom → base_link` at the costmap's stamp and builds a
boolean cell mask:

```
cell centres in odom:   X = origin.x + (col + 0.5)·res
                        Y = origin.y + (row + 0.5)·res
into base_link:         bx =  cos(yaw)·(X−rx) + sin(yaw)·(Y−ry)
                        by = −sin(yaw)·(X−rx) + cos(yaw)·(Y−ry)
mask = (bx ≥ 0) & (bx ≤ forward_m) & (|by| ≤ half_width_m)
```

This matters because the previous design sliced `grid[row_start:row_end,
col_start:col_end]` and introduced a `forward_sign` parameter to be determined
empirically. On an `odom`-aligned grid that is only correct at the single yaw where it
was measured. **As soon as the robot turned — and this robot strafes and turns
constantly — the detection window would sweep around to the robot's side and then its
back, silently, while still filling with plausible-looking data.** The mask formulation
removes the failure mode rather than testing for it:

- `forward_sign` is **deleted**. Forward is +x by convention.
- The "robot sits at grid centre" assumption is **deleted**. `info.origin` is used
  directly, so a non-centred grid is handled for free.
- Changing costmap resolution or size changes nothing about what the window means.

The one assumption that remains is that `info.origin.orientation` is identity (the grid
axes are `odom` axes). `nav2_costmap_2d` never rotates its grid, so this holds; the node
asserts it and warns if violated rather than silently producing a wrong mask.

**Trigger logic.** It deliberately does not trigger on a single lethal cell. Either
condition fires `/costmap_app/obstacle_detected = True`:

- At least a minimum **fraction** of masked cells are at/above the lethal threshold
  (starting point: 30%), **or**
- A single connected cluster of lethal cells within the mask reaches a minimum
  **physical area** — starting point ~100cm², specified in cm² and converted to a cell
  count at runtime from the live resolution (8-connectivity, so point-cloud blobs that
  touch only diagonally don't fragment below threshold).

Both thresholds are tuned during validation against real sweep data.

**On `window_forward_m`.** The starting value of 1.0 m slightly exceeds the camera's
0.94 m coverage ceiling (§4b), so the far ~6 cm of the window is LiDAR-only. This is
intentional — the window is a *stopping* window sized against stopping distance, not a
camera-coverage window, and the LiDAR legitimately fills it. It is recorded here so that
nobody later reads "camera detection at 1.0 m" into a trigger that fired from LiDAR
marks. Worth noting the margin is thin: at ~0.4 m stopping distance, a camera-only
detection at 0.6 m leaves roughly 0.2 m of slack, so demo runs should be at low speed.

The demo response on trigger is a full stop plus buzzer — no avoidance maneuver, per §1a.

**Velocity topic — resolved.** The robot has two distinct velocity paths:

- `/cmd_vel` — what this project's `path_tracker` publishes to; consumed by
  `odom_publisher`.
- `/controller/cmd_vel` — what the vendor apps publish to; **currently has 5
  publishers** (`lidar_app`, `line_following`, `object_tracking`, `hand_gesture`,
  `joystick_control`).

The POC publishes its zero-velocity stop to **`/cmd_vel`**, matching this project's own
node, and is configured with the topic as a parameter. Publishing into
`/controller/cmd_vel` would put it in a five-way race with vendor apps that ROS 2 will
not arbitrate.

**Stop latching.** Publishing a single zero `Twist` per costmap update (5Hz) does not
hold a stop — this robot has no motion watchdog by default, which is what allowed an
8-minute uncommanded spin previously. The POC therefore requires this project's
existing `motion_watchdog` node in the loop rather than reimplementing a stop-hold.

## 6. Data flow

```
Aurora depth image  (640x400 @ 14.7Hz, unrectified)
        │
        ▼
depth_preprocess_node (OpenCV: mask invalid, mask self-arm ROI [YAML], denoise, debug overlay)
        │  cleaned depth image
        ▼
image_proc rectify (NEAREST-NEIGHBOUR interpolation — linear creates flying pixels)
        │  /poc_fusion/depth_rect
        ▼
depth_image_proc (point_cloud_xyz)  ◄── camera_info (driver-provided, verified valid)
        │  PointCloud2  /poc_fusion/points
        ▼
nav2_costmap_2d local costmap  ◄── LaserScan /scan_raw (LD19 driver, 9.87Hz, UNFILTERED)
   (global_frame: odom, rolling; obstacle layer, height-filtered, persistence 0.3s)
        │  fused occupancy grid @ 10Hz
        ▼
costmap_stop_monitor_node
   (TF odom→base_link → body-aligned cell mask; occupancy-% OR connected-cluster-area)
        │  Bool: obstacle_detected
        ▼
Buzzer trigger + zero Twist on /cmd_vel  ──►  motion_watchdog  ──►  motors
```

## 7. Error handling

- If `camera_info` is missing or invalid, `point_cloud_xyz` silently produces no usable
  points and the costmap runs LiDAR-only. The launch logs a visible warning if
  `camera_info` is not received within a few seconds of startup.
- If the costmap node fails to activate, the launch fails loudly rather than letting the
  stop monitor run against a dead topic.
- **If the scan topic is wrong, the costmap activates and publishes an all-free grid.**
  This is the highest-value silent failure to guard: Nav2 does not error on a dead
  observation source. The scan topic is detected once and propagated from a single
  config key, never hard-coded per file.
- **Self-obstacle exclusion is pose-specific.** The mask must be re-verified any time
  the arm pose changes or the camera is bumped. A stale mask either blinds the camera
  (over-masking) or causes constant false triggers from the robot's own gripper
  (under-masking).
- **Camera failure degrades gracefully to LiDAR-only.** If depth frames stop for longer
  than a configured timeout (2.0s), the system continues on LiDAR alone. Camera marks
  age out within `observation_persistence` (0.5s), the costmap keeps updating from the
  LiDAR source, and a watchdog logs an explicit error stating fusion is no longer
  active, plus a recovery message if frames resume. Degrading to the proven sensor is
  safer than halting for a stop-trigger POC; failing *silently* is the unacceptable
  outcome.
- **Dynamic obstacles.** Nav2's obstacle layer clears cells no longer observed as
  occupied within current sensor FOV, so a person walking through frame clears once out
  of view. Confirm during validation rather than assuming.

## 8. Validation plan

**Step 0 — environment and TF verification.** Largely **already complete** (§0): the
TF chain resolves, `camera_info` is valid, `odom → base_link` resolves, rates are
measured. What remains is recording it into the repo and confirming `depth_image_proc`
is installed.

1. **Point cloud sanity check** — in RViz2, with the arm locked, confirm a held object
   appears in the cloud at its true position relative to the robot.
2. **Camera pose left at ~40°** (§4a). Record the live `base_footprint →
   depth_camera_link` transform at the start of each session so any drift or sag is
   caught, and confirm the pitch still reads 39.9° ± 1°. No arm motion.
3. **Self-mask verification** — confirm the mask excludes the robot's own gripper
   without excluding real obstacles near frame edges. Record measured ROI into config.
4. **Ground-plane filtering** — confirm bare floor ahead is not marked occupied.
   Expect iteration on `min_obstacle_height`.
5. **Detection sweep** — object at **0.3 m, 0.5 m, 0.7 m, 0.9 m**, plus **0.6 m** at
   the left and right edges of the horizontal FOV (where structured-light illumination
   falls off). Distances are bounded by the 0.94 m coverage ceiling derived in §4b —
   the earlier 1 m and 2 m points are outside the camera's line of sight at 40° and
   would have recorded geometric misses as sensor failures. Record false-positive rate
   on bare floor.
   Also record, at each distance, how much of the object's height is in frame (§4b
   table) — a mark generated from a 1 cm sliver at 0.9 m is a much weaker detection
   than one from 16 cm at 0.3 m, and the cluster-area threshold has to accommodate both.
6. **Rotation correctness** — with an obstacle fixed in place, rotate the robot in
   place through ±90° and confirm the detection window tracks the robot body: detection
   goes True only while the obstacle is actually ahead. This directly exercises the
   §5.4 mask under the condition that would have broken the previous row/column design.
6a. **Persistence ghosting under wheel slip** — the failure mode §5.3 reduced
   persistence for. Derive the persistence bound from the trial CSVs' measured
   per-surface slippage (drift over the window < 5cm), then test empirically: rapid
   holonomic rotation and strafe on the **high-slip surfaces (metal, granite)** with
   clear space ahead, counting false triggers. Repeat on wood/concrete as a low-slip
   control. If ghosting appears only on the slippery surfaces, that is odometry drift,
   not sensor noise, and persistence is the correct knob.
7. **Stop-monitor threshold tuning** — validate occupancy % and cluster-area thresholds
   against sweep data.
8. **Compute budget check** — record baseline (stock stack) and fused CPU/RAM
   separately so the cost attributable to this POC is visible rather than inferred.
9. **Latency measurement** — see §8a.
10. **Live demo runs** — drive toward an obstacle, confirm stop + buzzer from the fused
    signal.

### 8a. Latency budget

Responsiveness is the defining property of a stop system and the previous draft
asserted a `<200ms` target without measuring anything against it. It is measured
explicitly, split into what is and isn't observable in software:

**Measured (pipeline latency).** From the camera frame's `header.stamp` to the
wall-clock instant `obstacle_detected` transitions False→True. Both ends are on the
same machine so the clocks are comparable. A small recorder node subscribes to
`/poc_fusion/depth_cleaned` (which carries the camera stamp verbatim) and
`/costmap_app/obstacle_detected`, and on each rising edge records
`now − (stamp of newest depth frame preceding the transition)`. Report median and
95th percentile over at least 20 rising edges, not a single sample.

This covers: preprocessing, projection, costmap update, publish interval, and monitor
evaluation — i.e. every stage this POC actually adds.

**Not measured, and stated as such.** True "obstacle physically enters FOV → stop
command" additionally includes the sensor's own exposure and internal processing
latency, which is not recoverable from message timestamps and would need external
instrumentation (high-frame-rate video of the robot and a logged event) to capture.
The previous draft's success criterion was written in these terms but no method could
have produced it. The POC reports pipeline latency, states that it is a **lower bound**
on physical-entry latency, and does not claim otherwise.

**Budget context.** At the previously specified `publish_frequency: 5Hz` the costmap
publish interval alone contributed up to 200ms before any processing, so the earlier
draft's `<200ms` end-to-end target was unreachable by construction — inconsistent with
its own config. Rather than only loosening the criterion, `publish_frequency` is raised
to 10Hz (§5.3), halving the queueing floor to 100ms. Remaining budget at that rate:

| Stage | Expected |
|---|---|
| Camera frame → preprocess output | ~1 frame (68ms) + filter cost |
| Preprocess → point cloud | small, measured |
| Cloud → costmap incorporation | ≤1 update period (100ms) |
| Costmap publish queueing | ≤100ms worst case |
| Monitor evaluation | small (3600-cell numpy mask) |

**Target: p95 ≤ 300ms**, now plausible rather than arithmetically excluded. If the
measurement misses it, the next lever is the median filter kernel, not the publish rate.

**Quantitative success criteria:**

- Detection rate >95% across the step-5 sweep (0.3–0.9 m, centre and both FOV edges).
- Zero floor false positives across at least 10 stationary bare-floor samples.
- Rotation correctness (step 6) passes: no detection fires for an obstacle that is not
  ahead of the robot at any yaw tested.
- Pipeline latency (§8a) median and p95 recorded and reported. **Target p95 ≤ 300ms**
  at 10Hz publish.
- Zero ghosting false positives on metal and granite during the §8 step 6a slip test,
  with the persistence value justified against measured per-surface drift rather than
  chosen by feel.
- No missed obstacle within **0.9 m** across all sweep/demo trials (the camera's
  coverage ceiling at 40°, §4b). Beyond that the LiDAR alone is responsible and the
  fused system is expected to behave exactly as LiDAR-only.
- Sustained average CPU below 80%, with baseline and fused figures recorded separately.
- Graceful degradation verified: camera disconnected mid-run → LiDAR-only continues,
  watchdog logs it, reconnect restores fusion.
- Five consecutive successful live demo runs.

## 9. Assumptions and risks

- **Resolved (was the biggest risk):** the TF chain from `base_link` to the camera
  optical frame exists, is live, and is driven by the arm's joints. Verified 2026-08-06.
  The previous draft called this the single biggest schedule risk; it is retired.
- **Resolved:** driver `camera_info` is valid. The approximate-intrinsics fallback is
  removed.
- **Open blocker:** the arm control path is unreliable (§4). `/joint_states` echoes
  commanded state rather than servo feedback, and **TF is derived from it**, so TF is
  not an independent confirmation — a claim to the contrary in an earlier draft of
  this spec was wrong. Both commanding and verifying the 20° pose need a path that
  does not route through `/joint_states` alone.
- **Risk:** persisted camera marks are only as accurate as odometry, and this platform
  is being studied precisely because its mecanum odometry degrades on slick surfaces.
  Ghosting false positives on metal and granite are a foreseeable failure mode, tested
  directly in §8 step 6a.
- **Risk:** ground-plane false positives remain the most likely failure mode, and the
  20° pose is a deliberate compromise that accepts some floor in frame. Expect
  iteration on `min_obstacle_height`.
- **Risk:** unrectified depth (§5.2) contributes position error that grows toward frame
  edges, which is where §8 step 5 deliberately tests.
- **Risk:** `nav2_costmap_2d` running standalone outside a full Nav2 bringup is a
  supported but less-travelled configuration. Confirmed the executable exists; that it
  activates and publishes is the first thing the plan checks.
- **Timeline:** 2–3 days, most of it in ground filtering, threshold tuning, and the
  §12 A/B measurement.

## 10. Out of scope / explicitly deferred

- Feeding the fused costmap into `avoidance.py`'s state machine — see §11.
- Any integration with, or replacement of, the LiDAR-only trigger used for the surface
  trials.
- Full Nav2 navigation (global costmap, map server, planner, behavior tree).
- Formal camera intrinsic calibration (checkerboard) and depth rectification.

## 11. Future extension

This POC is scoped as a perception layer for an avoidance stack **that already exists**
(§1a). It stops at "detect reliably and halt" because consuming the signal means
touching trial code, not because the consumer needs to be built.

```
LiDAR + depth camera
  -> Fused local costmap            <- this POC ends here
       -> Safe free-space representation
            -> avoidance.py's existing STRAFE / gap-finding / return-to-path
               (already implemented, currently fed by LiDAR sectors alone)
```

**Why the costmap is the right intermediate representation**, rather than a
purpose-built stop signal: a costmap encodes not just "something is in front of me" but
*where free space is*, in metric coordinates. `avoidance.py`'s `_assess()` already
chooses a strafe direction from left/right clearance estimates derived from LiDAR
sectors; a fused costmap is a strictly richer input to exactly that decision, and one
that would have seen the pedestal desk. A boolean stop trigger throws away the
information the existing planner already knows how to use.

The follow-on is therefore narrow and well-defined: replace the sector-clearance input
to `_assess()` with a costmap query, behind a flag, on a branch, after the dataset is
complete.

## 12. Fusion benefit measurement

Tasks up to §8 prove the system works; they do not prove the camera helped. LiDAR alone
already stops for obstacles. The A/B measurement compares fused against a LiDAR-only
control config differing in exactly one line (`observation_sources`), across four
obstacle classes, 5 trials each. Classes 1, 2, and 4 run at **0.6 m** (not 1 m — outside
the camera's 0.94 m coverage ceiling at 40°, §4b, where the comparison would be
LiDAR-vs-LiDAR by construction) at the §4a operating pose. Class 3 runs at **1.0–1.5 m**
at the separate §4d overhang-test pose, per its own visible-window protocol — the
distances aren't comparable across classes because the poses and coverage geometry
differ; each class's distance is chosen against its own pose's ceiling, not a shared one.

1. **Tall box (~30cm)** — control. Both sensors should see it. At 0.6 m the camera sees
   roughly its bottom 9 cm (§4b), which is ample to mark.
2. **Low-profile object (~8–12cm)** — likely below the LD19 scan plane, and fully
   within the camera's frame at 0.6 m. **This is the class that carries the POC at 40°.**
3. **Overhanging object** — e.g. a pedestal-style form. The class that caused the real
   2026-07-22 collision and the strongest available motivation.
   **Not testable at the §4a operating pose** — nothing above the camera's 0.246m
   height ever enters frame there (§4b). **Testable at the §4d overhang-test pose**
   instead: run this one class at that pose (test object at 1.0–1.5m, per §4d's
   visible-window protocol), with the arm returned to 40° immediately after. Do not
   run classes 1/2/4 at the §4d pose — its near-field blind zone (§4d) would corrupt
   those results, not the camera.
4. **Thin vertical obstacle** — a chair leg. LiDAR's strength; included to check the
   camera path doesn't *degrade* anything.

Plus false-positive counts per condition over bare-floor runs, since fusion that catches
more obstacles but also stops for phantoms is not obviously a win.

**Report honestly even if the benefit is small or absent.** A null result on some
classes is legitimate and publishable, and far better than a reviewer finding the gap
later.

**State the pose limitation as a limitation, not a silence.** The writeup must say that
classes 1/2/4 run at the stock fall-prevention pose (§4a), which bounds their coverage
to ~0.94m and to objects below 0.246m — and that class 3 required a second, test-only
pose (§4d) to be observable at all, with its own narrower 0.68m–~2m visible window and a
near-field blind zone that rules it out for general use. The overhang result is real but
pose-specific, not evidence that the §4a operating pose sees overhangs. Claiming fusion
benefit while quietly omitting that distinction is the failure mode to avoid here.
