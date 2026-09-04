"""POC fusion launch file: depth + LiDAR into one Nav2 local costmap.

Brings up, in order:

  Depth pipeline
    depth_preprocess_node          -> /poc_fusion/depth_cleaned
    image_proc::RectifyNode        -> /poc_fusion/depth_rect
      (nearest-neighbour interpolation -- see the `interpolation` override
      below; NOT the linear default, which synthesizes "flying pixel"
      artifacts across depth discontinuities)
    depth_image_proc::PointCloudXyzNode -> /poc_fusion/points

  Costmap
    nav2_costmap_2d (standalone Costmap2DROS lifecycle node) fusing
      /scan_raw (LiDAR) + /poc_fusion/points (depth) into one obstacle layer
      -> /costmap/costmap, /costmap/costmap_updates
    nav2_lifecycle_manager (autostart) bringing that node to `active`

  Detection
    costmap_stop_monitor_node, consuming /costmap/costmap_raw and publishing
      a tri-state clear/blocked/unknown signal

  Response (OPT-IN, off by default)
    stop_action_node, enabled with `stop_action:=true`. It is a SERIES GATE
    on cmd_vel -- read its REQUIRED TOPOLOGY section before enabling it.

STATUS: an in-progress proof of concept, not a finished subsystem.

End-to-end integration of this chain WAS verified live on 2026-08-10 (sensor
ingress, costmap representation, active-costmap validation, clear -> CLEAR,
lethal obstacle -> safety response, and recovery all confirmed with sample
counts; see the Task 8 section of docs/poc_fusion_verification.md). Two
things are not done: depth-only detection of an overhang was never
live-verified, and a bench test on 2026-08-11 failed its gate -- depth cannot
contribute for floor-standing obstacles at this camera's mounting geometry,
which is the main open question hanging over the approach.

Validation/tuning, compute budget, graceful degradation and the fusion
benefit A/B measurement were not carried out. docs/README.md maps the task
numbering used throughout these files to stages and their status.

The two depth composable nodes are loaded into a single `poc_fusion_container`
(rclcpp_components) rather than run as separate processes. nav2_costmap_2d
ships no composable-node plugin in Humble, so the costmap and the lifecycle
manager run as their own processes alongside the container (see the comment
inside the container's node list).

`depth_preprocess_node` is a plain (non-composable) Python node -- rclpy
composable-node support is immature relative to rclcpp's, and it already
shipped as a standalone `Node` action, so it stays that way here rather than
being force-fit into the container.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from poc_fusion.lib.costmap_param_keys import resolve_scan_topic_key

import os
import yaml


def generate_launch_description():
    pkg_share = get_package_share_directory('poc_fusion')
    depth_preprocess_params = os.path.join(
        pkg_share, 'config', 'depth_preprocess_params.yaml')
    costmap_params_path = os.path.join(
        pkg_share, 'config', 'costmap_params.yaml')

    depth_cleaned_topic = '/poc_fusion/depth_cleaned'
    depth_rect_topic = '/poc_fusion/depth_rect'
    points_topic = '/poc_fusion/points'
    camera_info_topic = '/ascamera/camera_publisher/depth0/camera_info'

    # --- Section: depth preprocessing (Task 4) -----------------------------
    depth_preprocess_node = Node(
        package='poc_fusion',
        executable='depth_preprocess_node',
        name='depth_preprocess_node',
        output='screen',
        parameters=[depth_preprocess_params],
    )

    # --- Section: point cloud generation (Task 5, this task) ---------------
    #
    # Step 2: rectify with nearest-neighbour interpolation. image_proc's
    # `interpolation` parameter takes an OpenCV interpolation-flag int;
    # 0 == cv2.INTER_NEAREST. Verified live by `ros2 param get` on the
    # running node (see docs/poc_fusion_verification.md, Task 5 section) --
    # do not trust this comment alone.
    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='depth_rectify_node',
        namespace='poc_fusion',
        parameters=[{'interpolation': 0}],
        remappings=[
            ('image', depth_cleaned_topic),
            ('camera_info', camera_info_topic),
            ('image_rect', depth_rect_topic),
        ],
    )

    # Step 1 + Step 3: point cloud projection, remapped per the brief.
    point_cloud_xyz_node = ComposableNode(
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyzNode',
        name='point_cloud_xyz_node',
        namespace='poc_fusion',
        remappings=[
            ('image_rect', depth_rect_topic),
            ('camera_info', camera_info_topic),
            ('points', points_topic),
        ],
    )

    poc_fusion_container = ComposableNodeContainer(
        name='poc_fusion_container',
        namespace='poc_fusion',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            rectify_node,
            point_cloud_xyz_node,
            # nav2_costmap_2d does not ship a composable-node plugin in
            # Humble (only the standalone `nav2_costmap_2d` executable, a
            # full Costmap2DROS lifecycle node -- see `ros2 pkg executables
            # nav2_costmap_2d`), so there is nothing to load into this
            # container for Task 6. It is launched as its own process
            # below instead.
        ],
        output='screen',
    )

    # --- Section: costmap (Task 6, this task) -------------------------------
    #
    # scan_topic is read out of costmap_params.yaml at launch-description
    # generation time and injected as a parameter override for the costmap
    # node's obstacle_layer.scan.topic parameter. This is the ONLY place the
    # scan topic literal is read from; costmap_params.yaml itself leaves that
    # key unset (see its header comment) and no file hard-codes /scan or
    # /scan_raw -- costmap_params.yaml's `scan_topic` key is the single
    # source of truth.
    #
    # costmap_params.yaml's `costmap` block is extracted here and passed to
    # the Node action as an in-memory dict rather than passing the file path
    # straight through as --params-file. This is not stylistic: standard ROS
    # 2 params-file YAML permits ONLY node-name (or wildcard) keys at the top
    # level, each holding a `ros__parameters` map. costmap_params.yaml's
    # top-level `scan_topic` key (needed so this one file stays the single
    # source of truth for the scan topic) violates that when the raw file is
    # handed to rcl's yaml parser -- verified live: passing the file directly
    # made the costmap process abort at startup with `rcl: Failed to parse
    # global arguments` / `Cannot have a value before ros__parameters`
    # (docs/poc_fusion_verification.md, Task 6). Extracting just the
    # `costmap.costmap.ros__parameters` sub-tree in Python and passing THAT
    # as a dict sidesteps the strict top-level-key rule entirely; launch_ros
    # writes it back out to a well-formed temporary params file for the node.
    with open(costmap_params_path, 'r') as f:
        costmap_params_full = yaml.safe_load(f)
    scan_topic = costmap_params_full['scan_topic']
    costmap_ros_params = costmap_params_full['costmap']['costmap']['ros__parameters']

    # The override KEY is derived from the params tree, not restated as a
    # literal. `resolve_scan_topic_key` walks `plugins` -> the single
    # nav2_costmap_2d::ObstacleLayer block -> `observation_sources` -> the
    # single source whose data_type is LaserScan, and returns
    # '<layer>.<source>.topic'. Writing 'obstacle_layer.scan.topic' here
    # instead would hard-code two names costmap_params.yaml owns: rename
    # either and the override silently becomes an unused parameter while the
    # real source falls back to nav2's default topic (the source name, which
    # does not exist) -- an all-free costmap with no error. Every ambiguous
    # or missing case raises RuntimeError, so generate_launch_description()
    # aborts the launch instead of coming up wrong (review finding I5;
    # unit-tested in test/test_costmap_param_keys.py, mutation-proved live
    # in docs/poc_fusion_verification.md, "Task 6 fix round 1").
    scan_topic_key = resolve_scan_topic_key(costmap_ros_params)

    # Node name/namespace deliberately NOT overridden here, and the actual
    # coupling that creates is NOT the YAML key path. Verified against the
    # installed launch_ros source
    # (/opt/ros/humble/lib/python3.10/site-packages/launch_ros/actions/node.py:369-377,
    # pasted in docs/poc_fusion_verification.md, "Task 6 fix round 1"):
    # `_create_params_file_from_dict` writes the dict under
    # `self.node_name if self.is_node_name_fully_specified() else '/**'`.
    # This action passes no `name=`/`namespace=`, so the temp params file is
    # written under the `/**` WILDCARD and the parameters load regardless of
    # what the node ends up calling itself. costmap_params.yaml's
    # `costmap: costmap: ros__parameters:` nesting is stripped by the Python
    # extraction above and never reaches rcl, so that nesting is
    # DOCUMENTATION of the observed default node name, not a binding -- an
    # earlier version of this comment claimed a silent fallback to defaults
    # here, which is wrong.
    # The real name dependency is `node_names: ['costmap/costmap']` on the
    # lifecycle manager below, and its failure mode is a HANG, not an error:
    # the manager waits forever on a change_state service that never appears.
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        output='screen',
        parameters=[
            costmap_ros_params,
            {scan_topic_key: scan_topic},
        ],
        # See the "fail loudly" block on the lifecycle manager below.
        on_exit=Shutdown(reason='costmap node exited; tearing down the launch '
                                'so nothing subscribes to a dead costmap'),
    )

    # nav2_lifecycle_manager brings the costmap lifecycle node from
    # unconfigured -> inactive -> active. autostart: true means it does
    # this automatically at launch rather than waiting for an external
    # trigger (there is no BT navigator or other orchestrator in this POC
    # to send one). node_names must be the costmap node's fully-qualified
    # name AS SEEN FROM THIS MANAGER -- verified live to be `costmap/costmap`
    # (docs/poc_fusion_verification.md, Task 6: lifecycle_manager log shows
    # it successfully bringing that exact name up to `active`, and a
    # `ros2 lifecycle get /costmap/costmap` readback confirms the state).
    #
    # bond_timeout: 0.0 (disables the post-activation bond liveliness
    # check; this is a standard, documented nav2_lifecycle_manager
    # parameter, not custom code). Verified live and necessary: with a
    # nonzero bond_timeout (tried 4.0s, then 10.0s), the costmap node
    # still transitioned all the way to `active` (confirmed both by its
    # own log lines and a direct `ros2 lifecycle get /costmap/costmap` ->
    # `active [3]`), but nav2_lifecycle_manager separately logged
    # `Server costmap/costmap was unable to be reached after N.00s by
    # bond. This server may be misconfigured.` every time, because
    # `ros2 topic info /bond -v` showed the ONLY publisher and subscriber
    # on /bond were both `lifecycle_manager_costmap` itself -- the
    # standalone `nav2_costmap_2d` executable's Costmap2DROS never calls
    # `createBond()` the way the full servers embedded in a real Nav2
    # bringup (controller_server, planner_server) do, so there is no peer
    # for the manager to bond with, ever, at any timeout value. At
    # bond_timeout: 4.0 this even crashed the lifecycle_manager process
    # outright (`terminate called ... statemap::TransitionUndefinedException`)
    # rather than merely logging. Disabling the check is therefore not a
    # tolerance workaround for a slow node -- it removes a check for a
    # signal this particular node structurally never sends.
    #
    # --- "Fail loudly" (brief Step 4): what IS and IS NOT covered ----------
    #
    # COVERED, by the `on_exit=Shutdown(...)` on both this node and the
    # costmap node above: if EITHER process dies, the whole launch is torn
    # down. An earlier version of this file relied on `autostart: true`
    # plus an ERROR log line and asserted that satisfied the requirement.
    # It did not -- that is "fail in the scrollback", not "fail loudly", and
    # it left two real observed failures with the launch still green:
    #   * lifecycle_manager CRASHING outright (observed at bond_timeout 4.0:
    #     `statemap::TransitionUndefinedException`, exit -6) while the rest
    #     of the launch kept running;
    #   * `Failed to bring up all requested nodes. Aborting bringup.` with
    #     the manager process still alive.
    # In both cases `ros2 launch` stayed up and Task 8's stop monitor would
    # have subscribed to a costmap topic that never publishes -- verbatim
    # the outcome brief Step 4 exists to prevent. The first of those two is
    # now covered; the second is not (see below).
    #
    # NOT COVERED: the manager-alive-but-costmap-never-active case
    # ("Aborting bringup" without a process exit). No launch action here
    # gates on the costmap actually reaching `active`, so this launch CAN
    # still come up green with an inactive costmap that publishes nothing.
    # This is stated explicitly rather than papered over: Task 8's stop
    # monitor MUST refuse to start (or must fail loudly) when
    # `/costmap/costmap` is not in state `active`, and must not treat
    # "subscribed, no messages" as "no obstacles". This gap is recorded in
    # the Task 6 report as well.
    #
    # Note the bond check is NOT what would have caught either case: see the
    # bond_timeout block above -- the standalone nav2_costmap_2d node never
    # creates a bond at all, so the bond check only ever produced a false
    # negative here.
    lifecycle_manager_costmap = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['costmap/costmap'],
            'bond_timeout': 0.0,
        }],
        on_exit=Shutdown(reason='lifecycle_manager_costmap exited; tearing '
                                'down the launch so nothing subscribes to a '
                                'costmap that may never activate'),
    )

    # --- Section: costmap stop monitor (Task 7) -----------------------------
    #
    # Reads config/stop_monitor_params.yaml directly as a --params-file: that
    # file's only top-level key is the node name, so it is already valid ROS
    # 2 params-file YAML and needs none of the sub-tree extraction the
    # costmap params file above requires.
    #
    # The monitor consumes /costmap/costmap_raw (nav2_msgs/Costmap) rather
    # than /costmap/costmap (nav_msgs/OccupancyGrid) -- a correction to the
    # Task 7 brief's Step 1, forced by two live measurements pasted in
    # docs/poc_fusion_verification.md under "Task 7 -- Step 2": over one
    # 90.15 s window a single subscriber saw max cost 100 on the
    # OccupancyGrid versus 254 on costmap_raw (so lethal_threshold 253 could
    # never be reached there, and the monitor would report CLEAR forever with
    # no error), and received 1 OccupancyGrid message against 562 on
    # costmap_raw. The topic name is a PARAMETER in that YAML, not a literal
    # here.
    #
    # It publishes no velocity and commands no motion -- acting on the signal
    # is Task 9's scope. Ordering within a LaunchDescription does not gate
    # startup; the monitor's own tri-state is what refuses to report CLEAR
    # before the costmap is confirmed ACTIVE, which is Task 6 Step 4's
    # carried-forward gap closed in the consumer rather than in the launch.
    #
    # Task 8 owns integration verification of the whole chain; this action is
    # wired here only so Task 8 has something to verify.
    stop_monitor_params = os.path.join(
        pkg_share, 'config', 'stop_monitor_params.yaml')
    costmap_stop_monitor_node = Node(
        package='poc_fusion',
        executable='costmap_stop_monitor_node',
        name='costmap_stop_monitor_node',
        output='screen',
        parameters=[stop_monitor_params],
    )

    # --- Section: stop action (Task 9) --------------------------------------
    #
    # OFF BY DEFAULT, and that default is a safety decision, not laziness.
    # This is the first and only node in this launch that can command a
    # velocity. Task 8 verified the whole sensing chain with `ros2 launch
    # poc_fusion poc_fusion.launch.py` and nothing in that launch could move
    # a wheel; keeping the default at false preserves exactly that property,
    # so the verified command stays safe to run at any time.
    #
    # Enable explicitly, and read stop_action_node.py's REQUIRED TOPOLOGY
    # section first. stop_action_node is a SERIES GATE: the motion source
    # flows through it, so it must sit between that source and
    # `proximity_alert`'s motion_watchdog. The watchdog is required because a
    # zero Twist at costmap rate does NOT hold a stop on this platform.
    #
    #   ros2 run proximity_alert motion_watchdog
    #   ros2 run proximity_alert path_tracker --ros-args -r /cmd_vel:=/cmd_vel_raw
    #   ros2 launch poc_fusion poc_fusion.launch.py stop_action:=true
    #
    # The defaults already wire /cmd_vel_raw -> gate -> /cmd_vel_unsafe, which
    # is the watchdog's input. The parallel-publisher design these replaced was
    # measured failing in Task 9 Step 3 (51.6% forward duty cycle after a stop
    # engaged); see the config file for the full account.
    #
    # `publish_cmd_vel:=false` runs every decision, log line and alert but
    # creates no Twist publisher at all -- the observe-only mode used to
    # verify the chain on a stationary robot before any motion test. In that
    # mode the gate also passes no motion through.
    stop_action_arg = DeclareLaunchArgument(
        'stop_action', default_value='false',
        description='Bring up stop_action_node (Task 9). OFF by default: it '
                    'is the only node here that can command a velocity.')
    cmd_vel_in_topic_arg = DeclareLaunchArgument(
        'cmd_vel_in_topic', default_value='/cmd_vel_raw',
        description='Motion source feeding the gate. Remap path_tracker onto '
                    'this topic. Never /controller/cmd_vel.')
    cmd_vel_out_topic_arg = DeclareLaunchArgument(
        'cmd_vel_out_topic', default_value='/cmd_vel_unsafe',
        description="Gate output; must be motion_watchdog's input topic. "
                    'Never /controller/cmd_vel, and never equal to '
                    'cmd_vel_in_topic.')
    publish_cmd_vel_arg = DeclareLaunchArgument(
        'publish_cmd_vel', default_value='true',
        description='false = observe-only: decisions and alerts run, but no '
                    'Twist publisher is created and no wheel can be commanded.')

    stop_action_params = os.path.join(
        pkg_share, 'config', 'stop_action_params.yaml')
    stop_action_node = Node(
        package='poc_fusion',
        executable='stop_action_node',
        name='stop_action_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('stop_action')),
        parameters=[
            stop_action_params,
            # Launch arguments win over the YAML so the two safety-relevant
            # keys can be set at the command line without editing config.
            {'cmd_vel_in_topic': LaunchConfiguration('cmd_vel_in_topic')},
            {'cmd_vel_out_topic': LaunchConfiguration('cmd_vel_out_topic')},
            {'publish_cmd_vel': LaunchConfiguration('publish_cmd_vel')},
        ],
    )

    return LaunchDescription([
        stop_action_arg,
        cmd_vel_in_topic_arg,
        cmd_vel_out_topic_arg,
        publish_cmd_vel_arg,
        depth_preprocess_node,
        poc_fusion_container,
        costmap_node,
        lifecycle_manager_costmap,
        costmap_stop_monitor_node,
        stop_action_node,
    ])
