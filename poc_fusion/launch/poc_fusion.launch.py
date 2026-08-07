"""POC fusion launch file.

Task 5 (this task) builds the depth pipeline only:
  depth_preprocess_node (Task 4)
    -> /poc_fusion/depth_cleaned
  image_proc::RectifyNode (nearest-neighbour interpolation -- see the
    `interpolation` parameter override below; NOT the linear default, which
    synthesizes "flying pixel" artifacts across depth discontinuities)
    -> /poc_fusion/depth_rect
  depth_image_proc::PointCloudXyzNode
    -> /poc_fusion/points

Both composable nodes are loaded into a single `poc_fusion_container`
(rclcpp_components) rather than run as separate processes, matching the
brief's intent that this container is a natural place for Task 6 to also
load the costmap-related composable nodes and Task 8 to wire in the
lifecycle manager. Do NOT add Task 6's or Task 8's nodes here yet --
sections below are pre-labelled for where those additions belong.

`depth_preprocess_node` is a plain (non-composable) Python node -- rclpy
composable-node support is immature relative to rclcpp's, and Task 4 already
shipped it as a standalone `Node` action, so it stays that way here rather
than being force-fit into the container.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

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

    # Node name/namespace deliberately NOT overridden here. Verified live
    # (docs/poc_fusion_verification.md, Task 6) that `nav2_costmap_2d` run
    # with no name/namespace override comes up as /costmap/costmap --
    # matching the costmap_params.yaml key path `costmap: costmap:
    # ros__parameters:`. Overriding either without also changing the YAML
    # key path is exactly the silent-mismatch failure mode called out in
    # the Task 6 brief (costmap silently falls back to defaults, or the
    # lifecycle manager waits forever on a node name that does not exist).
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        output='screen',
        parameters=[
            costmap_ros_params,
            {'obstacle_layer.scan.topic': scan_topic},
        ],
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
    # "Fail loudly" (brief Step 4) is still satisfied without the bond
    # check: `autostart: true` means the manager attempts the transitions
    # immediately and unconditionally, a genuine configuration/transition
    # error still surfaces as an ERROR-level log line (as seen above), and
    # the verification doc's `ros2 lifecycle get` / `ros2 topic hz` /
    # `ros2 topic echo --once` readbacks are the actual proof the costmap
    # reached `active` and is producing data -- not the bond mechanism.
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
    )

    # --- Task 8 finishes wiring and verifies the whole chain here ---

    return LaunchDescription([
        depth_preprocess_node,
        poc_fusion_container,
        costmap_node,
        lifecycle_manager_costmap,
    ])
