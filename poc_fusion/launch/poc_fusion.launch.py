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


def generate_launch_description():
    pkg_share = get_package_share_directory('poc_fusion')
    depth_preprocess_params = os.path.join(
        pkg_share, 'config', 'depth_preprocess_params.yaml')

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
            # --- Task 6 adds costmap-related composable nodes here ---
        ],
        output='screen',
    )

    # --- Task 6 adds the costmap node / lifecycle manager below ---
    # --- Task 8 finishes wiring and verifies the whole chain here ---

    return LaunchDescription([
        depth_preprocess_node,
        poc_fusion_container,
    ])
