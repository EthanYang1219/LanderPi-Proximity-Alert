"""Derive nav2_costmap_2d parameter key paths from the parameter tree itself.

Dependency-free pure logic (no ROS, no yaml I/O): takes the already-parsed
`costmap.costmap.ros__parameters` dict and returns key strings.

Why this exists (Task 6 fix round 1, review finding I5): the launch file has
to inject the scan topic as a parameter override, and the override key is
`<obstacle layer name>.<laser scan source name>.topic`. Both of those names
are owned by costmap_params.yaml (`plugins:` and `observation_sources:`).
Restating them as a Python literal creates a silent-failure path: rename
either name in the YAML and the injected key becomes an unused parameter
while the real source falls back to nav2's default topic (which is the
source name, and does not exist) -- a valid, active, all-free costmap with
no error, exactly the failure CONSTRAINTS.md calls "the most expensive
silent failure available here".

Every ambiguous or missing case raises RuntimeError rather than guessing,
so the failure is hard (the launch description aborts) rather than soft.
"""

OBSTACLE_LAYER_PLUGIN = 'nav2_costmap_2d::ObstacleLayer'
LASER_SCAN_DATA_TYPE = 'LaserScan'


def resolve_obstacle_layer_name(costmap_ros_params):
    """Return the name of the single ObstacleLayer plugin in the params tree.

    Raises RuntimeError if `plugins` is missing, if a listed plugin has no
    parameter block, or if the number of ObstacleLayer plugins is not exactly
    one.
    """
    if 'plugins' not in costmap_ros_params:
        raise RuntimeError(
            "costmap params have no 'plugins' key; cannot determine which "
            "layer owns the observation sources")

    matches = []
    for plugin_name in costmap_ros_params['plugins']:
        if plugin_name not in costmap_ros_params:
            raise RuntimeError(
                "costmap params list plugin '{}' but contain no '{}' "
                "parameter block for it".format(plugin_name, plugin_name))
        block = costmap_ros_params[plugin_name]
        if isinstance(block, dict) and block.get('plugin') == OBSTACLE_LAYER_PLUGIN:
            matches.append(plugin_name)

    if len(matches) != 1:
        raise RuntimeError(
            "expected exactly one {} plugin in costmap params, found {}: "
            "{}".format(OBSTACLE_LAYER_PLUGIN, len(matches), matches))
    return matches[0]


def resolve_scan_source_name(costmap_ros_params, layer_name):
    """Return the name of the single LaserScan observation source on `layer_name`.

    Raises RuntimeError if `observation_sources` is missing, if a named source
    has no parameter block, or if the number of LaserScan sources is not
    exactly one.
    """
    layer = costmap_ros_params[layer_name]
    if 'observation_sources' not in layer:
        raise RuntimeError(
            "costmap layer '{}' has no 'observation_sources' key".format(
                layer_name))

    matches = []
    for source_name in layer['observation_sources'].split():
        if source_name not in layer:
            raise RuntimeError(
                "costmap layer '{}' lists observation source '{}' in "
                "observation_sources but contains no '{}' parameter block "
                "for it".format(layer_name, source_name, source_name))
        source = layer[source_name]
        if isinstance(source, dict) and source.get('data_type') == LASER_SCAN_DATA_TYPE:
            matches.append(source_name)

    if len(matches) != 1:
        raise RuntimeError(
            "expected exactly one {} observation source on costmap layer "
            "'{}', found {}: {}".format(
                LASER_SCAN_DATA_TYPE, layer_name, len(matches), matches))
    return matches[0]


def resolve_scan_topic_key(costmap_ros_params):
    """Return the parameter key for the LaserScan source's `topic` parameter.

    e.g. 'obstacle_layer.scan.topic' -- but derived from the params tree, so
    renaming the layer or the source in costmap_params.yaml moves the key
    with it instead of silently orphaning the override.
    """
    layer_name = resolve_obstacle_layer_name(costmap_ros_params)
    source_name = resolve_scan_source_name(costmap_ros_params, layer_name)
    return '{}.{}.topic'.format(layer_name, source_name)
