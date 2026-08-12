"""Tests for deriving costmap parameter key paths from the params tree itself.

Task 6 fix round 1 (review finding I5): poc_fusion.launch.py used to hard-code
the string 'obstacle_layer.scan.topic' when injecting the scan topic override,
while costmap_params.yaml owns both of those names (`plugins:` and
`observation_sources:`). Renaming either in the YAML made the injected key
become an unused parameter while the real source silently fell back to
nav2's default topic -- an all-free costmap with no error. These tests pin
the derivation, and pin that every ambiguous/missing case raises loudly
instead of returning something plausible.
"""
import pytest

from poc_fusion.lib.costmap_param_keys import resolve_scan_topic_key


def _params(layer_name='obstacle_layer',
            sources='scan pointcloud',
            scan_name='scan',
            scan_data_type='LaserScan',
            plugins=None,
            include_scan_block=True):
    layer = {
        'plugin': 'nav2_costmap_2d::ObstacleLayer',
        'enabled': True,
        'observation_sources': sources,
        'pointcloud': {'topic': '/poc_fusion/points',
                       'data_type': 'PointCloud2'},
    }
    if include_scan_block:
        layer[scan_name] = {'data_type': scan_data_type}
    return {
        'global_frame': 'odom',
        'plugins': ['obstacle_layer'] if plugins is None else plugins,
        layer_name: layer,
    }


def test_happy_path_returns_layer_and_source_derived_key():
    assert resolve_scan_topic_key(_params()) == 'obstacle_layer.scan.topic'


def test_renamed_layer_is_followed():
    params = _params(layer_name='fusion_obstacle_layer',
                     plugins=['fusion_obstacle_layer'])
    assert resolve_scan_topic_key(params) == 'fusion_obstacle_layer.scan.topic'


def test_renamed_scan_source_is_followed():
    params = _params(sources='lidar pointcloud', scan_name='lidar')
    assert resolve_scan_topic_key(params) == 'obstacle_layer.lidar.topic'


def test_source_order_does_not_matter():
    params = _params(sources='pointcloud scan')
    assert resolve_scan_topic_key(params) == 'obstacle_layer.scan.topic'


def test_missing_plugins_key_raises():
    params = _params()
    del params['plugins']
    with pytest.raises(RuntimeError, match='plugins'):
        resolve_scan_topic_key(params)


def test_no_obstacle_layer_plugin_raises():
    params = _params()
    params['obstacle_layer']['plugin'] = 'nav2_costmap_2d::VoxelLayer'
    with pytest.raises(RuntimeError, match='ObstacleLayer'):
        resolve_scan_topic_key(params)


def test_plugin_listed_but_block_missing_raises():
    params = _params()
    del params['obstacle_layer']
    with pytest.raises(RuntimeError, match='obstacle_layer'):
        resolve_scan_topic_key(params)


def test_missing_observation_sources_raises():
    params = _params()
    del params['obstacle_layer']['observation_sources']
    with pytest.raises(RuntimeError, match='observation_sources'):
        resolve_scan_topic_key(params)


def test_source_named_but_block_missing_raises():
    params = _params(include_scan_block=False)
    with pytest.raises(RuntimeError, match='scan'):
        resolve_scan_topic_key(params)


def test_no_laserscan_source_raises():
    params = _params(scan_data_type='PointCloud2')
    with pytest.raises(RuntimeError, match='LaserScan'):
        resolve_scan_topic_key(params)


def test_two_laserscan_sources_is_ambiguous_and_raises():
    params = _params(sources='scan scan2')
    params['obstacle_layer']['scan2'] = {'data_type': 'LaserScan'}
    with pytest.raises(RuntimeError, match='exactly one'):
        resolve_scan_topic_key(params)


def test_two_obstacle_layer_plugins_is_ambiguous_and_raises():
    params = _params(plugins=['obstacle_layer', 'obstacle_layer_b'])
    params['obstacle_layer_b'] = dict(params['obstacle_layer'])
    with pytest.raises(RuntimeError, match='exactly one'):
        resolve_scan_topic_key(params)
