"""Tests for decoding nav2_msgs/Costmap's raw cost grid (Task 7).

Task 7 consumes `/costmap/costmap_raw` (`nav2_msgs/Costmap`) rather than
`/costmap/costmap` (`nav_msgs/OccupancyGrid`), because only the raw topic
carries nav2's real 0..255 cost scale that `lethal_threshold: 253` is
expressed in. That choice brings one hazard with it, and this module exists
for that hazard alone:

    On the raw scale 255 is NO_INFORMATION -- "I have never seen this cell"
    -- and it is NUMERICALLY LARGER than 254 (LETHAL_OBSTACLE).

`obstacle_detection.should_stop()` thresholds with `>= lethal_threshold`
and documents that unknown cells arrive as -1. Handing it raw bytes
unchanged would therefore classify every never-observed cell as lethal:
the monitor would report OBSTACLE from unknown space, which at startup is
most of a rolling costmap. That is a false STOP, which is the safe
direction, but it is still a wrong reading and it would make the tri-state
useless for the trial record.
"""

import numpy as np
import pytest

from poc_fusion.lib.costmap_raw import (
    NO_INFORMATION,
    UNKNOWN_SENTINEL,
    decode_raw_costmap,
)


def test_decodes_row_major_into_the_declared_shape():
    grid = decode_raw_costmap([0, 1, 2, 3, 4, 5], size_x=3, size_y=2)
    assert grid.shape == (2, 3)
    # Row-major, starting with (0,0), per the message definition.
    assert grid[0, 0] == 0
    assert grid[0, 2] == 2
    assert grid[1, 0] == 3


def test_no_information_becomes_the_unknown_sentinel_not_a_lethal_cost():
    # THE test this module exists for. 255 (NO_INFORMATION) is numerically
    # above 254 (LETHAL_OBSTACLE), so passing raw bytes straight through
    # would make every never-observed cell read as lethal. Delete the
    # remapping and this returns 255, which is >= any sane lethal_threshold.
    grid = decode_raw_costmap([NO_INFORMATION], size_x=1, size_y=1)
    assert grid[0, 0] == UNKNOWN_SENTINEL
    assert grid[0, 0] < 253


def test_lethal_and_inscribed_costs_survive_unchanged():
    # 254 == LETHAL_OBSTACLE, 253 == INSCRIBED_INFLATED_OBSTACLE. Both must
    # come through intact -- a decoder that clamped or rescaled them would
    # make lethal_threshold: 253 unreachable, the exact silent-CLEAR-forever
    # failure that consuming costmap_raw was chosen to avoid.
    grid = decode_raw_costmap([254, 253, 252, 0], size_x=4, size_y=1)
    assert list(grid[0]) == [254, 253, 252, 0]


def test_dtype_is_signed_so_the_sentinel_is_representable():
    # uint8 cannot hold -1; a decoder that kept uint8 would wrap the
    # sentinel back round to 255 and silently undo the remapping above.
    grid = decode_raw_costmap([NO_INFORMATION], size_x=1, size_y=1)
    assert np.issubdtype(grid.dtype, np.signedinteger)


def test_unknown_cells_do_not_trigger_should_stop():
    # End-to-end with the real detector: a window that is entirely unknown
    # must not read as an obstacle. This is the property the remapping
    # exists to guarantee, asserted through the consumer rather than only
    # at the decoder's own boundary.
    from poc_fusion.lib.obstacle_detection import should_stop
    grid = decode_raw_costmap([NO_INFORMATION] * 16, size_x=4, size_y=4)
    mask = np.ones((4, 4), dtype=bool)
    assert should_stop(grid, mask, resolution=0.05, lethal_threshold=253,
                       min_occupancy_fraction=0.30,
                       min_cluster_area_cm2=100.0) is False


def test_size_mismatch_raises_rather_than_reshaping_wrongly():
    # A truncated or over-long data array means the message is not what it
    # claims. Reshaping whatever fits would silently evaluate the window
    # against a misaligned grid, so refuse instead.
    with pytest.raises(ValueError):
        decode_raw_costmap([0, 1, 2], size_x=2, size_y=2)
