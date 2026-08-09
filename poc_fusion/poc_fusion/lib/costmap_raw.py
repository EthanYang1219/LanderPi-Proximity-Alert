"""Decoding for nav2_msgs/Costmap's raw cost grid (Task 7).

No ROS: takes the message's `data` sequence and declared dimensions as
plain numbers, returns a numpy array, so it is testable off-robot.

WHY A DECODER IS NEEDED AT ALL
------------------------------
The monitor consumes `/costmap/costmap_raw` (`nav2_msgs/Costmap`), whose
`data` is `uint8[]` on nav2's real cost scale:

    0   FREE_SPACE
    252 and below   ordinary/inflated costs
    253 INSCRIBED_INFLATED_OBSTACLE
    254 LETHAL_OBSTACLE
    255 NO_INFORMATION

255 means "never observed", yet it is numerically ABOVE 254. Every
threshold in `obstacle_detection` is a `>= lethal_threshold` comparison,
and that module documents unknown cells as arriving as -1 (the
`nav_msgs/OccupancyGrid` convention it was written against in Task 3).
Passing raw bytes straight through would therefore classify unknown space
as lethal. Remapping 255 to -1 here keeps `obstacle_detection`'s existing,
already-reviewed contract intact and confines the raw-scale knowledge to
this one module.

The array is returned signed for the same reason: -1 is not representable
in uint8, so a uint8 result would wrap the sentinel straight back to 255
and silently undo the remapping.
"""

import numpy as np

#: nav2's costmap_2d::NO_INFORMATION.
NO_INFORMATION = 255

#: The value NO_INFORMATION is remapped to -- `obstacle_detection`'s
#: documented "unknown" input, below every meaningful lethal threshold.
UNKNOWN_SENTINEL = -1


def decode_raw_costmap(data, size_x, size_y):
    """Raw `nav2_msgs/Costmap.data` -> a signed (size_y, size_x) grid.

    `size_x` is the horizontal cell count (columns) and `size_y` the
    vertical one (rows); the message stores data in row-major order
    starting at cell (0, 0).

    Raises ValueError if `data` does not hold exactly size_x * size_y
    cells: a length mismatch means the message is not what its metadata
    claims, and reshaping whatever fits would evaluate the detection
    window against a misaligned grid with no visible symptom.
    """
    flat = np.asarray(data, dtype=np.int16)
    expected = int(size_x) * int(size_y)
    if flat.size != expected:
        raise ValueError(
            f'costmap data length {flat.size} does not match declared '
            f'size_x={size_x} * size_y={size_y} = {expected}')
    grid = flat.reshape((int(size_y), int(size_x)))
    return np.where(grid == NO_INFORMATION, UNKNOWN_SENTINEL, grid)
