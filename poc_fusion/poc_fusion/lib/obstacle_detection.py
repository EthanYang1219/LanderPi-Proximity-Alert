"""Pure obstacle-detection logic over a costmap window.

No ROS, no hardware: grids, masks and thresholds are plain numbers/arrays.
"""

import numpy as np
from scipy import ndimage

from .window_geometry import area_cm2_to_cells

_EIGHT_CONNECTIVITY = np.ones((3, 3), dtype=int)


def occupancy_fraction(values, lethal_threshold):
    """Fraction of `values` (1-D array of masked cell costs) that are lethal.

    Unknown cells (-1) are treated as not-lethal.
    """
    values = np.asarray(values)
    if values.size == 0:
        return 0.0
    lethal = values >= lethal_threshold
    return float(np.count_nonzero(lethal)) / float(values.size)


def largest_cluster_cells(grid, mask, lethal_threshold):
    """Size (cell count) of the largest 8-connected lethal cluster in `mask`.

    Unknown cells (-1) are treated as not-lethal. 8-connectivity is used
    because point-cloud marks arrive as speckled blobs that frequently
    touch only diagonally; 4-connectivity fragments them below threshold.
    """
    lethal = (grid >= lethal_threshold) & mask
    labeled, num_features = ndimage.label(lethal, structure=_EIGHT_CONNECTIVITY)
    if num_features == 0:
        return 0
    sizes = ndimage.sum(lethal, labeled, index=range(1, num_features + 1))
    return int(sizes.max())


def should_stop(grid, mask, resolution, lethal_threshold,
                 min_occupancy_fraction, min_cluster_area_cm2):
    """True if the masked window is dense enough or has a large enough
    lethal cluster to warrant stopping.
    """
    fraction = occupancy_fraction(grid[mask], lethal_threshold)
    fraction_trigger = fraction >= min_occupancy_fraction

    cluster_cells = largest_cluster_cells(grid, mask, lethal_threshold)
    min_cluster_cells = area_cm2_to_cells(min_cluster_area_cm2, resolution)
    area_trigger = cluster_cells >= min_cluster_cells

    return bool(fraction_trigger or area_trigger)
