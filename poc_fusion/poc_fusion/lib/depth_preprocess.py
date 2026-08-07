"""Pure array logic for the depth-preprocessing node.

No ROS, no cv_bridge: depth frames are passed in as plain numpy arrays
(mono16 / 16UC1 buffers -- unsigned 16-bit millimetres), never as messages.
"""

import numpy as np
from scipy import ndimage

# mono16 (and 16UC1, the identical buffer layout under a different encoding
# string) is an unsigned integer type: NaN cannot occur, so invalid pixels
# are exactly 0. There is deliberately no NaN branch here -- see Step 2 of
# the Task 4 brief. If the driver is ever reconfigured to 32FC1, both 0.0
# and NaN become possible and this module must change along with the
# node's startup encoding assertion.


def invalid_pixel_mask(depth):
    """Boolean mask, True where `depth` is an invalid (zero) pixel."""
    return depth == 0


def invalid_fraction(depth):
    """Fraction of `depth` that is invalid (zero)."""
    depth = np.asarray(depth)
    if depth.size == 0:
        return 0.0
    return float(np.count_nonzero(invalid_pixel_mask(depth))) / float(depth.size)


def apply_roi_mask(depth, row_min, row_max, col_min, col_max):
    """Return a copy of `depth` with the self-arm ROI box invalidated (0).

    The box is the region of the frame occupied by the robot's own arm, so
    depth values inside it are the arm itself, not a real obstacle return.
    Rows/cols follow numpy slicing conventions (`row_max`/`col_max`
    exclusive). The caller passes explicit bounds rather than a config
    dict, mirroring `lib/window_geometry.py`'s style.
    """
    masked = np.array(depth, copy=True)
    masked[row_min:row_max, col_min:col_max] = 0
    return masked


def median_filter_depth(depth, kernel_size):
    """Median-filter `depth` with a square kernel, preserving dtype/shape.

    Thin wrapper over scipy.ndimage.median_filter: the logic of its own is
    casting the result back to the input dtype, since median_filter can
    otherwise promote the output type.
    """
    depth = np.asarray(depth)
    filtered = ndimage.median_filter(depth, size=kernel_size)
    return filtered.astype(depth.dtype)


def clean_depth(depth, row_min, row_max, col_min, col_max, kernel_size):
    """Full Step 3+4 pipeline: ROI-mask, median-filter, ROI-mask again.

    The median filter can pull valid neighbour values across the ROI
    boundary and partially un-mask the self-arm box at its edges (Task 4
    review, Important 2). Reapplying the mask after filtering makes the ROI
    mask the pipeline's final word regardless of kernel size, so the arm is
    never reported as an obstacle once Task 11 supplies a real (non-zero-
    width) box.
    """
    masked = apply_roi_mask(depth, row_min, row_max, col_min, col_max)
    filtered = median_filter_depth(masked, kernel_size)
    return apply_roi_mask(filtered, row_min, row_max, col_min, col_max)
