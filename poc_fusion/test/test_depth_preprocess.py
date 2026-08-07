import numpy as np

from poc_fusion.lib.depth_preprocess import (
    invalid_pixel_mask,
    invalid_fraction,
    apply_roi_mask,
    median_filter_depth,
)


# --- invalid_pixel_mask ------------------------------------------------------

def test_invalid_pixel_mask_flags_exact_zero():
    # mono16 is unsigned; invalid pixels are exactly 0, never NaN.
    depth = np.array([[0, 500, 0], [1200, 0, 999]], dtype=np.uint16)
    mask = invalid_pixel_mask(depth)
    expected = np.array([[True, False, True], [False, True, False]])
    assert np.array_equal(mask, expected)


def test_invalid_pixel_mask_all_valid_is_all_false():
    depth = np.full((4, 4), 1000, dtype=np.uint16)
    mask = invalid_pixel_mask(depth)
    assert not mask.any()


# --- invalid_fraction ---------------------------------------------------------

def test_invalid_fraction_all_zero_is_one():
    depth = np.zeros((5, 5), dtype=np.uint16)
    assert invalid_fraction(depth) == 1.0


def test_invalid_fraction_all_valid_is_zero():
    depth = np.full((5, 5), 800, dtype=np.uint16)
    assert invalid_fraction(depth) == 0.0


def test_invalid_fraction_partial():
    # 4 of 16 pixels are zero -> 0.25
    depth = np.full((4, 4), 500, dtype=np.uint16)
    depth[0, :] = 0
    assert invalid_fraction(depth) == 0.25


# --- apply_roi_mask -----------------------------------------------------------

def test_apply_roi_mask_zeros_pixels_inside_box():
    # The ROI box marks the arm's self-occlusion region; pixels inside it
    # are invalidated (set to 0), pixels outside are untouched.
    depth = np.full((6, 6), 700, dtype=np.uint16)
    masked = apply_roi_mask(depth, row_min=1, row_max=3, col_min=2, col_max=4)
    assert (masked[1:3, 2:4] == 0).all()


def test_apply_roi_mask_leaves_pixels_outside_box_untouched():
    depth = np.full((6, 6), 700, dtype=np.uint16)
    masked = apply_roi_mask(depth, row_min=1, row_max=3, col_min=2, col_max=4)
    outside = masked.copy()
    outside[1:3, 2:4] = 700  # restore the box to compare the rest
    assert (outside == 700).all()


def test_apply_roi_mask_does_not_mutate_input():
    depth = np.full((6, 6), 700, dtype=np.uint16)
    original = depth.copy()
    apply_roi_mask(depth, row_min=0, row_max=2, col_min=0, col_max=2)
    assert np.array_equal(depth, original)


def test_apply_roi_mask_zero_width_box_masks_nothing():
    # The actual Task 4 config placeholder uses a zero-width box
    # (row_min == row_max, col_min == col_max) to mask NOTHING -- the
    # "start permissive" design decision. Task 11 replaces it with real
    # arm-occlusion bounds.
    depth = np.full((6, 6), 700, dtype=np.uint16)
    masked = apply_roi_mask(depth, row_min=2, row_max=2, col_min=3, col_max=3)
    assert (masked == 700).all()


# --- median_filter_depth -------------------------------------------------------

def test_median_filter_depth_removes_single_pixel_spike():
    depth = np.full((5, 5), 600, dtype=np.uint16)
    depth[2, 2] = 60000  # spike, e.g. sensor noise
    filtered = median_filter_depth(depth, kernel_size=3)
    assert filtered[2, 2] == 600


def test_median_filter_depth_preserves_dtype():
    depth = np.full((5, 5), 600, dtype=np.uint16)
    filtered = median_filter_depth(depth, kernel_size=3)
    assert filtered.dtype == np.uint16


def test_median_filter_depth_preserves_shape():
    depth = np.full((7, 9), 600, dtype=np.uint16)
    filtered = median_filter_depth(depth, kernel_size=3)
    assert filtered.shape == (7, 9)
