import numpy as np

from poc_fusion.lib.obstacle_detection import (
    occupancy_fraction,
    largest_cluster_cells,
    should_stop,
)


LETHAL = 100


# --- occupancy_fraction -----------------------------------------------------

def test_occupancy_fraction_all_free_is_zero():
    values = np.zeros(16, dtype=np.int16)
    assert occupancy_fraction(values, LETHAL) == 0.0


def test_occupancy_fraction_all_lethal_is_one():
    values = np.full(16, LETHAL, dtype=np.int16)
    assert occupancy_fraction(values, LETHAL) == 1.0


def test_occupancy_fraction_treats_unknown_as_not_lethal():
    # -1 (unknown) must NOT count as lethal even though it sits below
    # lethal_threshold only by convention: this pins the intent against a
    # future change that casts costs to an unsigned type, where -1 would
    # wrap to 255 and silently become "lethal".
    values = np.array([-1, -1, LETHAL, LETHAL], dtype=np.int16)
    assert occupancy_fraction(values, LETHAL) == 0.5


# --- largest_cluster_cells ---------------------------------------------------

def test_largest_cluster_cells_empty_grid_is_zero():
    grid = np.zeros((5, 5), dtype=np.int16)
    mask = np.ones((5, 5), dtype=bool)
    assert largest_cluster_cells(grid, mask, LETHAL) == 0


def test_largest_cluster_cells_diagonal_chain_is_one_cluster_under_8_connectivity():
    # A diagonal-only chain of 5 lethal cells. Under 8-connectivity this is
    # ONE connected component of size 5. Under 4-connectivity (the design
    # this replaced) it fragments into 5 separate size-1 components and the
    # area trigger never fires. This test pins the 8-connectivity choice.
    grid = np.zeros((5, 5), dtype=np.int16)
    mask = np.ones((5, 5), dtype=bool)
    for i in range(5):
        grid[i, i] = LETHAL
    assert largest_cluster_cells(grid, mask, LETHAL) == 5


def test_largest_cluster_cells_respects_mask():
    # Cells outside the mask must not contribute even if lethal.
    grid = np.full((5, 5), LETHAL, dtype=np.int16)
    mask = np.zeros((5, 5), dtype=bool)
    mask[0:2, 0:2] = True  # only a 2x2 block is inside the detection window
    assert largest_cluster_cells(grid, mask, LETHAL) == 4


# --- should_stop -------------------------------------------------------------

def test_should_stop_empty_grid_is_false():
    grid = np.zeros((20, 20), dtype=np.int16)
    mask = np.ones((20, 20), dtype=bool)
    assert should_stop(grid, mask, resolution=0.1, lethal_threshold=LETHAL,
                        min_occupancy_fraction=0.30,
                        min_cluster_area_cm2=100) is False


def test_should_stop_fully_lethal_window_is_true():
    grid = np.zeros((20, 20), dtype=np.int16)
    mask = np.zeros((20, 20), dtype=bool)
    mask[5:10, 5:10] = True
    grid[mask] = LETHAL
    assert should_stop(grid, mask, resolution=0.1, lethal_threshold=LETHAL,
                        min_occupancy_fraction=0.30,
                        min_cluster_area_cm2=100) is True


def test_should_stop_triggers_on_diagonal_cluster_area_alone():
    # A diagonal chain of 5 lethal cells inside a big, mostly-free window:
    # occupancy fraction stays tiny (5/400 = 1.25%, well under 0.30) so this
    # can only trigger via the cluster-area condition, and only because the
    # 5 diagonal cells are counted as one 8-connected cluster.
    #
    # resolution=0.1 -> one cell = 100 cm^2. min_cluster_area_cm2=250 -> the
    # threshold is 2.5 cells: a single isolated lethal cell (100 cm^2, what
    # a buggy 4-connectivity implementation would see as the largest
    # fragment of this diagonal chain) does NOT meet it, but the 5-cell
    # 8-connected cluster (500 cm^2) does. This is what makes the test
    # actually discriminate the connectivity choice, not just pass under
    # either connectivity because a single cell already clears the bar.
    grid = np.zeros((20, 20), dtype=np.int16)
    mask = np.ones((20, 20), dtype=bool)  # 400 cells
    for i in range(5):
        grid[i, i] = LETHAL
    assert should_stop(grid, mask, resolution=0.1, lethal_threshold=LETHAL,
                        min_occupancy_fraction=0.30,
                        min_cluster_area_cm2=250) is True


def test_should_stop_subthreshold_speckle_is_false():
    # Isolated single-cell lethal marks, not touching each other even
    # diagonally. At resolution=0.05, one cell = 25 cm^2, under the 100
    # cm^2 area threshold, and occupancy fraction is tiny too.
    grid = np.zeros((20, 20), dtype=np.int16)
    mask = np.ones((20, 20), dtype=bool)
    grid[2, 2] = LETHAL
    grid[10, 15] = LETHAL
    grid[17, 4] = LETHAL
    assert should_stop(grid, mask, resolution=0.05, lethal_threshold=LETHAL,
                        min_occupancy_fraction=0.30,
                        min_cluster_area_cm2=100) is False


def test_should_stop_unknown_cells_are_not_lethal():
    # Fill the whole window with "unknown" (-1) except a tiny lethal
    # speckle. If -1 were ever (mis)treated as lethal this would trip the
    # occupancy-fraction condition; it must not.
    grid = np.full((20, 20), -1, dtype=np.int16)
    mask = np.ones((20, 20), dtype=bool)
    grid[2, 2] = LETHAL
    assert should_stop(grid, mask, resolution=0.05, lethal_threshold=LETHAL,
                        min_occupancy_fraction=0.30,
                        min_cluster_area_cm2=100) is False
