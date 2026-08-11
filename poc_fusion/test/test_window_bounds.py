"""The stop window must fit BETWEEN the robot's bumper and the frozen
avoidance trigger. Nothing tested that relationship before, which is why
`window_forward_m: 1.0` sat against `safety_distance: 0.20` undetected until
it was found by inspection on 2026-08-11.

The conflict it missed: the gate zeroed all motion at 1.0 m, so the robot
could never close to the 0.20 m the avoidance controller needs to start a
maneuver. Measured consequence in Task 9 Part 2 -- 186 consecutive `True`
readings, 30.9 s, the gate deleting 46 live forward commands.

These tests are the regression guard. `safety_distance` is imported from the
FROZEN `proximity_alert` package rather than restated, so if that value ever
moves, this fails instead of silently going stale.
"""
import os
import sys

import pytest
import yaml

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), '..', '..', 'proximity_alert'))

from proximity_alert.avoidance import AvoidanceConfig  # noqa: E402

from poc_fusion.lib.window_bounds import (  # noqa: E402
    feasible_window_forward_m,
    max_window_forward_m,
    max_window_half_width_m,
    min_window_forward_m,
)

# --- Measured geometry ----------------------------------------------------
# base_link -> lidar_frame x offset. URDF joint chain: base_link ->
# back_shell_middle (-0.00443) -> lidar_frame (+0.077448) = 0.073018.
# Cross-checked against live TF on 2026-08-11: base_footprint -> lidar_frame
# translation [0.073, 0.000, 0.093].
LIDAR_X_OFFSET_M = 0.0730

# poc_fusion/config/costmap_params.yaml -> resolution: 0.05
COSTMAP_RESOLUTION_M = 0.05

# --- PENDING PHYSICAL MEASUREMENT -----------------------------------------
# Deliberately None, not a plausible-looking placeholder. The config guards
# below SKIP while these are None rather than passing on invented numbers --
# a green suite here must mean the deployed window was checked against real
# measurements, never that a default happened to satisfy an assumed bound.
#
# MEASURED_FRONT_EXTENT_M      ruler: base_link -> frontmost physical point.
#                              URDF gives only a collision MESH, and TF was
#                              already 7 mm off the ruler in bench Phase 0,
#                              so this is measured, not derived.
# MEASURED_ROBOT_HALF_WIDTH_M  ruler: half the widest overall width.
# MEASURED_STOPPING_DISTANCE_M distance travelled after a zero command at
#                              forward_speed 0.20 m/s. The Task 9 Part 2 run
#                              measured 0.009 m but at 0.05 m/s -- a 4x
#                              extrapolation is not evidence, so it is
#                              re-measured at the operating speed.
MEASURED_FRONT_EXTENT_M = None
MEASURED_ROBOT_HALF_WIDTH_M = None
MEASURED_STOPPING_DISTANCE_M = None

# Lateral gap allowed beyond the robot's own width before the backstop stops
# treating an object as "about to be hit". Not a measurement -- a stated
# design margin, sized at half a costmap cell.
SWEPT_PATH_CLEARANCE_M = 0.025

_PENDING = (MEASURED_FRONT_EXTENT_M is None
            or MEASURED_ROBOT_HALF_WIDTH_M is None
            or MEASURED_STOPPING_DISTANCE_M is None)
_pending_reason = 'awaiting physical measurement; see module docstring'

_CFG_DIR = os.path.join(os.path.dirname(__file__), '..', 'config')


def _load(name):
    with open(os.path.join(_CFG_DIR, name)) as f:
        return yaml.safe_load(f)


def _monitor_params():
    doc = _load('stop_monitor_params.yaml')
    node = doc[next(iter(doc))]
    return node['ros__parameters']


# --- Pure envelope math ---------------------------------------------------

def test_max_forward_leaves_room_for_cell_quantisation():
    """The window edge lands on a costmap cell CENTRE, and the grid is not
    aligned to the robot (rolling window in odom), so the effective edge
    jitters by up to half a cell. The bound must absorb that."""
    bound = max_window_forward_m(
        safety_distance_m=0.20,
        lidar_x_offset_m=0.0730,
        stopping_distance_m=0.0,
        cell_size_m=0.05,
    )
    # Obstacle sits at 0.20 + 0.073 = 0.273 m when avoidance trips.
    # Half a cell of jitter (0.025) must stay clear of it.
    assert bound == pytest.approx(0.273 - 0.025, abs=1e-9)


def test_max_forward_shrinks_by_avoidance_overshoot():
    """The robot coasts past the trigger before it halts, so the obstacle
    ends up CLOSER than safety_distance. More overshoot -> tighter bound."""
    slow = max_window_forward_m(0.20, 0.0730, 0.0, 0.05)
    fast = max_window_forward_m(0.20, 0.0730, 0.04, 0.05)
    assert fast < slow
    assert fast == pytest.approx(slow - 0.04, abs=1e-9)


def test_min_forward_covers_bumper_plus_stopping_distance():
    """Below this the gate fires too late to prevent contact."""
    bound = min_window_forward_m(
        front_extent_m=0.10,
        stopping_distance_m=0.036,
        cell_size_m=0.05,
    )
    # Worst-case jitter shrinks the effective window, so it must be padded.
    assert bound == pytest.approx(0.10 + 0.036 + 0.025, abs=1e-9)


def test_feasible_band_is_empty_when_bounds_cross():
    """A robot whose bumper reaches past its own avoidance trigger admits no
    valid window. The function must say so rather than return a nonsense
    range -- this is the failure mode that would otherwise be papered over."""
    lo, hi = feasible_window_forward_m(
        safety_distance_m=0.20,
        lidar_x_offset_m=0.0730,
        front_extent_m=0.30,
        stopping_distance_m=0.05,
        cell_size_m=0.05,
    )
    assert lo > hi


def test_half_width_confines_backstop_to_swept_path():
    """`front` is the minimum over a 180 deg arc (front_arc_deg = 180.0), so
    an obstacle at 60 deg bearing and 0.20 m range sits at x~0.17, y~0.17.
    A half-width wider than the robot's swept path lets that object hold the
    gate closed through a strafe that is legitimately clearing it."""
    bound = max_window_half_width_m(robot_half_width_m=0.09,
                                     clearance_m=0.03,
                                     cell_size_m=0.05)
    assert bound == pytest.approx(0.09 + 0.03 + 0.025, abs=1e-9)


# --- The regression guard on the DEPLOYED config --------------------------

@pytest.mark.skipif(_PENDING, reason=_pending_reason)
def test_deployed_window_forward_is_inside_the_avoidance_trigger():
    """THE test. Fails against window_forward_m: 1.0."""
    params = _monitor_params()
    cfg = AvoidanceConfig()

    bound = max_window_forward_m(
        safety_distance_m=cfg.safety_distance,
        lidar_x_offset_m=LIDAR_X_OFFSET_M,
        stopping_distance_m=MEASURED_STOPPING_DISTANCE_M,
        cell_size_m=COSTMAP_RESOLUTION_M,
    )
    assert params['window_forward_m'] < bound, (
        f"window_forward_m={params['window_forward_m']} would zero motion "
        f"before the avoidance controller can trip at "
        f"front<={cfg.safety_distance} m; must be < {bound:.4f} m"
    )


@pytest.mark.skipif(_PENDING, reason=_pending_reason)
def test_deployed_window_forward_still_prevents_contact():
    params = _monitor_params()
    bound = min_window_forward_m(
        front_extent_m=MEASURED_FRONT_EXTENT_M,
        stopping_distance_m=MEASURED_STOPPING_DISTANCE_M,
        cell_size_m=COSTMAP_RESOLUTION_M,
    )
    assert params['window_forward_m'] > bound, (
        f"window_forward_m={params['window_forward_m']} fires too late to "
        f"stop before contact; must be > {bound:.4f} m"
    )


@pytest.mark.skipif(_PENDING, reason=_pending_reason)
def test_deployed_half_width_does_not_veto_strafes():
    params = _monitor_params()
    bound = max_window_half_width_m(
        robot_half_width_m=MEASURED_ROBOT_HALF_WIDTH_M,
        clearance_m=SWEPT_PATH_CLEARANCE_M,
        cell_size_m=COSTMAP_RESOLUTION_M,
    )
    assert params['window_half_width_m'] <= bound, (
        f"window_half_width_m={params['window_half_width_m']} extends beyond "
        f"the robot's swept path; an obstacle being strafed past would hold "
        f"the gate closed. Must be <= {bound:.4f} m"
    )
