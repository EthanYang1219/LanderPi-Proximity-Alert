"""Tests for the stop monitor's tri-state decision logic (Task 7).

The governing safety semantic for this task: "'Costmap node exists' does not
equal 'costmap is operational.' The stop monitor must not interpret silence
from an inactive or broken costmap as 'there are no obstacles.'"

So the monitor's authoritative state is NOT a boolean. It is three states:

  CLEAR    - a fresh, valid costmap was received and the window holds no
             qualifying lethal cells;
  OBSTACLE - a fresh, valid costmap was received and the window does;
  UNKNOWN  - no costmap yet, a stale costmap, a costmap whose lifecycle node
             is not ACTIVE, or a costmap cycle whose TF lookup failed.

CLEAR and UNKNOWN must never collapse into the same value. Most of the tests
below exist specifically to fail if they ever do; each one names the
regression it catches.
"""

import pytest

from poc_fusion.lib.monitor_state import (
    CAMERA_FUSION_ACTIVE,
    CAMERA_LIDAR_ONLY,
    LIFECYCLE_ACTIVE,
    STATE_CLEAR,
    STATE_OBSTACLE,
    STATE_UNKNOWN,
    bool_value,
    camera_health,
    evaluate_state,
    is_identity_quaternion,
    should_publish_bool,
)


def _evaluate(**overrides):
    """A fully-healthy, obstacle-free evaluation, with overrides applied.

    Defaults are deliberately the happy path so each test can perturb exactly
    one input and attribute the resulting state to that one input.
    """
    kwargs = dict(
        costmap_age_s=0.05,
        staleness_bound_s=0.5,
        lifecycle_state=LIFECYCLE_ACTIVE,
        tf_ok=True,
        obstacle_in_window=False,
    )
    kwargs.update(overrides)
    return evaluate_state(**kwargs)


# --- The happy path, so the perturbation tests below mean something ---------

def test_fresh_active_costmap_with_empty_window_is_clear():
    assert _evaluate().state == STATE_CLEAR


def test_fresh_active_costmap_with_lethal_window_is_obstacle():
    assert _evaluate(obstacle_in_window=True).state == STATE_OBSTACLE


# --- Requirement 4: fail-safe startup --------------------------------------

def test_startup_before_any_costmap_is_unknown_not_clear():
    # CATCHES: a monitor that starts up believing the world is empty. If the
    # implementation initialised its state to CLEAR (or initialised a bare
    # `obstacle_detected = False`), this returns CLEAR and the test fails.
    # costmap_age_s=None is the "no costmap has ever arrived" input.
    result = _evaluate(costmap_age_s=None)
    assert result.state == STATE_UNKNOWN
    assert result.reason == 'no_costmap_received'


def test_startup_is_unknown_even_when_lifecycle_is_already_active():
    # CATCHES: inferring liveness from the lifecycle state alone. An ACTIVE
    # costmap node that has not yet published anything the monitor received
    # is still not evidence of an empty world.
    assert _evaluate(costmap_age_s=None,
                     lifecycle_state=LIFECYCLE_ACTIVE).state == STATE_UNKNOWN


# --- Requirement 3: staleness watchdog -------------------------------------

def test_stale_costmap_is_unknown_not_clear():
    # CATCHES: treating a stale costmap as fresh. Silence from a costmap that
    # has stopped publishing must not read as "no obstacles". Remove the
    # age-vs-bound comparison and this returns CLEAR.
    result = _evaluate(costmap_age_s=0.9, staleness_bound_s=0.5)
    assert result.state == STATE_UNKNOWN
    assert result.reason == 'costmap_stale'


def test_stale_costmap_is_unknown_even_when_an_obstacle_was_last_seen():
    # A stale OBSTACLE is not a fresh OBSTACLE either -- the monitor must not
    # keep asserting a stale positive as though it were current.
    assert _evaluate(costmap_age_s=0.9, obstacle_in_window=True).state == STATE_UNKNOWN


def test_costmap_exactly_at_the_staleness_bound_is_still_fresh():
    # Pins the boundary as inclusive so the bound means "older than", not
    # "at least"; guards against an off-by-one flap at the exact bound.
    assert _evaluate(costmap_age_s=0.5, staleness_bound_s=0.5).state == STATE_CLEAR


# --- Requirement 2: positive liveness, not inferred ------------------------

def test_inactive_costmap_is_unknown_not_clear():
    # CATCHES: the Task 6 Step 4 gap carried forward -- "lifecycle manager
    # alive but costmap never active". A fresh-looking message age is not
    # enough; without a positive ACTIVE confirmation the state is UNKNOWN.
    result = _evaluate(lifecycle_state='inactive')
    assert result.state == STATE_UNKNOWN
    assert result.reason == 'costmap_not_active'


def test_unqueried_lifecycle_state_is_unknown_not_clear():
    # CATCHES: defaulting the lifecycle state to "assume active" before the
    # get_state query has ever succeeded. Absence of an answer is not an
    # affirmative answer.
    assert _evaluate(lifecycle_state=None).state == STATE_UNKNOWN


@pytest.mark.parametrize('lifecycle_state',
                         ['unconfigured', 'inactive', 'finalized', 'errorprocessing'])
def test_every_non_active_lifecycle_state_is_unknown(lifecycle_state):
    assert _evaluate(lifecycle_state=lifecycle_state).state == STATE_UNKNOWN


# --- Plan Step 2: TF failure is a degraded condition, not a silent no-op ----

def test_tf_lookup_failure_is_unknown_not_clear():
    # CATCHES: skipping the cycle so quietly that the previous CLEAR stands.
    # A window that could not be placed in the grid was not evaluated at all.
    result = _evaluate(tf_ok=False)
    assert result.state == STATE_UNKNOWN
    assert result.reason == 'tf_lookup_failed'


# --- Requirement 1: the Bool output must never speak for UNKNOWN -----------

def test_bool_is_not_published_in_unknown():
    # CATCHES: publishing Bool(False) while degraded, which would let every
    # downstream consumer (Task 9's controller, Task 12's latency recorder)
    # read UNKNOWN as CLEAR. This is the single most important assertion in
    # the file.
    assert should_publish_bool(STATE_UNKNOWN) is False


def test_bool_is_published_in_clear_and_obstacle():
    assert should_publish_bool(STATE_CLEAR) is True
    assert should_publish_bool(STATE_OBSTACLE) is True


def test_bool_value_maps_only_the_two_determinate_states():
    assert bool_value(STATE_OBSTACLE) is True
    assert bool_value(STATE_CLEAR) is False


def test_bool_value_refuses_to_render_unknown():
    # CATCHES: a helper that quietly returns False for UNKNOWN. There is no
    # correct boolean for "I do not know", so asking for one is a bug and
    # must raise rather than silently answer "clear".
    with pytest.raises(ValueError):
        bool_value(STATE_UNKNOWN)


# --- Camera health is a SEPARATE degradation axis --------------------------

def test_camera_health_reports_fusion_active_within_timeout():
    assert camera_health(age_s=0.5, timeout_s=2.0) == CAMERA_FUSION_ACTIVE


def test_camera_health_reports_lidar_only_past_timeout():
    assert camera_health(age_s=2.5, timeout_s=2.0) == CAMERA_LIDAR_ONLY


def test_camera_health_before_any_frame_is_lidar_only():
    # Same fail-safe rule as the costmap: never assume fusion is running
    # before evidence that it is.
    assert camera_health(age_s=None, timeout_s=2.0) == CAMERA_LIDAR_ONLY


def test_dead_camera_does_not_make_an_active_costmap_unknown():
    # CATCHES: conflating the two degradation axes. A dead depth camera still
    # leaves a valid LiDAR-only costmap, which is a legitimate CLEAR/OBSTACLE
    # input. If camera health were folded into evaluate_state() the monitor
    # would blind itself to a costmap that is working fine.
    assert camera_health(age_s=None, timeout_s=2.0) == CAMERA_LIDAR_ONLY
    assert _evaluate().state == STATE_CLEAR


# --- Plan Step 3: the grid-rotation assumption in window_mask ---------------

def test_identity_quaternion_is_accepted():
    assert is_identity_quaternion(0.0, 0.0, 0.0, 1.0, tol=1e-6) is True


def test_rotated_grid_orientation_is_rejected():
    # window_mask() assumes grid axes are world axes. A yaw of 90 degrees on
    # the costmap origin would silently mask the wrong cells.
    assert is_identity_quaternion(0.0, 0.0, 0.7071, 0.7071, tol=1e-6) is False


def test_negated_identity_quaternion_is_accepted():
    # q and -q are the same rotation; a sign flip must not be reported as a
    # rotated grid.
    assert is_identity_quaternion(0.0, 0.0, 0.0, -1.0, tol=1e-6) is True
