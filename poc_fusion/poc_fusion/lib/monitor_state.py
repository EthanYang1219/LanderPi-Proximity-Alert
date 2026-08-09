"""Pure decision logic for the Task 7 costmap stop monitor.

No ROS, no hardware: ages, bounds and lifecycle labels are plain numbers and
strings so every rule here is unit-testable off-robot.

WHY THIS IS NOT A BOOLEAN
-------------------------
"Costmap node exists" does not equal "costmap is operational." A stop monitor
whose authoritative state is a bare `obstacle_detected` boolean cannot
distinguish

    "I received a fresh, valid costmap and there is nothing in the window"

from

    "I have received nothing at all, and I have no idea what is in front of me"

because both render as False. Those two must never share a code path or an
output value, so the authoritative state here is a tri-state:

    CLEAR     - fresh, valid costmap; window holds no qualifying lethal cells
    OBSTACLE  - fresh, valid costmap; window does hold them
    UNKNOWN   - no costmap yet / stale costmap / costmap not ACTIVE /
                TF lookup failed for the latest costmap

UNKNOWN is a first-class output, not an error path. `should_publish_bool()`
exists so the downstream `std_msgs/Bool` compatibility topic stays SILENT
while degraded rather than emitting a False that any consumer would read as
"clear". `bool_value()` raises on UNKNOWN for the same reason: there is no
correct boolean for "I do not know", so asking for one is a bug.

Camera health is a SEPARATE axis (`camera_health()`), deliberately not folded
into `evaluate_state()`. A dead depth camera still leaves a valid LiDAR-only
costmap, which is a perfectly good CLEAR/OBSTACLE input -- it degrades the
fusion contribution, not the costmap. Reporting both independently is what
lets a trial audit tell "fusion active" from "LiDAR-only", which is the
paper's actual contribution.
"""

from collections import namedtuple

STATE_CLEAR = 'CLEAR'
STATE_OBSTACLE = 'OBSTACLE'
STATE_UNKNOWN = 'UNKNOWN'

# The lifecycle label a nav2 lifecycle node reports when it is fully up.
LIFECYCLE_ACTIVE = 'active'

CAMERA_FUSION_ACTIVE = 'FUSION_ACTIVE'
CAMERA_LIDAR_ONLY = 'LIDAR_ONLY'

REASON_NO_COSTMAP = 'no_costmap_received'
REASON_NOT_ACTIVE = 'costmap_not_active'
REASON_STALE = 'costmap_stale'
REASON_TF_FAILED = 'tf_lookup_failed'
REASON_LETHAL = 'lethal_cells_in_window'
REASON_WINDOW_CLEAR = 'window_clear'

Evaluation = namedtuple('Evaluation', ['state', 'reason'])


def evaluate_state(costmap_age_s, staleness_bound_s, lifecycle_state,
                   tf_ok, obstacle_in_window):
    """The monitor's authoritative tri-state.

    Args:
        costmap_age_s: seconds since the last costmap message was received,
            or None if none has ever been received. None is the fail-safe
            startup input and yields UNKNOWN, never CLEAR.
        staleness_bound_s: the maximum age at which a costmap still counts as
            fresh. Inclusive -- an age exactly equal to the bound is fresh.
        lifecycle_state: the costmap lifecycle node's positively-confirmed
            state label, or None if it has never been successfully queried.
            Anything other than LIFECYCLE_ACTIVE yields UNKNOWN; absence of
            an answer is not an affirmative answer.
        tf_ok: whether the odom->base_link lookup succeeded at the latest
            costmap message's stamp.
        obstacle_in_window: the result of the latest window evaluation.

    The checks are ordered most-fundamental first so `reason` names the
    root condition rather than a downstream symptom.
    """
    if costmap_age_s is None:
        return Evaluation(STATE_UNKNOWN, REASON_NO_COSTMAP)
    if lifecycle_state != LIFECYCLE_ACTIVE:
        return Evaluation(STATE_UNKNOWN, REASON_NOT_ACTIVE)
    if costmap_age_s > staleness_bound_s:
        return Evaluation(STATE_UNKNOWN, REASON_STALE)
    if not tf_ok:
        return Evaluation(STATE_UNKNOWN, REASON_TF_FAILED)
    if obstacle_in_window:
        return Evaluation(STATE_OBSTACLE, REASON_LETHAL)
    return Evaluation(STATE_CLEAR, REASON_WINDOW_CLEAR)


def should_publish_bool(state):
    """Whether the derived `std_msgs/Bool` topic may speak for `state`.

    False for UNKNOWN: while degraded the node publishes NOTHING on the Bool.
    Publishing False there would hand every downstream consumer the exact
    CLEAR/UNKNOWN conflation the tri-state exists to prevent.
    """
    return state in (STATE_CLEAR, STATE_OBSTACLE)


def bool_value(state):
    """The Bool payload for a determinate state. Raises on UNKNOWN."""
    if state == STATE_OBSTACLE:
        return True
    if state == STATE_CLEAR:
        return False
    raise ValueError(
        f'no boolean rendering exists for state {state!r}; call '
        f'should_publish_bool() first and publish nothing when it is False')


def camera_health(age_s, timeout_s):
    """Depth-camera liveness, the second and independent degradation axis.

    `age_s` is seconds since the last cleaned depth frame, or None if none
    has ever arrived -- which reports LIDAR_ONLY, the same fail-safe rule the
    costmap side uses: never assume fusion is running before evidence it is.
    """
    if age_s is None:
        return CAMERA_LIDAR_ONLY
    if age_s > timeout_s:
        return CAMERA_LIDAR_ONLY
    return CAMERA_FUSION_ACTIVE


def is_identity_quaternion(x, y, z, w, tol):
    """Whether (x, y, z, w) is within `tol` of no rotation at all.

    `window_geometry.window_mask()` maps grid indices to world coordinates
    with a pure translation, which is only correct if the costmap's origin
    carries no rotation. nav2_costmap_2d never rotates its grid, so this
    holds -- checking it makes that the one explicit assumption rather than
    an implicit one. q and -q are the same rotation, so the sign of w is
    normalised away before comparing.
    """
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    return (abs(x) <= tol and abs(y) <= tol
            and abs(z) <= tol and abs(w - 1.0) <= tol)
