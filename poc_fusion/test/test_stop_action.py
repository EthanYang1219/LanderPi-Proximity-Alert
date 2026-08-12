"""Tests for the Task 9 stop action's pure decision logic.

The stop action consumes the monitor's DERIVED `std_msgs/Bool`
(`obstacle_detected`), not its authoritative tri-state. That is the whole
reason this module needs its own fail-safe rules.

`monitor_state.should_publish_bool()` makes the Bool topic go SILENT while the
monitor is UNKNOWN, rather than publishing False. That is correct for the
monitor -- False would read as "clear" -- but it means a Bool subscriber sees
UNKNOWN as *a gap in the stream*, indistinguishable at the message level from
"the monitor died", "the network dropped it", or "nothing has started yet".

So the stop action must never treat absence as permission to drive:

  - no Bool ever received            -> STOP  (not drive)
  - Bool received but now stale      -> STOP  (not drive)
  - Bool received, fresh, True       -> STOP, and fire the alert on the edge
  - Bool received, fresh, False      -> drive

Every test below names the regression it catches. The ones that matter most
are the two that fail if silence is ever allowed to mean "clear".
"""

import pytest

from poc_fusion.lib.stop_action import (
    REASON_CLEAR,
    REASON_NO_SIGNAL,
    REASON_OBSTACLE,
    REASON_SIGNAL_STALE,
    evaluate_stop_action,
)

BOUND = 0.5


def test_no_bool_ever_received_stops_rather_than_drives():
    """Regression: startup must not be mistaken for a clear path.

    Before the monitor has published anything there is no evidence of
    clearance, and 'no evidence of an obstacle' is not 'evidence of no
    obstacle'.
    """
    action = evaluate_stop_action(
        obstacle=None, signal_age_s=None,
        staleness_bound_s=BOUND, previous_reason=None)
    assert action.stop is True
    assert action.reason == REASON_NO_SIGNAL


def test_stale_bool_stops_rather_than_drives():
    """Regression: a monitor that went UNKNOWN (or died) must not free the
    robot to drive. The Bool simply stops arriving in both cases."""
    action = evaluate_stop_action(
        obstacle=False, signal_age_s=BOUND + 0.01,
        staleness_bound_s=BOUND, previous_reason=REASON_CLEAR)
    assert action.stop is True
    assert action.reason == REASON_SIGNAL_STALE


def test_stale_wins_over_a_stale_true_reading():
    """A stale True is still stale: we stop, but we report staleness, not a
    live obstacle, so the alert does not cry wolf on data we no longer trust."""
    action = evaluate_stop_action(
        obstacle=True, signal_age_s=BOUND + 0.01,
        staleness_bound_s=BOUND, previous_reason=REASON_CLEAR)
    assert action.stop is True
    assert action.reason == REASON_SIGNAL_STALE
    assert action.fire_alert is False


def test_age_exactly_at_the_bound_is_still_fresh():
    """The bound is inclusive, matching monitor_state.evaluate_state()."""
    action = evaluate_stop_action(
        obstacle=False, signal_age_s=BOUND,
        staleness_bound_s=BOUND, previous_reason=REASON_CLEAR)
    assert action.stop is False
    assert action.reason == REASON_CLEAR


def test_fresh_true_stops():
    action = evaluate_stop_action(
        obstacle=True, signal_age_s=0.1,
        staleness_bound_s=BOUND, previous_reason=REASON_CLEAR)
    assert action.stop is True
    assert action.reason == REASON_OBSTACLE


def test_fresh_false_drives():
    action = evaluate_stop_action(
        obstacle=False, signal_age_s=0.1,
        staleness_bound_s=BOUND, previous_reason=REASON_CLEAR)
    assert action.stop is False
    assert action.reason == REASON_CLEAR
    assert action.fire_alert is False


def test_alert_fires_on_the_rising_edge():
    action = evaluate_stop_action(
        obstacle=True, signal_age_s=0.1,
        staleness_bound_s=BOUND, previous_reason=REASON_CLEAR)
    assert action.fire_alert is True


def test_alert_fires_on_the_first_ever_obstacle_from_startup():
    """previous_reason None (nothing decided yet) is still a rising edge."""
    action = evaluate_stop_action(
        obstacle=True, signal_age_s=0.1,
        staleness_bound_s=BOUND, previous_reason=None)
    assert action.fire_alert is True


def test_alert_does_not_refire_while_the_obstacle_persists():
    """Regression: the buzzer must sound once per encounter, not at every
    costmap tick for as long as the obstacle is there."""
    action = evaluate_stop_action(
        obstacle=True, signal_age_s=0.1,
        staleness_bound_s=BOUND, previous_reason=REASON_OBSTACLE)
    assert action.stop is True
    assert action.fire_alert is False


def test_alert_does_not_fire_when_the_signal_is_merely_missing():
    """Regression: a dead monitor must stop the robot SILENTLY-as-to-the-alert
    -- the alert means 'obstacle seen', and nothing was seen here."""
    for reason_in in (None, REASON_CLEAR, REASON_OBSTACLE):
        action = evaluate_stop_action(
            obstacle=None, signal_age_s=None,
            staleness_bound_s=BOUND, previous_reason=reason_in)
        assert action.fire_alert is False


def test_missing_age_with_a_present_reading_is_treated_as_no_signal():
    """Defensive: a reading with no arrival time is not evidence of anything.
    It must fail to the stop side, never to the drive side."""
    action = evaluate_stop_action(
        obstacle=True, signal_age_s=None,
        staleness_bound_s=BOUND, previous_reason=REASON_CLEAR)
    assert action.stop is True
    assert action.reason == REASON_NO_SIGNAL


def test_recovery_after_an_obstacle_clears_allows_driving_again():
    action = evaluate_stop_action(
        obstacle=False, signal_age_s=0.1,
        staleness_bound_s=BOUND, previous_reason=REASON_OBSTACLE)
    assert action.stop is False
    assert action.reason == REASON_CLEAR


def test_re_detection_after_a_gap_fires_the_alert_again():
    """After the signal was lost and the stop was held, a fresh confirmed
    obstacle is a new determinate detection and is announced."""
    action = evaluate_stop_action(
        obstacle=True, signal_age_s=0.1,
        staleness_bound_s=BOUND, previous_reason=REASON_SIGNAL_STALE)
    assert action.fire_alert is True


@pytest.mark.parametrize('reason', [
    REASON_NO_SIGNAL, REASON_SIGNAL_STALE, REASON_OBSTACLE,
])
def test_every_non_clear_reason_commands_a_stop(reason):
    """Regression guard on the reason set itself: if a new reason is ever
    added it must be classified deliberately, not default to 'drive'."""
    assert reason != REASON_CLEAR
