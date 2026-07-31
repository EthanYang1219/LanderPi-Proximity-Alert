import math

import pytest

from proximity_alert.avoidance import DRIVE, HALT, STRAFE, TURN
from proximity_alert.nav_utils import (
    ARRIVED_OBSTACLE, ARRIVED_TARGET, CENTERING, DRIVING, STOPPED_ODOM_FAULT,
    along_track_distance, cross_track_error, decide_arrival,
    distance_from_start, lateral_correction, should_skip_controller,
)


def _decide(**kw):
    # cross_track_tolerance defaults to 0.0 here, which disables the arrival
    # centering phase -- these cases are about the stop-condition priority
    # order, and centering is exercised separately below.
    base = dict(controller_state=DRIVE, target_distance=0.0,
                along_track=0.0, odom_stale=False, already_arrived=False)
    base.update(kw)
    return decide_arrival(**base)


# --- distance math ---

def test_distance_from_start_straight_line():
    assert distance_from_start(0.0, 0.0, 3.0, 4.0) == 5.0


def test_distance_from_start_zero_when_unmoved():
    assert distance_from_start(1.0, 1.0, 1.0, 1.0) == 0.0


def test_distance_from_start_is_unsigned_displacement():
    # Driving backwards still increases distance travelled -- this is
    # displacement magnitude, not signed progress along the goal axis.
    assert distance_from_start(0.0, 0.0, -2.0, 0.0) == 2.0


# --- path-frame geometry (along-track / cross-track) ---

def test_along_and_cross_are_zero_at_the_start():
    assert along_track_distance(1.0, 2.0, 0.7, 1.0, 2.0) == 0.0
    assert cross_track_error(1.0, 2.0, 0.7, 1.0, 2.0) == 0.0


def test_cross_track_sign_is_left_positive():
    # Facing +x: a point at +y is to the robot's LEFT.
    assert cross_track_error(0.0, 0.0, 0.0, 1.0, 0.5) == pytest.approx(0.5)
    assert cross_track_error(0.0, 0.0, 0.0, 1.0, -0.5) == pytest.approx(-0.5)


def test_cross_track_sign_holds_on_a_rotated_path():
    # Facing +y: "left" is now -x. Proves the formula is not hardcoded to
    # the x-axis case.
    h = math.pi / 2
    assert cross_track_error(0.0, 0.0, h, -0.5, 1.0) == pytest.approx(0.5)
    assert cross_track_error(0.0, 0.0, h, 0.5, 1.0) == pytest.approx(-0.5)


def test_along_track_ignores_lateral_offset():
    # The whole point: 1.2 m down the line is 1.2 m of progress no matter
    # how far off to the side the robot has drifted.
    for offset in (0.0, 0.4, -0.9):
        assert along_track_distance(0.0, 0.0, 0.0, 1.2, offset) == pytest.approx(1.2)


def test_along_track_is_signed_progress_not_magnitude():
    # Backing up past the start is negative progress, unlike
    # distance_from_start which would report +2.0 here.
    assert along_track_distance(0.0, 0.0, 0.0, -2.0, 0.0) == pytest.approx(-2.0)


def test_along_track_beats_displacement_for_an_offset_robot():
    # The early-stop bug this replaced: hypot inflates progress when the
    # robot is off to the side, so a 1.25 m target would fire at 1.20 m.
    along = along_track_distance(0.0, 0.0, 0.0, 1.20, 0.40)
    assert distance_from_start(0.0, 0.0, 1.20, 0.40) > 1.25
    assert along == pytest.approx(1.20)


def test_along_and_cross_recompose_to_the_displacement():
    # (along, cross) is just the displacement rewritten in the path frame,
    # so its magnitude must be preserved.
    a = along_track_distance(1.0, -2.0, 0.9, 3.5, 4.0)
    c = cross_track_error(1.0, -2.0, 0.9, 3.5, 4.0)
    assert math.hypot(a, c) == pytest.approx(distance_from_start(1.0, -2.0, 3.5, 4.0))


# --- crab correction ---

def test_correction_steers_back_toward_the_line():
    # Drifted left (+) -> move right (-y), and vice versa.
    assert lateral_correction(0.20, kp=0.6, max_speed=0.10, deadband=0.03) < 0.0
    assert lateral_correction(-0.20, kp=0.6, max_speed=0.10, deadband=0.03) > 0.0


def test_correction_is_proportional_below_saturation():
    a = lateral_correction(0.04, kp=0.6, max_speed=10.0, deadband=0.0)
    b = lateral_correction(0.08, kp=0.6, max_speed=10.0, deadband=0.0)
    assert b == pytest.approx(2.0 * a)


def test_correction_saturates_at_max_speed():
    # A 5 m offset must not command 3 m/s sideways.
    assert lateral_correction(5.0, kp=0.6, max_speed=0.10,
                              deadband=0.03) == pytest.approx(-0.10)
    assert lateral_correction(-5.0, kp=0.6, max_speed=0.10,
                              deadband=0.03) == pytest.approx(0.10)


def test_correction_is_exactly_zero_on_the_line():
    assert lateral_correction(0.0, kp=0.6, max_speed=0.10, deadband=0.03) == 0.0


def test_deadband_silences_sub_resolution_offsets():
    assert lateral_correction(0.02, kp=0.6, max_speed=0.10, deadband=0.03) == 0.0
    assert lateral_correction(0.04, kp=0.6, max_speed=0.10, deadband=0.03) != 0.0


def test_zero_gain_disables_the_correction():
    assert lateral_correction(2.0, kp=0.0, max_speed=0.10, deadband=0.0) == 0.0


def test_ramp_scales_the_command_without_changing_its_sign():
    full = lateral_correction(0.20, kp=0.6, max_speed=0.10, deadband=0.03)
    half = lateral_correction(0.20, kp=0.6, max_speed=0.10, deadband=0.03,
                              ramp_scale=0.5)
    assert half == pytest.approx(full * 0.5)


def test_correction_converges_monotonically_from_a_large_offset():
    # Closed loop e_dot = -kp*e: simulate it and assert the error shrinks
    # every step and never crosses the line (no overshoot), which is the
    # stability claim the docstring makes.
    # 600 steps at 0.05 s = 30 s. Most of that is the saturated stretch --
    # closing 2.0 m at the 0.10 m/s clamp alone takes ~20 s -- so the
    # horizon has to outlast it to reach the proportional tail.
    e, dt = 2.0, 0.05
    for _ in range(600):
        v = lateral_correction(e, kp=0.6, max_speed=0.10, deadband=0.0)
        prev = e
        e += v * dt
        assert abs(e) < abs(prev)
        assert e > 0.0          # never overshoots past the line
    assert e < 0.05


# --- backward compatibility: target_distance disabled ---

def test_disabled_target_never_arrives_however_far_it_drives():
    assert _decide(target_distance=0.0, along_track=100.0) == DRIVING


def test_disabled_target_ignores_stale_odom():
    # No watchdog existed before this feature; leaving target_distance at
    # its default must not introduce one.
    assert _decide(target_distance=0.0, odom_stale=True) == DRIVING


def test_disabled_target_still_reports_obstacle_halt():
    assert _decide(target_distance=0.0, controller_state=HALT) == ARRIVED_OBSTACLE


# --- target-distance arrival ---

def test_arrives_when_target_reached_while_driving():
    assert _decide(target_distance=2.0, along_track=2.0) == ARRIVED_TARGET


def test_arrives_when_target_overshot():
    assert _decide(target_distance=2.0, along_track=2.4) == ARRIVED_TARGET


def test_drives_when_below_target():
    assert _decide(target_distance=2.0, along_track=1.9) == DRIVING


def test_arrival_deferred_mid_strafe():
    assert _decide(controller_state=STRAFE, target_distance=2.0,
                   along_track=5.0) == DRIVING


def test_arrival_deferred_mid_turn():
    assert _decide(controller_state=TURN, target_distance=2.0,
                   along_track=5.0) == DRIVING


def test_arrival_latch_survives_later_ticks():
    assert _decide(already_arrived=True, target_distance=2.0,
                   along_track=0.0) == ARRIVED_TARGET


# --- obstacle halt ---

def test_halt_reports_arrived_obstacle_not_target():
    # Controller gave up well before the target was reached.
    assert _decide(controller_state=HALT, target_distance=5.0,
                   along_track=1.0) == ARRIVED_OBSTACLE


def test_target_reached_wins_over_halt_only_from_drive():
    # Reaching the target during a HALT does not count as arriving at B:
    # the robot is stopped by an obstacle, not by the odometer.
    assert _decide(controller_state=HALT, target_distance=2.0,
                   along_track=2.5) == ARRIVED_OBSTACLE


# --- stale-odom watchdog ---

def test_stale_odom_stops_when_target_set():
    assert _decide(target_distance=2.0, odom_stale=True,
                   along_track=0.5) == STOPPED_ODOM_FAULT


def test_stale_odom_wins_over_untrustworthy_distance():
    # along_track is derived from odom, so stale odom means the
    # distance is stale too and must not be trusted to declare arrival.
    assert _decide(target_distance=2.0, odom_stale=True,
                   along_track=99.0) == STOPPED_ODOM_FAULT


def test_latched_arrival_outranks_stale_odom():
    assert _decide(already_arrived=True, target_distance=2.0,
                   odom_stale=True) == ARRIVED_TARGET


# --- arrival centering phase ---

def _centering(**kw):
    base = dict(controller_state=DRIVE, target_distance=2.0, along_track=2.0,
                odom_stale=False, already_arrived=False, cross_track_err=0.0,
                cross_track_tolerance=0.05, centering_elapsed=0.0,
                centering_timeout=5.0)
    base.update(kw)
    return decide_arrival(**base)


def test_centers_before_arriving_when_off_the_line():
    assert _centering(cross_track_err=0.30) == CENTERING


def test_arrives_immediately_when_already_within_tolerance():
    assert _centering(cross_track_err=0.02) == ARRIVED_TARGET


def test_centering_tolerance_is_unsigned():
    assert _centering(cross_track_err=-0.30) == CENTERING


def test_centering_gives_up_at_timeout_and_arrives_anyway():
    # A blocked flank can make the offset uncloseable; the run must still
    # terminate rather than crab at the finish line forever.
    assert _centering(cross_track_err=0.30, centering_elapsed=5.0) == ARRIVED_TARGET


def test_zero_tolerance_disables_centering_entirely():
    assert _centering(cross_track_err=99.0, cross_track_tolerance=0.0) == ARRIVED_TARGET


def test_centering_never_starts_before_the_distance_is_covered():
    assert _centering(along_track=1.5, cross_track_err=0.30) == DRIVING


def test_obstacle_halt_outranks_centering():
    # Stopped by an obstacle mid-centering is an obstacle stop, not arrival.
    assert _centering(controller_state=HALT, cross_track_err=0.30) == ARRIVED_OBSTACLE


def test_stale_odom_outranks_centering():
    # Cross-track is odom-derived too, so a stale estimate must not be
    # crabbed against.
    assert _centering(cross_track_err=0.30, odom_stale=True) == STOPPED_ODOM_FAULT


def test_latched_arrival_outranks_centering():
    assert _centering(cross_track_err=0.30, already_arrived=True) == ARRIVED_TARGET


# --- controller-skip predicate ---

def test_skip_controller_when_arrived():
    assert should_skip_controller(target_distance=2.0, odom_stale=False,
                                  already_arrived=True) is True


def test_skip_controller_when_stale_and_target_set():
    assert should_skip_controller(target_distance=2.0, odom_stale=True,
                                  already_arrived=False) is True


def test_no_skip_when_stale_but_target_disabled():
    assert should_skip_controller(target_distance=0.0, odom_stale=True,
                                  already_arrived=False) is False


def test_no_skip_when_driving_normally():
    assert should_skip_controller(target_distance=2.0, odom_stale=False,
                                  already_arrived=False) is False
