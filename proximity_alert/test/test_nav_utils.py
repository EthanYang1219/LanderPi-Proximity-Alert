from proximity_alert.avoidance import DRIVE, HALT, STRAFE, TURN
from proximity_alert.nav_utils import (
    ARRIVED_OBSTACLE, ARRIVED_TARGET, DRIVING, STOPPED_ODOM_FAULT,
    decide_arrival, distance_from_start, should_skip_controller,
)


def _decide(**kw):
    base = dict(controller_state=DRIVE, target_distance=0.0,
                distance_traveled=0.0, odom_stale=False, already_arrived=False)
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


# --- backward compatibility: target_distance disabled ---

def test_disabled_target_never_arrives_however_far_it_drives():
    assert _decide(target_distance=0.0, distance_traveled=100.0) == DRIVING


def test_disabled_target_ignores_stale_odom():
    # No watchdog existed before this feature; leaving target_distance at
    # its default must not introduce one.
    assert _decide(target_distance=0.0, odom_stale=True) == DRIVING


def test_disabled_target_still_reports_obstacle_halt():
    assert _decide(target_distance=0.0, controller_state=HALT) == ARRIVED_OBSTACLE


# --- target-distance arrival ---

def test_arrives_when_target_reached_while_driving():
    assert _decide(target_distance=2.0, distance_traveled=2.0) == ARRIVED_TARGET


def test_arrives_when_target_overshot():
    assert _decide(target_distance=2.0, distance_traveled=2.4) == ARRIVED_TARGET


def test_drives_when_below_target():
    assert _decide(target_distance=2.0, distance_traveled=1.9) == DRIVING


def test_arrival_deferred_mid_strafe():
    assert _decide(controller_state=STRAFE, target_distance=2.0,
                   distance_traveled=5.0) == DRIVING


def test_arrival_deferred_mid_turn():
    assert _decide(controller_state=TURN, target_distance=2.0,
                   distance_traveled=5.0) == DRIVING


def test_arrival_latch_survives_later_ticks():
    assert _decide(already_arrived=True, target_distance=2.0,
                   distance_traveled=0.0) == ARRIVED_TARGET


# --- obstacle halt ---

def test_halt_reports_arrived_obstacle_not_target():
    # Controller gave up well before the target was reached.
    assert _decide(controller_state=HALT, target_distance=5.0,
                   distance_traveled=1.0) == ARRIVED_OBSTACLE


def test_target_reached_wins_over_halt_only_from_drive():
    # Reaching the target during a HALT does not count as arriving at B:
    # the robot is stopped by an obstacle, not by the odometer.
    assert _decide(controller_state=HALT, target_distance=2.0,
                   distance_traveled=2.5) == ARRIVED_OBSTACLE


# --- stale-odom watchdog ---

def test_stale_odom_stops_when_target_set():
    assert _decide(target_distance=2.0, odom_stale=True,
                   distance_traveled=0.5) == STOPPED_ODOM_FAULT


def test_stale_odom_wins_over_untrustworthy_distance():
    # distance_traveled is derived from odom, so stale odom means the
    # distance is stale too and must not be trusted to declare arrival.
    assert _decide(target_distance=2.0, odom_stale=True,
                   distance_traveled=99.0) == STOPPED_ODOM_FAULT


def test_latched_arrival_outranks_stale_odom():
    assert _decide(already_arrived=True, target_distance=2.0,
                   odom_stale=True) == ARRIVED_TARGET


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
