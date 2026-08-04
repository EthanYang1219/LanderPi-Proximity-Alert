from proximity_alert.motion_watchdog_logic import ZERO, decide_watchdog_output


def test_zero_before_anything_ever_received():
    # A watchdog that hasn't heard from its input yet must default to safe,
    # not forward whatever motion happened to be latched before it started.
    assert decide_watchdog_output(None, None, 0.5) == ZERO


def test_zero_before_anything_ever_received_ignores_age():
    # age_s is meaningless when last_cmd is None; must not raise or matter.
    assert decide_watchdog_output(None, 999.0, 0.5) == ZERO


def test_forwards_fresh_command_unchanged():
    cmd = (0.2, -0.05, 0.1)
    assert decide_watchdog_output(cmd, 0.0, 0.5) == cmd


def test_forwards_command_right_at_the_timeout_boundary():
    cmd = (0.2, 0.0, 0.0)
    assert decide_watchdog_output(cmd, 0.5, 0.5) == cmd


def test_forces_zero_once_input_goes_stale():
    cmd = (0.2, 0.0, 0.0)
    assert decide_watchdog_output(cmd, 0.501, 0.5) == ZERO


def test_forces_zero_however_stale_the_input_is():
    # This is the exact failure mode this node exists for: the source died
    # or was orphaned minutes ago and will never publish again.
    cmd = (0.2, -0.75, 0.3)
    assert decide_watchdog_output(cmd, 600.0, 0.5) == ZERO


def test_a_legitimately_commanded_zero_still_passes_through_as_zero():
    # last_cmd == ZERO is not the same case as last_cmd is None -- both
    # produce ZERO output, but for different reasons, and neither should
    # raise or behave differently from a nonzero command at the same age.
    assert decide_watchdog_output(ZERO, 0.1, 0.5) == ZERO


def test_missing_age_with_a_real_last_cmd_still_forces_zero():
    # age_s is only ever None when last_cmd is also None, by construction of
    # the caller -- but the function must not misbehave (e.g. crash on a
    # None comparison) if that invariant is ever violated.
    assert decide_watchdog_output((0.2, 0.0, 0.0), None, 0.5) == ZERO
