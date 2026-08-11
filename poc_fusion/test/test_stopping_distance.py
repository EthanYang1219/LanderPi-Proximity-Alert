"""The reduction must recover a KNOWN stopping distance from synthetic data.

The whole reason this module exists rather than a two-reading subtraction is
scan-age bias, so the central test constructs samples whose newest point is
deliberately stale at `t_cmd` and checks the answer is still exact.
"""
import pytest

from poc_fusion.lib.stopping_distance import (
    analyse_run,
    approach_deadline_s,
    block_speeds,
    fit_approach,
    is_closing,
    settled_range,
    speed_is_steady,
    trigger_to_rest_m,
)

SCAN_DT = 0.1          # 10 Hz, matching the LD19
SPEED = 0.20
T_CMD = 1000.0


def _run(stopping_distance_m, scan_phase_s=0.0, speed=SPEED):
    """Synthetic run with an exactly known answer.

    `scan_phase_s` shifts the sample grid so the newest approach sample is
    that stale at `t_cmd`. The recovered distance must not depend on it.
    """
    samples = []
    # Approach: range decreases at `speed`, sampled on a grid offset by phase.
    t = T_CMD - 2.0 + scan_phase_s
    while t <= T_CMD - 1e-9:
        samples.append((t, 1.0 + speed * (T_CMD - t)))
        t += SCAN_DT
    # Settled: range at the command instant was 1.0, so it ends up
    # 1.0 - stopping_distance.
    t = T_CMD + 0.5
    while t <= T_CMD + 4.0:
        samples.append((t, 1.0 - stopping_distance_m))
        t += SCAN_DT
    return samples


def _analyse(samples):
    return analyse_run(samples, T_CMD, fit_window_s=1.0,
                       settle_skip_s=1.0, settle_window_s=1.5)


def test_recovers_a_known_stopping_distance():
    result = _analyse(_run(0.036))
    assert result['stopping_distance_m'] == pytest.approx(0.036, abs=1e-9)


def test_result_is_independent_of_scan_age_at_the_command_instant():
    """THE test. A two-reading subtraction would be biased by up to
    speed * SCAN_DT = 0.02 m here -- half the quantity being measured."""
    answers = [_analyse(_run(0.036, scan_phase_s=p))['stopping_distance_m']
               for p in (0.0, 0.025, 0.05, 0.075, 0.099)]
    assert max(answers) - min(answers) < 1e-9
    # And confirm the bias being avoided is genuinely large enough to matter.
    assert SPEED * SCAN_DT > 0.036 / 2


def test_reports_the_approach_speed_as_positive_toward_the_wall():
    result = _analyse(_run(0.036))
    assert result['approach_speed_mps'] == pytest.approx(SPEED, abs=1e-9)


def test_zero_stopping_distance_is_reported_as_zero():
    """An instantaneous stop must not acquire a spurious offset from the
    fit-and-extrapolate machinery."""
    result = _analyse(_run(0.0))
    assert result['stopping_distance_m'] == pytest.approx(0.0, abs=1e-9)


def test_fit_reports_residual_spread_so_noise_is_visible():
    samples = _run(0.036)
    noisy = [(t, r + (0.004 if i % 2 else -0.004))
             for i, (t, r) in enumerate(samples)]
    _, _, _, clean_sd = fit_approach(_run(0.036), T_CMD, 1.0)
    _, _, _, noisy_sd = fit_approach(noisy, T_CMD, 1.0)
    assert clean_sd < 1e-9
    assert noisy_sd > 0.003


def test_too_few_approach_samples_raises_rather_than_guessing():
    sparse = [(T_CMD - 0.9, 1.18), (T_CMD - 0.3, 1.06)]
    sparse += [(T_CMD + 1.2, 0.964)] * 8
    with pytest.raises(ValueError, match='fit window'):
        _analyse(sparse)


def test_too_few_settle_samples_raises_rather_than_guessing():
    samples = [(t, r) for t, r in _run(0.036) if t <= T_CMD + 1.2]
    with pytest.raises(ValueError, match='settle window'):
        _analyse(samples)


def test_degenerate_fit_window_raises():
    samples = [(T_CMD, 1.0)] * 6 + [(T_CMD + 1.5, 0.964)] * 6
    with pytest.raises(ValueError, match='degenerate'):
        _analyse(samples)


def test_settle_window_uses_median_so_one_dropout_cannot_move_it():
    samples = _run(0.036)
    corrupted = []
    seen = 0
    for t, r in samples:
        if t > T_CMD + 1.0 and seen == 0:
            seen = 1
            corrupted.append((t, 3.5))     # one absurd return
            continue
        corrupted.append((t, r))
    assert (_analyse(corrupted)['stopping_distance_m']
            == pytest.approx(0.036, abs=1e-9))


def test_steadiness_accepts_a_steady_run_that_missed_the_commanded_speed():
    """Run 1: odom said a rock-steady 0.2001 m/s, LiDAR ground truth said
    0.1819 m/s. The robot was steady; the wheels were slipping ~10%. That is
    a usable measurement plus a separate finding, not a void run."""
    slipping_but_steady = [0.180, 0.184, 0.181, 0.183, 0.182]
    assert speed_is_steady(slipping_but_steady, max_spread_mps=0.03)


def test_steadiness_rejects_a_run_still_accelerating():
    ramping = [0.05, 0.11, 0.16, 0.19, 0.20]
    assert not speed_is_steady(ramping, max_spread_mps=0.03)


def test_steadiness_needs_more_than_one_block():
    with pytest.raises(ValueError, match='at least 2'):
        speed_is_steady([0.18], max_spread_mps=0.03)


def test_trigger_to_rest_is_immune_to_the_scan_phase_offset():
    """The primary metric. A constant per-scan timestamp offset shifts every
    fit-extrapolated answer but cannot touch a difference of two ranges."""
    # Run 1's actual numbers: triggering scan 0.7940, settled 0.7350.
    assert trigger_to_rest_m(0.7940, 0.7350) == pytest.approx(0.0590, abs=1e-9)
    # Whatever the phase offset is, both readings carry it identically.
    for offset_m in (0.0, 0.009, 0.018):
        assert (trigger_to_rest_m(0.7940 - offset_m, 0.7350 - offset_m)
                == pytest.approx(0.0590, abs=1e-9))


def test_closing_check_catches_a_robot_driving_the_wrong_way():
    """The direction guard. If the scan-angle convention were reversed the
    robot would drive away from the wall; range would RISE, not fall."""
    # Driving toward the wall: range falls. Closing.
    assert is_closing(start_range_m=1.82, current_range_m=1.60,
                      min_decrease_m=0.10)
    # Driving away: range rises. Not closing -> abort.
    assert not is_closing(start_range_m=1.82, current_range_m=2.10,
                          min_decrease_m=0.10)
    # Stationary (wheels slipping, motors not engaging): not closing.
    assert not is_closing(start_range_m=1.82, current_range_m=1.82,
                          min_decrease_m=0.10)
    # Moving, but not yet far enough to be conclusive.
    assert not is_closing(start_range_m=1.82, current_range_m=1.77,
                          min_decrease_m=0.10)


def test_approach_deadline_scales_with_the_distance_to_cover():
    near = approach_deadline_s(1.0, 0.8, SPEED, slack_factor=2.5, slack_s=2.0)
    far = approach_deadline_s(3.0, 0.8, SPEED, slack_factor=2.5, slack_s=2.0)
    assert far > near
    # 1.822 m start, 0.80 m trigger => 1.022 m at 0.20 m/s = 5.11 s nominal.
    actual = approach_deadline_s(1.822, 0.8, SPEED, 2.5, 2.0)
    assert actual == pytest.approx(5.11 * 2.5 + 2.0, abs=1e-6)
    # It must be a real bound, not a formality: well under the old flat 20 s.
    assert actual < 20.0


def test_approach_deadline_refuses_a_start_inside_the_trigger():
    """Placing the robot already closer than the trigger is a setup error,
    not a zero-length approach to be waved through."""
    with pytest.raises(ValueError, match='not beyond the trigger'):
        approach_deadline_s(0.5, 0.8, SPEED, 2.5, 2.0)


def test_block_speeds_recover_a_constant_approach_speed():
    speeds = block_speeds(_run(0.036), T_CMD, span_s=2.0, block_s=0.5)
    assert len(speeds) == 4
    for s in speeds:
        assert s == pytest.approx(SPEED, abs=1e-9)


def test_block_speeds_expose_a_ramp_that_a_single_fit_would_average_away():
    """The reason blocks exist. A robot still accelerating has the right
    MEAN speed and the wrong steadiness; one global fit reports only the
    mean and would wave the run through."""
    samples = []
    t, r = T_CMD - 2.0, 1.4
    while t <= T_CMD + 1e-9:
        samples.append((t, r))
        v = 0.05 + 0.075 * (t - (T_CMD - 2.0))   # ramping 0.05 -> 0.20
        r -= v * SCAN_DT
        t += SCAN_DT
    speeds = block_speeds(samples, T_CMD, span_s=2.0, block_s=0.5)
    assert not speed_is_steady(speeds, max_spread_mps=0.03)
    # The mean over the whole approach is ~0.125 m/s and perfectly plausible.
    assert 0.10 < sum(speeds) / len(speeds) < 0.15


def test_block_speeds_are_positive_toward_the_wall():
    """Sign convention: closing is positive, matching approach_speed_mps."""
    receding = [(T_CMD - 2.0 + i * SCAN_DT, 1.0 + SPEED * i * SCAN_DT)
                for i in range(21)]
    assert all(s < 0 for s in block_speeds(receding, T_CMD, 2.0, 0.5))


def test_block_speeds_skip_blocks_too_sparse_to_measure():
    """A dropout must drop that block, not fabricate a speed from one point."""
    samples = [(t, r) for t, r in _run(0.036)
               if not (T_CMD - 1.5 < t < T_CMD - 1.05)]
    speeds = block_speeds(samples, T_CMD, span_s=2.0, block_s=0.5)
    assert len(speeds) == 3
    for s in speeds:
        assert s == pytest.approx(SPEED, abs=1e-9)


def test_settled_range_returns_its_sample_count_and_spread():
    median, n, sd = settled_range(_run(0.036), T_CMD, 1.0, 1.5)
    assert median == pytest.approx(0.964, abs=1e-9)
    assert n >= 4
    assert sd == pytest.approx(0.0, abs=1e-9)
