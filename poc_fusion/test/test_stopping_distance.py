"""The reduction must recover a KNOWN stopping distance from synthetic data.

The whole reason this module exists rather than a two-reading subtraction is
scan-age bias, so the central test constructs samples whose newest point is
deliberately stale at `t_cmd` and checks the answer is still exact.
"""
import pytest

from poc_fusion.lib.stopping_distance import (
    analyse_run,
    fit_approach,
    settled_range,
    speed_is_valid,
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


def test_speed_validity_rejects_a_run_that_was_not_at_speed():
    assert speed_is_valid(0.200, 0.20, 0.03)
    assert speed_is_valid(0.225, 0.20, 0.03)
    assert not speed_is_valid(0.120, 0.20, 0.03)


def test_settled_range_returns_its_sample_count_and_spread():
    median, n, sd = settled_range(_run(0.036), T_CMD, 1.0, 1.5)
    assert median == pytest.approx(0.964, abs=1e-9)
    assert n >= 4
    assert sd == pytest.approx(0.0, abs=1e-9)
