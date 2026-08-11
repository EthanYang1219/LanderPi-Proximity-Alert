"""Reduce a stopping-distance run to a number. No ROS, no hardware.

Input is a list of `(stamp_s, front_range_m)` samples spanning an approach at
constant speed, a stop command at `t_cmd`, and a settled period afterwards.

THE PRIMARY METRIC
------------------
`trigger_to_rest_m`: the range at the triggering scan minus the settled range.
Two range readings from the same sensor, differenced.

`fit_approach` / `analyse_run` implement an alternative that fits the approach
and extrapolates to the stop-command clock time. That is retained as a
CROSS-CHECK only. It looks more careful and is worse here, because the LD19
samples its forward beam at an unknown but constant phase within each ~0.1 s
rotation: an offset invisible in a fitted slope and fully present in anything
that compares a range against a clock. See `trigger_to_rest_m` for the measured
consequence on run 1.

Validity is `speed_is_steady` over `block_speeds`, not agreement with the
commanded speed -- see those functions for why the distinction is load-bearing.
"""
import statistics


def fit_approach(samples, t_cmd, window_s):
    """Least-squares `r(t) = intercept + slope*t` over `[t_cmd-window, t_cmd]`.

    Returns `(slope, intercept, n, residual_sd)`. Raises ValueError when the
    window holds too few distinct points to define a line -- an empty result
    is a real outcome of a bad run and must not be papered over with a
    fabricated fit.
    """
    seg = [(t, r) for t, r in samples if t_cmd - window_s <= t <= t_cmd]
    n = len(seg)
    if n < 4:
        raise ValueError(
            f'only {n} samples in the {window_s} s fit window; need 4')

    mean_t = sum(t for t, _ in seg) / n
    mean_r = sum(r for _, r in seg) / n
    sxx = sum((t - mean_t) ** 2 for t, _ in seg)
    if sxx == 0.0:
        raise ValueError('degenerate fit window: all samples share one stamp')

    slope = sum((t - mean_t) * (r - mean_r) for t, r in seg) / sxx
    intercept = mean_r - slope * mean_t
    resid = [r - (slope * t + intercept) for t, r in seg]
    return slope, intercept, n, statistics.pstdev(resid)


def settled_range(samples, t_cmd, skip_s, window_s):
    """Median range over `[t_cmd+skip, t_cmd+skip+window]`.

    `skip_s` exists so the robot's own deceleration transient is excluded;
    median rather than mean so one dropped or spurious return cannot drag the
    endpoint. Returns `(median, n, sd)`.
    """
    seg = [r for t, r in samples
           if t_cmd + skip_s <= t <= t_cmd + skip_s + window_s]
    if len(seg) < 4:
        raise ValueError(
            f'only {len(seg)} samples in the settle window; need 4')
    return statistics.median(seg), len(seg), statistics.pstdev(seg)


def approach_deadline_s(start_range_m, trigger_range_m, speed_mps,
                        slack_factor, slack_s):
    """How long the approach may take before it is treated as a fault.

    Derived from the distance actually to be covered rather than fixed, so
    the runaway window scales with the run instead of being generously long
    for every run. `slack_factor` covers the acceleration ramp and any
    speed shortfall; `slack_s` covers fixed startup latency.
    """
    travel = start_range_m - trigger_range_m
    if travel <= 0.0:
        raise ValueError(
            f'start range {start_range_m} m is not beyond the trigger range '
            f'{trigger_range_m} m; there is no approach to make')
    return (travel / speed_mps) * slack_factor + slack_s


def is_closing(start_range_m, current_range_m, min_decrease_m):
    """Is the robot actually getting CLOSER to the wall?

    This is the direction check, and it is deliberately a runtime check
    rather than a pre-run assumption. The forward cone being stable proves
    the beam sees a flat surface; it does NOT prove the beam points along
    the robot's +x. If the scan-angle convention were reversed, or the LiDAR
    were mounted rotated, the robot would drive AWAY from the measured wall
    into space nothing has measured.

    Rather than trust the URDF, the TF rotation and the driver to agree, the
    run asserts the consequence: range must fall. A test that verifies its
    own premise beats one that documents it.
    """
    return (start_range_m - current_range_m) >= min_decrease_m


def trigger_to_rest_m(trigger_scan_range_m, settled_range_m):
    """THE primary measurement: range at the triggering scan, minus rest.

    WHY THIS RATHER THAN THE FIT
    ----------------------------
    The LD19 assembles a full rotation over ~0.1 s, so the forward beam is
    sampled at some phase within each scan. That phase is unknown but
    CONSTANT, which makes it invisible in the fitted slope and fully visible
    in any quantity that compares a range against a CLOCK time. Extrapolating
    the fit to the stop-command instant is exactly such a quantity: a constant
    stamp offset there biases the answer by up to v * 0.1 s = 18 mm at
    0.18 m/s, on a quantity of order 50 mm. Measured on run 1, the fit-based
    estimate moved between 44.6 and 57.2 mm purely with the assumed t_cmd --
    i.e. it straddled the decision threshold on a timestamp convention that
    was never verified.

    Differencing two RANGE readings from the same sensor cancels that offset
    identically, whatever it is.

    It is also the operationally meaningful number. The safety question is
    "the monitor sees an obstacle at range X; where does the robot come to
    rest?", and the monitor sees range readings, not clock times.

    LOWER BOUND, NOT AN ESTIMATE
    ----------------------------
    The measured value understates the deployed system. Here the stop command
    followed the triggering scan by one control-loop iteration; in the
    deployed stack the costmap update, the monitor tick and the gate all sit
    in between. So treat the result as a floor on the real stopping distance,
    never as a central value to size a margin around.
    """
    return trigger_scan_range_m - settled_range_m


def block_speeds(samples, t_cmd, span_s, block_s):
    """Closing speed within each block of the approach, endpoint-to-endpoint.

    Deliberately per-block rather than one global fit: a global fit reports a
    single average and hides an acceleration ramp inside it. Steadiness is a
    statement about the SPREAD across blocks, so the blocks have to exist.
    """
    speeds = []
    edge = t_cmd - span_s
    while edge < t_cmd - 1e-9:
        seg = [(t, r) for t, r in samples if edge <= t <= edge + block_s]
        edge += block_s
        if len(seg) < 2:
            continue
        dt = seg[-1][0] - seg[0][0]
        if dt <= 0.0:
            continue
        speeds.append((seg[0][1] - seg[-1][1]) / dt)   # +ve = closing
    return speeds


def speed_is_steady(block_speeds_mps, max_spread_mps):
    """Was the robot at a CONSTANT speed through the approach?

    Deliberately NOT a comparison against the commanded speed. Run 1 showed
    why: odom twist reported a rock-steady 0.2001 m/s while the LiDAR ground
    truth was 0.1819 m/s. The robot was perfectly steady; the wheels were
    slipping ~10%. Voiding that run for "not at the commanded speed" would
    discard a good measurement and hide the slip.

    Steadiness is what the measurement actually requires -- a stopping
    distance is only meaningful from a constant initial speed. Whether that
    speed equals the commanded one is a separate finding, reported separately.
    """
    if len(block_speeds_mps) < 2:
        raise ValueError('need at least 2 blocks to judge steadiness')
    return (max(block_speeds_mps) - min(block_speeds_mps)) <= max_spread_mps


def analyse_run(samples, t_cmd, fit_window_s, settle_skip_s, settle_window_s):
    """Full reduction. Returns a dict; raises ValueError on an unusable run.

    `approach_speed_mps` is reported as positive-toward-the-wall, i.e. the
    negated slope, because range DECREASES as the robot advances.
    """
    slope, intercept, n_fit, resid_sd = fit_approach(
        samples, t_cmd, fit_window_s)
    settled, n_settle, settle_sd = settled_range(
        samples, t_cmd, settle_skip_s, settle_window_s)

    range_at_cmd = slope * t_cmd + intercept
    return {
        'approach_speed_mps': -slope,
        'fit_points': n_fit,
        'fit_residual_sd_m': resid_sd,
        'range_at_cmd_m': range_at_cmd,
        'settled_range_m': settled,
        'settle_points': n_settle,
        'settle_sd_m': settle_sd,
        'stopping_distance_m': range_at_cmd - settled,
    }
