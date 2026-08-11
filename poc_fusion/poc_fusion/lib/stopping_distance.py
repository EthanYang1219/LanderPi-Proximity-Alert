"""Reduce a stopping-distance run to a number. No ROS, no hardware.

Input is a list of `(stamp_s, front_range_m)` samples spanning an approach at
constant speed, a stop command at `t_cmd`, and a settled period afterwards.

WHY NOT JUST SUBTRACT TWO READINGS
----------------------------------
The obvious method -- range when zero was commanded, minus range once stopped
-- carries a bias equal to the age of the newest scan at the command instant.
At 0.20 m/s and a ~10 Hz LiDAR that is up to 0.02 m, which is the same order
as the stopping distance itself. The bias would be comparable to the result.

So the approach segment is fit as a straight line using each sample's OWN
timestamp and evaluated AT `t_cmd`. Scan age cancels because it is carried in
the timestamps rather than ignored.

The fitted speed is not incidental -- it is the run's validity check. If the
robot was not actually holding the commanded speed when zero was sent, the
"stopping distance" is measuring something else, and `speed_is_valid` says so
rather than letting a plausible-looking number through.
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


def speed_is_valid(fitted_speed_mps, commanded_speed_mps, tolerance_mps):
    """Was the robot actually at the commanded speed when zero was sent?

    If not, the measured decrease in range is not a stopping distance at that
    speed, and the run must be discarded rather than reported.
    """
    return abs(fitted_speed_mps - commanded_speed_mps) <= tolerance_mps


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
