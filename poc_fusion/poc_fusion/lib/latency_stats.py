"""Pure logic for the Task 12 latency recorder.

No ROS: stamps and arrival times are plain floats (seconds) so every rule is
unit-testable off-robot.

WHAT IS BEING MEASURED
----------------------
On each False->True transition of `obstacle_detected`, the recorded latency is

    now  -  (stamp of the newest depth frame that had ARRIVED before the
             transition)

Two different clocks appear in that sentence and conflating them is the
classic error here:

  * `Frame.received` -- when this process got the frame. Used ONLY to decide
    which frame the pipeline could plausibly have acted on.
  * `Frame.stamp`    -- the camera's own timestamp, carried verbatim through
    `/poc_fusion/depth_cleaned` (Task 4 Step 1). Used as the measurement
    ORIGIN.

Selecting by stamp instead of by arrival would let a frame that has not
reached the pipeline yet win, understating latency by roughly the whole
pipeline delay.

WHAT THIS FIGURE IS NOT
-----------------------
It is a LOWER BOUND on true physical-entry-to-stop latency, and Task 12
Step 3 requires it to be reported as such. It covers preprocessing,
projection, costmap update, the publish interval and monitor evaluation --
everything this POC adds. It does NOT cover the camera's own exposure and
internal processing latency, which is unrecoverable from message stamps and
would need external high-frame-rate video to capture. Do not describe the
result as end-to-end.
"""

from collections import namedtuple

Frame = namedtuple('Frame', ['stamp', 'received'])

Summary = namedtuple('Summary', ['n', 'median', 'p95', 'min', 'max'])


def is_rising_edge(previous, current):
    """Whether this reading is a MEASURED False->True transition.

    `previous` is None before any message has been seen. A first-ever True is
    deliberately NOT an edge: nothing was observed falling, so the transition
    instant is unknown and including it would seed the sample set with a
    latency measured from an arbitrary start.
    """
    return previous is False and current is True


def newest_preceding_frame(frames, transition_time_s):
    """The newest frame that had ARRIVED at or before `transition_time_s`.

    Returns None when no frame qualifies. None is meaningful: the caller must
    DROP that edge rather than fall back to whatever frame arrives next,
    which would fabricate a latency the system never produced.

    Selection is by `received`, never by `stamp` -- see the module docstring.
    Ties on `received` resolve to the last such frame in `frames`, which is
    arrival order.
    """
    best = None
    for frame in frames:
        if frame.received <= transition_time_s:
            if best is None or frame.received >= best.received:
                best = frame
    return best


def summarize(samples):
    """n, median, p95, min and max over a latency sample set.

    An empty sample set reports n=0 with None statistics rather than 0.0 --
    a run that captured nothing must never render as a perfect latency.

    p95 is NEAREST-RANK (the smallest observed value at or above the 95th
    percentile position), not linearly interpolated. Two reasons, both
    deliberate: at the n≈20 Task 12 Step 2 calls for, the interpolation
    choice moves the answer materially, and a latency quoted in a write-up
    should be a value the system actually produced rather than one
    synthesized between two samples.

    The caller's list is not mutated.
    """
    ordered = sorted(samples)
    n = len(ordered)
    if n == 0:
        return Summary(n=0, median=None, p95=None, min=None, max=None)

    mid = n // 2
    if n % 2 == 1:
        median = ordered[mid]
    else:
        median = (ordered[mid - 1] + ordered[mid]) / 2.0

    # Nearest-rank: ceil(0.95 * n) as a 1-based rank, clamped into range.
    rank = -(-95 * n // 100)  # ceil(0.95 * n) using integer arithmetic
    rank = max(1, min(rank, n))

    return Summary(n=n, median=median, p95=ordered[rank - 1],
                   min=ordered[0], max=ordered[-1])
