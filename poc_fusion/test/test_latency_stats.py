"""Tests for the Task 12 latency recorder's pure logic.

Task 12 Step 1: on each False->True transition of `obstacle_detected`, record
`now - (stamp of the newest depth frame preceding the transition)`.

Three things here are easy to get quietly wrong, and each has tests that fail
if they regress:

1. **What counts as a rising edge.** A first-ever message of True is NOT a
   measured False->True transition -- nothing was observed falling. Counting
   it would silently seed the sample set with an edge whose start time is
   unknown.
2. **Which depth frame is "preceding".** It is the newest frame that had
   ARRIVED before the transition, and the latency is measured from that
   frame's own STAMP. Mixing the two clocks (arrival vs stamp) is the classic
   error and would understate latency by the whole pipeline delay.
3. **How p95 is computed.** At n≈20 the choice between nearest-rank and
   linear interpolation moves the answer materially, so the method is pinned
   by test rather than left to a library default.
"""

import pytest

from poc_fusion.lib.latency_stats import (
    Frame,
    is_rising_edge,
    newest_preceding_frame,
    summarize,
)


# --- rising edge ---------------------------------------------------------

def test_false_to_true_is_a_rising_edge():
    assert is_rising_edge(previous=False, current=True) is True


def test_first_ever_message_true_is_not_a_rising_edge():
    """Regression: nothing was observed falling, so there is no measurable
    transition. Counting it would seed the sample set with an unknown start."""
    assert is_rising_edge(previous=None, current=True) is False


def test_true_to_true_is_not_a_rising_edge():
    assert is_rising_edge(previous=True, current=True) is False


def test_true_to_false_is_not_a_rising_edge():
    assert is_rising_edge(previous=True, current=False) is False


def test_false_to_false_is_not_a_rising_edge():
    assert is_rising_edge(previous=False, current=False) is False


# --- which frame precedes the transition ---------------------------------

def test_picks_the_newest_frame_that_arrived_before_the_transition():
    frames = [
        Frame(stamp=1.00, received=1.05),
        Frame(stamp=1.10, received=1.15),
        Frame(stamp=1.20, received=1.25),
    ]
    assert newest_preceding_frame(frames, 1.18).stamp == 1.10


def test_a_frame_that_arrived_after_the_transition_is_excluded():
    """Regression: the recorder must not measure against a frame the pipeline
    could not possibly have used yet."""
    frames = [
        Frame(stamp=1.00, received=1.05),
        Frame(stamp=1.90, received=1.95),
    ]
    assert newest_preceding_frame(frames, 1.50).stamp == 1.00


def test_frame_arriving_exactly_at_the_transition_counts():
    frames = [Frame(stamp=1.00, received=1.50)]
    assert newest_preceding_frame(frames, 1.50).stamp == 1.00


def test_no_preceding_frame_returns_none_rather_than_guessing():
    """Regression: with no depth frame yet there is no lower bound to report.
    Returning None forces the caller to DROP the edge instead of inventing a
    latency from whatever frame arrives next."""
    frames = [Frame(stamp=2.00, received=2.05)]
    assert newest_preceding_frame(frames, 1.00) is None
    assert newest_preceding_frame([], 1.00) is None


def test_selection_is_by_arrival_not_by_stamp():
    """Regression guard on the two-clock error.

    BOTH frames arrived before the transition, so the eligibility filter
    cannot decide this one -- only the comparator can. Stamps are in the
    OPPOSITE order to arrivals, so picking 'newest' by stamp returns 5.00
    while picking by arrival returns 1.00. Written this way deliberately: an
    earlier version of this test put the late frame outside the window, so
    the filter answered it and a stamp-based comparator passed unnoticed.
    """
    frames = [
        Frame(stamp=5.00, received=1.10),   # arrived FIRST, newest stamp
        Frame(stamp=1.00, received=1.90),   # arrived LAST, oldest stamp
    ]
    assert newest_preceding_frame(frames, 2.00).stamp == 1.00


# --- summary statistics --------------------------------------------------

def test_summarize_reports_n_median_and_p95():
    s = summarize([0.10, 0.20, 0.30])
    assert s.n == 3
    assert s.median == pytest.approx(0.20)


def test_median_of_an_even_sample_averages_the_middle_two():
    s = summarize([0.10, 0.20, 0.30, 0.40])
    assert s.median == pytest.approx(0.25)


def test_p95_uses_nearest_rank_and_is_always_an_observed_value():
    """Pinned deliberately: at n around 20 the interpolation choice moves the
    answer, and a latency figure quoted in a paper should be a value the
    system actually produced, not one synthesized between two samples."""
    samples = [float(i) for i in range(1, 21)]  # 1..20
    s = summarize(samples)
    assert s.p95 == 19.0
    assert s.p95 in samples


def test_p95_rounds_the_rank_up_not_down():
    """The discriminating case for nearest-rank.

    n=3: 0.95*3 = 2.85. Rounding the rank UP gives rank 3 -> 0.30; truncating
    gives rank 2 -> 0.20. n=20 does NOT separate these (both land on 19), so
    the n=20 test above passes under either rule -- this one is what actually
    pins the method.
    """
    assert summarize([0.10, 0.20, 0.30]).p95 == pytest.approx(0.30)
    assert summarize([0.1, 0.2, 0.3, 0.4, 0.5]).p95 == pytest.approx(0.5)


def test_p95_of_a_single_sample_is_that_sample():
    s = summarize([0.42])
    assert s.p95 == pytest.approx(0.42)
    assert s.median == pytest.approx(0.42)


def test_summarize_reports_min_and_max():
    s = summarize([0.30, 0.10, 0.20])
    assert s.min == pytest.approx(0.10)
    assert s.max == pytest.approx(0.30)


def test_summarize_does_not_mutate_the_caller_s_list():
    samples = [0.30, 0.10, 0.20]
    summarize(samples)
    assert samples == [0.30, 0.10, 0.20]


def test_summarize_of_nothing_is_empty_not_an_exception_or_a_zero():
    """Regression: a run that captured no edges must report n=0, never 0.0 s,
    which would read as a perfect latency."""
    s = summarize([])
    assert s.n == 0
    assert s.median is None
    assert s.p95 is None
