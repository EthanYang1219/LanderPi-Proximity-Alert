"""Tests for the series gate that makes a stop actually stop the robot.

WHY THIS EXISTS
---------------
Task 9 Step 3 measured a real failure: with `stop_action_node` publishing zeros
*alongside* a live motion source on the same topic, ROS 2's last-write-wins gave
a **51.6% forward duty cycle for 9.5 s after a stop engaged**. The robot kept
being commanded forward while the system reported it was holding a stop.

The fix is topological, not numerical: the motion command must flow THROUGH this
decision instead of competing with it. These tests pin the one property that
makes that true --

    when stop is active, NO forward command can reach the output, ever,
    regardless of how fresh or fast the motion source is.

A test here that passes while the robot still creeps forward would be worse than
no test, so the stop-wins cases are written against every input state that could
plausibly race it, not just the happy one.
"""

import pytest

from poc_fusion.lib.cmd_vel_gate import (
    GATE_FORWARD,
    GATE_SILENT,
    GATE_ZERO,
    evaluate_gate,
)


# --- stop always wins -----------------------------------------------------

def test_stop_emits_zero_even_with_a_fresh_motion_command():
    """THE regression test for the Task 9 Step 3 failure.

    A live motion source is exactly the condition that produced the 51.6%
    forward duty cycle. Fresh input must not survive a stop.
    """
    assert evaluate_gate(
        stop=True, has_input=True, input_age_s=0.0, input_timeout_s=0.5,
    ) == GATE_ZERO


def test_stop_emits_zero_when_input_is_stale():
    assert evaluate_gate(
        stop=True, has_input=True, input_age_s=99.0, input_timeout_s=0.5,
    ) == GATE_ZERO


def test_stop_emits_zero_when_no_input_was_ever_received():
    assert evaluate_gate(
        stop=True, has_input=False, input_age_s=None, input_timeout_s=0.5,
    ) == GATE_ZERO


@pytest.mark.parametrize('age', [0.0, 0.1, 0.5, 0.51, 1000.0])
def test_stop_emits_zero_for_every_possible_input_age(age):
    """Stop must not be conditional on the motion source's timing in any way."""
    assert evaluate_gate(
        stop=True, has_input=True, input_age_s=age, input_timeout_s=0.5,
    ) == GATE_ZERO


# --- forwarding when clear ------------------------------------------------

def test_fresh_input_is_forwarded_when_not_stopped():
    assert evaluate_gate(
        stop=False, has_input=True, input_age_s=0.05, input_timeout_s=0.5,
    ) == GATE_FORWARD


def test_input_exactly_at_the_timeout_is_still_forwarded():
    """Staleness is age > timeout, not >=. Pinned so the boundary cannot drift
    into dropping a command that arrived exactly on time."""
    assert evaluate_gate(
        stop=False, has_input=True, input_age_s=0.5, input_timeout_s=0.5,
    ) == GATE_FORWARD


# --- silence when there is nothing to forward -----------------------------

def test_no_input_ever_produces_silence_not_a_manufactured_zero():
    """The gate must never invent a command. With no motion source, publishing
    nothing lets motion_watchdog's timeout own the stop -- one mechanism, not
    two competing ones."""
    assert evaluate_gate(
        stop=False, has_input=False, input_age_s=None, input_timeout_s=0.5,
    ) == GATE_SILENT


def test_stale_input_produces_silence():
    assert evaluate_gate(
        stop=False, has_input=True, input_age_s=0.51, input_timeout_s=0.5,
    ) == GATE_SILENT


def test_has_input_false_wins_even_if_an_age_is_still_supplied():
    """Regression found by mutation testing: deleting the `has_input` guard
    changed nothing, because every other test that set has_input=False also set
    input_age_s=None, so the None check answered the case and the guard was
    never exercised.

    A caller that clears has_input while retaining the last age -- an easy
    refactor to make -- would then forward a command from a source that is gone.
    """
    assert evaluate_gate(
        stop=False, has_input=False, input_age_s=0.0, input_timeout_s=0.5,
    ) == GATE_SILENT


# --- the three outcomes are distinct --------------------------------------

def test_the_three_outcomes_are_distinct_values():
    """Regression: if any two of these ever compare equal, a stop could be
    indistinguishable from a forward at the call site."""
    assert len({GATE_ZERO, GATE_FORWARD, GATE_SILENT}) == 3
