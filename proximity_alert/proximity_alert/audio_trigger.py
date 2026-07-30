"""Pure, ROS-free trigger logic for the obstacle_audio node.

Decides whether a newly-received /avoidance_decision record should play the
alert WAV: once per obstacle encounter (a new encounter_id in a non-DRIVE
state), never on every escalation step within the same encounter, and never
on DRIVE (including the "cleared" record emitted when an encounter ends).
See docs/superpowers/specs/2026-07-24-obstacle-audio-alert-design.md.
"""
from proximity_alert.decision_record import DecisionRecord


def should_play(decision: DecisionRecord, last_played_encounter_id):
    if decision.state == "DRIVE":
        return False
    return decision.encounter_id != last_played_encounter_id
