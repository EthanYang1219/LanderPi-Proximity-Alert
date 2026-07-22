import math
from proximity_alert.avoidance import AvoidanceController, AvoidanceConfig


def _sectors(front=5.0, fl=5.0, fc=5.0, fr=5.0, left=5.0, right=5.0, rear=5.0):
    return {"front": front, "front_left": fl, "front_center": fc, "front_right": fr,
            "left": left, "right": right, "rear": rear}


def _cfg(**kw):
    c = AvoidanceConfig()
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def test_drives_straight_when_clear():
    c = AvoidanceController(_cfg())
    c.set_goal_heading(0.0)
    out = c.step(_sectors(), obstacle=None, gap_bearing=None, current_yaw=0.0, now=0.0)
    assert out.state == "DRIVE"
    assert out.linear_x > 0.0


def test_confirm_scans_debounce_before_assess():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=2, safety_distance=0.30))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    out1 = c.step(_sectors(front=0.29, fc=0.29), obst, None, 0.0, 0.0)
    assert out1.state == "DRIVE" and out1.linear_x == 0.0
    out2 = c.step(_sectors(front=0.29, fc=0.29), obst, None, 0.0, 0.1)
    assert out2.state in ("STRAFE", "TURN")


def test_assess_chooses_strafe_for_narrow_obstacle_with_clear_side():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    out = c.step(_sectors(front=0.29, fc=0.29, left=1.5), obst, None, 0.0, 0.0)
    assert out.state == "STRAFE"
    assert out.linear_y > 0.0            # strafing left (+)
    assert out.decision.chosen_maneuver == "STRAFE"


def test_assess_chooses_turn_when_side_blocked():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1, strafe_side_clearance_min=0.30))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    out = c.step(_sectors(front=0.29, fc=0.29, left=0.15, right=1.5), obst, None, 0.0, 0.0)
    assert out.state == "TURN"


def test_assess_chooses_turn_when_obstacle_too_wide():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1, max_obstacle_span_deg=50.0))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    out = c.step(_sectors(front=0.29, fc=0.29, left=2.0), obst, None, 0.0, 0.0)
    assert out.state == "TURN"


def test_strafe_completes_to_drive_when_front_clears():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    c.step(_sectors(front=0.29, fc=0.29, left=1.5), obst, None, 0.0, 0.0)  # -> STRAFE
    out = c.step(_sectors(front=5.0), None, None, 0.0, 0.4)               # front now clear
    assert out.state == "DRIVE"


def test_escalates_to_recover_then_halt():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1, max_avoid_attempts=2))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    blocked = _sectors(front=0.29, fc=0.29, left=0.15, right=0.15, rear=0.15)
    t = 0.0
    seen = set()
    out = None
    for _ in range(40):
        out = c.step(blocked, obst, gap_bearing=None, current_yaw=0.0, now=t)
        seen.add(out.state)
        t += 2.0
    assert "RECOVER" in seen
    assert out.state == "HALT"


def test_recover_commits_toward_gap_when_available():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1, max_avoid_attempts=1))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    blocked = _sectors(front=0.29, fc=0.29, left=0.2, right=0.2, rear=2.0)
    t = 0.0
    out = None
    for _ in range(6):
        out = c.step(blocked, obst, gap_bearing=math.radians(40), current_yaw=0.0, now=t)
        t += 2.0
        if out.state == "RECOVER":
            break
    assert out.state == "RECOVER"
    assert out.angular_z != 0.0


def test_blip_during_cooldown_resumes_same_encounter():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1, clear_drive_duration=3.0))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    c.step(_sectors(front=0.29, fc=0.29, left=1.5), obst, None, 0.0, 0.0)  # encounter A
    id_a = c.encounter_id
    c.step(_sectors(front=5.0), None, None, 0.0, 0.4)                       # brief clear
    out = c.step(_sectors(front=0.29, fc=0.29, left=1.5), obst, None, 0.0, 1.0)
    assert out.decision.encounter_id == id_a


def test_detection_after_cooldown_is_new_encounter():
    c = AvoidanceController(_cfg(obstacle_confirm_scans=1, clear_drive_duration=1.0))
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    c.step(_sectors(front=0.29, fc=0.29, left=1.5), obst, None, 0.0, 0.0)  # encounter A
    id_a = c.encounter_id
    for t in (0.4, 1.0, 2.0, 3.5):
        c.step(_sectors(front=5.0), None, None, 0.0, t)
    out = c.step(_sectors(front=0.29, fc=0.29, left=1.5), obst, None, 0.0, 4.0)
    assert out.decision.encounter_id == id_a + 1
