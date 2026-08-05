import math
from proximity_alert.avoidance import AvoidanceController, AvoidanceConfig, _distance


def _sectors(front=5.0, fl=5.0, fc=5.0, fr=5.0, left=5.0, right=5.0, rear=5.0):
    # Defaults are all "wide open" (5m) so a test only needs to override the
    # sector(s) it actually cares about.
    return {"front": front, "front_left": fl, "front_center": fc, "front_right": fr,
            "left": left, "right": right, "rear": rear}


def _cfg(**kw):
    c = AvoidanceConfig()
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def _blocked_front(cfg):
    # _drive() triggers on front <= safety_distance, so this is always
    # "just close enough to block" regardless of what safety_distance is
    # currently configured to -- avoids hardcoding an absolute distance that
    # silently stops being blocking if the default changes.
    return cfg.safety_distance


def test_drives_straight_when_clear():
    c = AvoidanceController(_cfg())
    c.set_goal_heading(0.0)
    out = c.step(_sectors(), obstacle=None, gap_bearing=None, current_yaw=0.0,
                 current_pos=(0.0, 0.0), now=0.0)
    assert out.state == "DRIVE"
    assert out.linear_x > 0.0


def test_confirm_scans_debounce_before_assess():
    # First blocked scan should hold (debounce), second should commit to a maneuver.
    cfg = _cfg(obstacle_confirm_scans=2)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    out1 = c.step(_sectors(front=front, fc=front), obst, None, 0.0, (0.0, 0.0), 0.0)
    assert out1.state == "DRIVE" and out1.linear_x == 0.0
    out2 = c.step(_sectors(front=front, fc=front), obst, None, 0.0, (0.0, 0.0), 0.1)
    assert out2.state in ("STRAFE", "TURN")


def test_assess_chooses_strafe_for_narrow_obstacle_with_clear_side():
    # strafe_speed/strafe_timeout pinned explicitly: at today's live defaults
    # max_strafe_distance (0.75) already exceeds max_cumulative_strafe (0.60),
    # which makes strafe_ok's cap check fail unconditionally -- STRAFE could
    # never be selected regardless of obstacle geometry. Pin known-consistent
    # values here so this test verifies the LADDER LOGIC, not today's tuning.
    cfg = _cfg(obstacle_confirm_scans=1, strafe_speed=0.25, strafe_timeout=1.5)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    out = c.step(_sectors(front=front, fc=front, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.0)
    assert out.state == "STRAFE"
    assert out.linear_y > 0.0            # strafing left (+)
    assert out.decision.chosen_maneuver == "STRAFE"


def test_assess_chooses_turn_when_side_blocked():
    cfg = _cfg(obstacle_confirm_scans=1, strafe_side_clearance_min=0.30)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    out = c.step(_sectors(front=front, fc=front, left=0.15, right=1.5), obst, None, 0.0, (0.0, 0.0), 0.0)
    assert out.state == "TURN"


def test_assess_chooses_turn_when_obstacle_too_wide():
    # Physically wide obstacle (1.2 m across) -> can't strafe past it, must turn.
    cfg = _cfg(obstacle_confirm_scans=1, max_obstacle_width=0.50)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    out = c.step(_sectors(front=front, fc=front, left=2.0), obst, None, 0.0, (0.0, 0.0), 0.0)
    assert out.state == "TURN"


def test_assess_strafes_past_close_narrow_box_despite_wide_angular_span():
    # A physically narrow box (30 cm wide) seen up close subtends a LARGE
    # angular span (~65 deg) simply because it is close. The strafe gate must
    # key on physical width (y_hi - y_lo), not angular span -- otherwise every
    # real obstacle at trigger range looks "too wide" and strafing is impossible.
    # Regression for the all-TURN behavior observed on hardware 2026-07-22.
    # strafe_speed/strafe_timeout pinned -- see comment in
    # test_assess_chooses_strafe_for_narrow_obstacle_with_clear_side.
    cfg = _cfg(obstacle_confirm_scans=1, max_obstacle_width=0.50,
               strafe_speed=0.25, strafe_timeout=1.5)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 65.0, "y_lo": -0.15, "y_hi": 0.15, "preferred_side": -1.0}
    front = _blocked_front(cfg)
    out = c.step(_sectors(front=front, fc=front, fr=front, right=1.5), obst, None, 0.0, (0.0, 0.0), 0.0)
    assert out.state == "STRAFE"
    assert out.linear_y < 0.0            # strafing right (-)
    assert out.decision.chosen_maneuver == "STRAFE"


def test_strafe_completes_to_drive_when_front_clears():
    # strafe_speed/strafe_timeout pinned -- see comment in
    # test_assess_chooses_strafe_for_narrow_obstacle_with_clear_side.
    cfg = _cfg(obstacle_confirm_scans=1, strafe_speed=0.25, strafe_timeout=1.5)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    c.step(_sectors(front=front, fc=front, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.0)  # -> STRAFE
    out = c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 0.4)                  # front now clear
    assert out.state == "DRIVE"


def test_escalates_to_recover_then_halt():
    # Boxed in on every side -- STRAFE/TURN keep failing until max_avoid_attempts
    # is exceeded, then one RECOVER attempt, then HALT once that also fails.
    cfg = _cfg(obstacle_confirm_scans=1, max_avoid_attempts=2)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    blocked = _sectors(front=front, fc=front, left=0.15, right=0.15, rear=0.15)
    t = 0.0
    seen = set()
    out = None
    for _ in range(40):
        out = c.step(blocked, obst, gap_bearing=None, current_yaw=0.0,
                     current_pos=(0.0, 0.0), now=t)
        seen.add(out.state)
        t += 2.0
    assert "RECOVER" in seen
    assert out.state == "HALT"


def test_recover_commits_toward_gap_when_available():
    # With a gap bearing available, RECOVER should commit toward it (nonzero turn)
    # instead of halting outright.
    cfg = _cfg(obstacle_confirm_scans=1, max_avoid_attempts=1)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    blocked = _sectors(front=front, fc=front, left=0.2, right=0.2, rear=2.0)
    t = 0.0
    out = None
    for _ in range(6):
        out = c.step(blocked, obst, gap_bearing=math.radians(40), current_yaw=0.0,
                     current_pos=(0.0, 0.0), now=t)
        t += 2.0
        if out.state == "RECOVER":
            break
    assert out.state == "RECOVER"
    assert out.angular_z != 0.0


# ---------- reverse-arc geometry (turn_radius / rear_taper_zone) ----------


def _turning(cfg, rear):
    """Drives the controller into TURN with the given rear clearance and
    returns that tick's output. Obstacle is wide (forces TURN, not STRAFE)
    and both sides are blocked so it can't sidestep."""
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 80.0, "y_lo": -0.6, "y_hi": 0.6, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    s = _sectors(front=front, fc=front, left=0.15, right=0.15, rear=rear)
    out = c.step(s, obst, gap_bearing=None, current_yaw=0.0,
                 current_pos=(0.0, 0.0), now=0.0)
    assert out.state == "TURN", f"expected TURN, got {out.state}"
    return out


def test_turn_arcs_backward_while_rotating_by_default():
    # Baseline: TURN commands reverse and rotation on the SAME tick (an arc),
    # not a rotate-in-place pivot. Guards the property the radius math rests on.
    out = _turning(_cfg(obstacle_confirm_scans=1), rear=5.0)
    assert out.linear_x < 0.0
    assert out.angular_z != 0.0


def test_legacy_reverse_speed_used_when_turn_radius_unset():
    cfg = _cfg(obstacle_confirm_scans=1, avoid_reverse_speed=0.10, turn_radius=0.0)
    out = _turning(cfg, rear=5.0)
    assert out.linear_x == -0.10


def test_turn_radius_derives_reverse_speed_from_turn_speed():
    # radius = v / omega, so v = radius * omega. A 0.5m radius at 0.75 rad/s
    # is 0.375 m/s -- far wider than the legacy 0.10/0.75 = ~0.13m arc.
    cfg = _cfg(obstacle_confirm_scans=1, turn_radius=0.5, turn_speed=0.75,
               avoid_reverse_speed=0.10)
    out = _turning(cfg, rear=5.0)
    assert math.isclose(out.linear_x, -0.375, rel_tol=1e-9)
    assert math.isclose(abs(out.linear_x / out.angular_z), 0.5, rel_tol=1e-9)


def test_reverse_still_blocked_by_rear_clearance_regardless_of_radius():
    # A wide radius must never override the rear-clearance safety gate.
    cfg = _cfg(obstacle_confirm_scans=1, turn_radius=0.5, rear_clearance_min=0.25)
    out = _turning(cfg, rear=0.10)
    assert out.linear_x == 0.0
    assert out.angular_z != 0.0          # still rotates, just no longer backing up


def test_taper_scales_reverse_down_near_rear_limit():
    # Halfway into the taper zone -> half the reverse speed, instead of the
    # hard 0/full snap that made the maneuver lurch arc -> pivot.
    cfg = _cfg(obstacle_confirm_scans=1, turn_radius=0.5, turn_speed=0.75,
               rear_clearance_min=0.25, rear_taper_zone=0.20)
    out = _turning(cfg, rear=0.35)       # 0.10 of headroom into a 0.20 zone
    assert math.isclose(out.linear_x, -0.375 * 0.5, rel_tol=1e-9)


def test_taper_does_not_exceed_full_speed_with_open_rear():
    cfg = _cfg(obstacle_confirm_scans=1, turn_radius=0.5, turn_speed=0.75,
               rear_clearance_min=0.25, rear_taper_zone=0.20)
    out = _turning(cfg, rear=float("inf"))
    assert math.isclose(out.linear_x, -0.375, rel_tol=1e-9)


# --- cross-track (return-to-line) crab correction ---

def test_no_crab_unless_asked():
    # Regression guard: a controller nobody calls set_lateral_correction on
    # must behave exactly as it did before the feature existed.
    c = AvoidanceController(_cfg())
    c.set_goal_heading(0.0)
    out = c.step(_sectors(), None, None, 0.0, (0.0, 0.0), 0.0)
    assert out.linear_y == 0.0


def test_crab_applied_while_driving():
    c = AvoidanceController(_cfg(cross_track_ramp_time=0.0))
    c.set_goal_heading(0.0)
    c.set_lateral_correction(-0.10)
    out = c.step(_sectors(), None, None, 0.0, (0.0, 0.0), 0.0)
    assert out.state == "DRIVE"
    assert out.linear_y == -0.10
    assert out.linear_x > 0.0        # still driving forward, not just sliding


def test_crab_does_not_disturb_heading_or_speed():
    # The crab is an independent axis: it must not bleed into forward speed
    # or the heading hold.
    c = AvoidanceController(_cfg(cross_track_ramp_time=0.0))
    c.set_goal_heading(0.0)
    plain = c.step(_sectors(), None, None, 0.0, (0.0, 0.0), 0.0)

    c2 = AvoidanceController(_cfg(cross_track_ramp_time=0.0))
    c2.set_goal_heading(0.0)
    c2.set_lateral_correction(-0.10)
    crabbed = c2.step(_sectors(), None, None, 0.0, (0.0, 0.0), 0.0)

    assert crabbed.linear_x == plain.linear_x
    assert crabbed.angular_z == plain.angular_z


def test_ramp_fades_the_crab_in_over_time():
    cfg = _cfg(cross_track_ramp_time=1.0)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    c.set_lateral_correction(-0.10)
    at_entry = c.step(_sectors(), None, None, 0.0, (0.0, 0.0), 0.0)
    midway = c.step(_sectors(), None, None, 0.0, (0.0, 0.0), 0.5)
    settled = c.step(_sectors(), None, None, 0.0, (0.0, 0.0), 2.0)
    assert at_entry.linear_y == 0.0                    # starts from nothing
    assert midway.linear_y == -0.05                    # half ramped
    assert settled.linear_y == -0.10                   # clamped at full


def test_no_crab_while_confirming_an_obstacle():
    # The confirming tick deliberately holds still; crabbing through it
    # would move the robot while it is deciding whether it is blocked.
    cfg = _cfg(obstacle_confirm_scans=2, cross_track_ramp_time=0.0)
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    c.set_lateral_correction(-0.10)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    out = c.step(_sectors(front=front, fc=front), obst, None, 0.0, (0.0, 0.0), 0.0)
    assert out.state == "DRIVE" and out.linear_x == 0.0
    assert out.linear_y == 0.0


def _strafe_then_clear(cfg, correction, inside_flank):
    """Drive the controller into a leftward STRAFE, then clear the front so
    it completes back to DRIVE, and return that completing output.

    _locked_dir ends up +1 (dodged left), so the obstacle is on the RIGHT
    and a correction back toward the right is the gated direction.
    """
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    out = c.step(_sectors(front=front, fc=front, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.0)
    assert out.state == "STRAFE"
    c.set_lateral_correction(correction)
    # Front now clear -> _strafe completes to DRIVE on this tick.
    return c, c.step(_sectors(left=1.5, right=inside_flank), None, None, 0.0, (0.0, 0.0), 0.5)


def _strafe_cfg(**kw):
    base = dict(obstacle_confirm_scans=1, strafe_speed=0.25, strafe_timeout=1.5,
                cross_track_ramp_time=0.0)
    base.update(kw)
    return _cfg(**base)


def test_crab_blocked_toward_the_obstacle_just_avoided():
    # The limit-cycle guard. A strafe makes no forward progress, so on
    # completion the robot is level with the obstacle -- crabbing back
    # toward it would re-block the front and strafe out again, forever.
    cfg = _strafe_cfg()
    c, out = _strafe_then_clear(cfg, correction=-0.10,
                                inside_flank=cfg.pass_clearance - 0.1)
    assert out.state == "DRIVE"
    assert out.linear_y == 0.0


def test_crab_allowed_away_from_the_obstacle_just_avoided():
    # Moving further from the obstacle can only increase clearance, so it is
    # never gated even with the flank still close.
    cfg = _strafe_cfg()
    c, out = _strafe_then_clear(cfg, correction=+0.10,
                                inside_flank=cfg.pass_clearance - 0.1)
    assert out.linear_y == +0.10


def test_crab_toward_obstacle_allowed_once_the_flank_is_clear():
    cfg = _strafe_cfg()
    c, out = _strafe_then_clear(cfg, correction=-0.10,
                                inside_flank=cfg.pass_clearance + 0.5)
    assert out.linear_y == -0.10


def test_crab_ungated_once_the_encounter_closes():
    # After clear_drive_distance of confirmed-clear driving there is no
    # remembered obstacle left to protect, so the correction runs unrestricted
    # even back toward where the obstacle used to be.
    cfg = _strafe_cfg(clear_drive_distance=0.3, encounter_close_confirm_scans=1)
    c, out = _strafe_then_clear(cfg, correction=-0.10,
                                inside_flank=cfg.pass_clearance - 0.1)
    assert out.linear_y == 0.0                       # gated at first
    flank = _sectors(right=cfg.pass_clearance - 0.1)
    c.step(flank, None, None, 0.0, (0.0, 0.0), 1.0)   # streak=1, start_pos=(0,0)
    for i, x in enumerate((0.1, 0.2)):               # sub-jump-threshold increments
        c.step(flank, None, None, 0.0, (x, 0.0), 1.5 + i * 0.5)
    late = c.step(flank, None, None, 0.0, (0.3, 0.0), 3.0)   # 0.3m >= 0.3m -> closes
    assert late.linear_y == -0.10                    # encounter closed, gate lifted


def test_maneuvers_never_see_the_crab():
    # A correction arriving mid-strafe must not bend the strafe: the
    # maneuver states own linear_y outright.
    cfg = _strafe_cfg()
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    c.step(_sectors(front=front, fc=front, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.0)
    c.set_lateral_correction(-0.10)
    out = c.step(_sectors(front=front, fc=front, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.1)
    assert out.state == "STRAFE"
    assert out.linear_y == +cfg.strafe_speed     # full strafe, uncontaminated


# ---------- distance-based encounter close ----------


def test_distance_helper_straight_line():
    assert _distance((0.0, 0.0), (3.0, 4.0)) == 5.0


def test_config_has_distance_based_encounter_fields():
    cfg = AvoidanceConfig()
    assert cfg.clear_drive_distance == 0.3
    assert cfg.encounter_close_confirm_scans == 3
    assert cfg.odom_jump_threshold == 0.15
    assert not hasattr(cfg, "clear_drive_duration")


def _enter_encounter(c, obst=None):
    """Drives one confirmed obstacle detection so the controller is inside
    an encounter (matches whatever obstacle_confirm_scans the config uses)."""
    obst = obst or {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    out = None
    for i in range(c.config.obstacle_confirm_scans):
        out = c.step(_sectors(front=0.15, fc=0.15, left=1.5), obst, None, 0.0, (0.0, 0.0), float(i))
    return out


def test_streak_below_confirm_scans_does_not_start_distance():
    c = AvoidanceController(_cfg(encounter_close_confirm_scans=3, clear_drive_distance=0.3))
    c.set_goal_heading(0.0)
    _enter_encounter(c)
    # only 2 consecutive clear scans (< encounter_close_confirm_scans=3), far enough
    # displacement-wise that it WOULD close if distance tracking had started
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.0)   # maneuver -> DRIVE
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.1)   # streak=1
    out = c.step(_sectors(front=5.0), None, None, 0.0, (1.0, 0.0), 10.2)  # streak=2
    assert out.decision is None or out.decision.encounter_id == c.encounter_id  # still same encounter, not yet closed
    assert c._in_encounter is True


def test_encounter_closes_once_distance_threshold_reached():
    c = AvoidanceController(_cfg(encounter_close_confirm_scans=2, clear_drive_distance=0.3))
    c.set_goal_heading(0.0)
    _enter_encounter(c)
    # Increments stay under odom_jump_threshold (0.15) so they read as real
    # travel, not a discontinuity -- a single 0.3m hop would be rejected as a
    # jump before it could ever close the encounter.
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.0)  # maneuver -> DRIVE
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.1)  # streak=1
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.2)  # streak=2, start_pos=(0,0)
    c.step(_sectors(front=5.0), None, None, 0.0, (0.1, 0.0), 10.3)  # 0.1m
    c.step(_sectors(front=5.0), None, None, 0.0, (0.2, 0.0), 10.4)  # 0.2m < 0.3m, not yet
    assert c._in_encounter is True
    c.step(_sectors(front=5.0), None, None, 0.0, (0.3, 0.0), 10.5)  # 0.3m >= 0.3m
    assert c._in_encounter is False
    assert c.consecutive_avoid_count == 0
    assert c.cumulative_strafe == 0.0


def test_block_before_distance_threshold_preserves_same_encounter():
    c = AvoidanceController(_cfg(encounter_close_confirm_scans=2, clear_drive_distance=0.3,
                                  obstacle_confirm_scans=1))
    c.set_goal_heading(0.0)
    _enter_encounter(c)
    id_a = c.encounter_id
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.0)  # streak=1
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.1)  # streak=2, start_pos set
    c.step(_sectors(front=5.0), None, None, 0.0, (0.1, 0.0), 10.2)  # 0.1m, not closed yet
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    out = c.step(_sectors(front=0.15, fc=0.15, left=1.5), obst, None, 0.0, (0.15, 0.0), 10.3)  # blocked again
    assert out.decision.encounter_id == id_a  # same encounter, not reset


def test_detection_after_sufficient_clear_distance_is_new_encounter():
    c = AvoidanceController(_cfg(encounter_close_confirm_scans=2, clear_drive_distance=0.3,
                                  obstacle_confirm_scans=1))
    c.set_goal_heading(0.0)
    _enter_encounter(c)
    id_a = c.encounter_id
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.0)  # maneuver -> DRIVE
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.1)  # streak=1
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.2)  # streak=2, start_pos=(0,0)
    for i, x in enumerate((0.1, 0.2, 0.3)):     # sub-jump-threshold increments
        c.step(_sectors(front=5.0), None, None, 0.0, (x, 0.0), 10.3 + i * 0.1)
    assert c._in_encounter is False             # closed: 0.3m >= 0.3m
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    out = c.step(_sectors(front=0.15, fc=0.15, left=1.5), obst, None, 0.0, (0.3, 0.0), 10.7)
    assert out.decision.encounter_id == id_a + 1


def test_interrupted_clear_periods_do_not_sum():
    c = AvoidanceController(_cfg(encounter_close_confirm_scans=1, clear_drive_distance=0.3,
                                  obstacle_confirm_scans=1))
    c.set_goal_heading(0.0)
    _enter_encounter(c)
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.0)   # streak=1, start_pos=(0,0)
    c.step(_sectors(front=5.0), None, None, 0.0, (0.15, 0.0), 10.1)  # 0.15m clear
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    c.step(_sectors(front=0.15, fc=0.15, left=1.5), obst, None, 0.0, (0.15, 0.0), 10.2)  # blocked -> resets
    c.step(_sectors(front=5.0), None, None, 0.0, (0.15, 0.0), 10.3)   # streak=1 again, start_pos=(0.15,0)
    out = c.step(_sectors(front=5.0), None, None, 0.0, (0.30, 0.0), 10.4)  # only 0.15m this stretch
    assert c._in_encounter is True   # 0.15m + 0.15m must NOT have summed to 0.3m


def test_none_position_never_advances_tracking():
    c = AvoidanceController(_cfg(encounter_close_confirm_scans=1, clear_drive_distance=0.3))
    c.set_goal_heading(0.0)
    _enter_encounter(c)
    for t in range(10, 20):
        c.step(_sectors(front=5.0), None, None, 0.0, None, float(t))
    assert c._in_encounter is True  # never closes without a real position


def test_discontinuous_position_jump_aborts_measurement():
    c = AvoidanceController(_cfg(encounter_close_confirm_scans=1, clear_drive_distance=0.3,
                                  odom_jump_threshold=0.15, obstacle_confirm_scans=1))
    c.set_goal_heading(0.0)
    _enter_encounter(c)
    # The measurement must already be ACTIVE when the jump arrives -- otherwise
    # the jump is merely adopted as the first tracked position and the detector
    # is never exercised at all.
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.0)   # maneuver -> DRIVE
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.1)   # streak=1, start_pos=(0,0)
    c.step(_sectors(front=5.0), None, None, 0.0, (0.1, 0.0), 10.2)   # 0.1m of real travel
    c.step(_sectors(front=5.0), None, None, 0.0, (5.0, 0.0), 10.3)   # 5m jump -- discontinuity, not real motion
    assert c._in_encounter is True  # must NOT have closed off a bogus 5m "clear distance"
    assert c._clean_drive_start_pos is None  # measurement abandoned, not advanced
    # Post-jump: several small, realistic increments (each well under
    # odom_jump_threshold) summing past clear_drive_distance -- NOT one big
    # step, which would itself look like another discontinuity rather than
    # real incremental travel.
    c.step(_sectors(front=5.0), None, None, 0.0, (5.0, 0.0), 10.4)    # streak restarts here, start_pos=(5,0)
    c.step(_sectors(front=5.0), None, None, 0.0, (5.11, 0.0), 10.5)   # +0.11m
    c.step(_sectors(front=5.0), None, None, 0.0, (5.22, 0.0), 10.6)   # +0.11m (0.22m total)
    c.step(_sectors(front=5.0), None, None, 0.0, (5.33, 0.0), 10.7)   # +0.11m (0.33m total >= 0.3m)
    assert c._in_encounter is False


def test_encounter_closes_at_exactly_the_distance_threshold():
    c = AvoidanceController(_cfg(encounter_close_confirm_scans=1, clear_drive_distance=0.3,
                                  obstacle_confirm_scans=1))
    c.set_goal_heading(0.0)
    _enter_encounter(c)
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.0)   # maneuver -> DRIVE
    c.step(_sectors(front=5.0), None, None, 0.0, (0.0, 0.0), 10.1)   # streak=1, start_pos=(0,0)
    c.step(_sectors(front=5.0), None, None, 0.0, (0.1, 0.0), 10.2)
    c.step(_sectors(front=5.0), None, None, 0.0, (0.2, 0.0), 10.3)
    assert c._in_encounter is True                                   # 0.2m, still short
    c.step(_sectors(front=5.0), None, None, 0.0, (0.3, 0.0), 10.4)   # exactly 0.30m, not over
    assert c._in_encounter is False  # >= threshold must close, not only strictly >


def test_clear_driving_after_closure_does_not_reaccumulate():
    c = AvoidanceController(_cfg(encounter_close_confirm_scans=1, clear_drive_distance=0.3,
                                  obstacle_confirm_scans=1))
    c.set_goal_heading(0.0)
    _enter_encounter(c)
    # First tick completes the maneuver back to DRIVE; the rest accumulate.
    for i, x in enumerate((0.0, 0.0, 0.1, 0.2, 0.3)):   # closes on the last one
        c.step(_sectors(front=5.0), None, None, 0.0, (x, 0.0), 10.0 + i * 0.1)
    assert c._in_encounter is False
    # Continuing to drive clear afterward must not touch any tracking state
    # (it's a no-op once outside an encounter) -- no crash, no stale state.
    c.step(_sectors(front=5.0), None, None, 0.0, (1.0, 0.0), 10.5)
    out = c.step(_sectors(front=5.0), None, None, 0.0, (2.0, 0.0), 10.6)
    assert out.state == "DRIVE"
    assert c._in_encounter is False


# ---------- mid-maneuver invalidation: STRAFE ----------

def _strafe_entry_cfg():
    # strafe_speed/strafe_timeout pinned so STRAFE is actually reachable --
    # see the comment in test_assess_chooses_strafe_for_narrow_obstacle_with_clear_side.
    return _cfg(obstacle_confirm_scans=2, strafe_speed=0.25, strafe_timeout=1.5)


def _enter_strafe(c, cfg):
    """Two blocked DRIVE ticks (obstacle_confirm_scans=2) -> _assess() -> STRAFE."""
    obst = {"span_deg": 10.0, "y_lo": -0.05, "y_hi": 0.05, "preferred_side": 1.0}
    front = _blocked_front(cfg)
    blocked = _sectors(front=front, fc=front, left=1.5)
    c.step(blocked, obst, None, 0.0, (0.0, 0.0), 0.0)          # confirming
    out = c.step(blocked, obst, None, 0.0, (0.0, 0.0), 0.1)    # -> _assess -> STRAFE
    assert out.state == "STRAFE", f"setup failed, got {out.state}"
    assert c.consecutive_avoid_count == 1
    return obst


def test_strafe_aborts_when_flank_closing_sustained():
    cfg = _strafe_entry_cfg()
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_strafe(c, cfg)
    # Flank drops below strafe_side_clearance_min (0.20) for 2 consecutive ticks.
    # front held at 0.25: above safety_distance so it is not "blocked", below
    # clear_threshold (0.30) so the strafe does not complete -- isolating the flank.
    closing = _sectors(front=0.25, left=0.15)
    c.step(closing, obst, None, 0.0, (0.0, 0.0), 0.2)          # count=1
    out = c.step(closing, obst, None, 0.0, (0.0, 0.0), 0.3)    # count=2 -> abort
    assert c.consecutive_avoid_count == 2                       # _assess() ran again
    assert out.state == "TURN"   # left=0.15 < strafe_side_clearance_min -> can't re-strafe


def test_strafe_does_not_abort_on_single_close_reading():
    cfg = _strafe_entry_cfg()
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_strafe(c, cfg)
    out = c.step(_sectors(front=0.25, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.2)
    assert out.state == "STRAFE"
    assert c.consecutive_avoid_count == 1      # no re-assess -- only one close tick


def test_strafe_abort_counter_resets_on_clear_tick():
    cfg = _strafe_entry_cfg()
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_strafe(c, cfg)
    c.step(_sectors(front=0.25, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.2)   # count=1
    c.step(_sectors(front=0.25, left=1.5), obst, None, 0.0, (0.0, 0.0), 0.3)    # count=0
    out = c.step(_sectors(front=0.25, left=0.15), obst, None, 0.0, (0.0, 0.0), 0.4)  # count=1
    assert out.state == "STRAFE"
    assert c.consecutive_avoid_count == 1      # never reached 2 consecutive


def test_strafe_does_not_abort_on_blocked_front_alone():
    # Regression guard for the removed absolute-front term. front <= safety_distance
    # is the NORMAL condition throughout a strafe (it is why the strafe started), so
    # a wide-open flank must keep the maneuver running no matter how close front is.
    cfg = _strafe_entry_cfg()
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_strafe(c, cfg)
    blocked_front_open_flank = _sectors(front=0.10, left=1.5)
    for i, t in enumerate((0.2, 0.3, 0.4, 0.5)):
        out = c.step(blocked_front_open_flank, obst, None, 0.0, (0.0, 0.0), t)
        assert out.state == "STRAFE", f"aborted on tick {i} with an open flank"
    assert c.consecutive_avoid_count == 1      # never re-assessed


def test_aborted_maneuver_replans_and_resumes_driving():
    """STRAFE aborts mid-maneuver, re-plans, and returns to normal driving.

    The isolated tests above each pin one behavior; this proves the abort is
    RECOVERABLE rather than merely detected -- that the ladder returns to
    driving instead of stalling or escalating once the hazard passes.
    """
    cfg = _strafe_entry_cfg()
    c = AvoidanceController(cfg)
    c.set_goal_heading(0.0)
    obst = _enter_strafe(c, cfg)
    encounter = c.encounter_id

    # Second obstacle appears alongside: the flank we committed to closes.
    closing = _sectors(front=0.25, left=0.15)
    c.step(closing, obst, None, 0.0, (0.0, 0.0), 0.2)
    out = c.step(closing, obst, None, 0.0, (0.0, 0.0), 0.3)
    assert out.state == "TURN"          # aborted the strafe, re-planned as a turn

    # Both obstacles clear. TURN runs out its turn_timeout (1.5s) and hands off to
    # DRIVE_PAST, which needs front >= clear_threshold and inside flank >=
    # pass_clearance sustained for clear_confirm_time (0.5s) before completing.
    clear = _sectors(front=5.0, left=5.0, right=5.0)
    c.step(clear, None, None, 0.0, (0.0, 0.0), 0.4)     # still turning
    c.step(clear, None, None, 0.0, (0.0, 0.0), 2.0)     # timeout -> DRIVE_PAST
    out = c.step(clear, None, None, 0.0, (0.0, 0.0), 2.7)   # clear_confirm_time met

    assert out.state == "DRIVE"
    assert out.linear_x > 0.0                       # actually driving again
    assert c.encounter_id == encounter              # same encounter throughout
    assert c.has_recovered_this_encounter is False  # never escalated to RECOVER
    assert c.consecutive_avoid_count <= cfg.max_avoid_attempts
