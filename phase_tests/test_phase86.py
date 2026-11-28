"""Phase 86 (routes b4 / b5 / aa / ac, report 118.2 / 119): no "take control / turn exceeds limit" while the driver is
already steering toward the blinker side and the lane change still waits for its nudge. Only the event is exempt."""
import os

import pytest

import openpilot.selfdrive.selfdrived.selfdrived as sd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMPACT_TQ = (233.0, 428.0, 336.0, -25.0, -331.0, -351.0, -154.0, 50.0, 148.0, 78.0)   # route b6 4980.98 s on (hands off)


@pytest.fixture
def keep():
  saved = {k: getattr(sd, k) for k in ("LC_PUSH_SAT_EXEMPT", "LC_PUSH_TORQUE", "LC_PUSH_FRAMES", "LC_PUSH_HOLD_S")}
  yield
  for k, v in saved.items():
    setattr(sd, k, v)


def feed(bp, n, tq, left=False, right=True, pre=True):
  return [bp.update(left, right, tq, pre) for _ in range(n)]


def test_shipped_values():
  assert sd.LC_PUSH_SAT_EXEMPT is True
  assert sd.LC_PUSH_TORQUE == 150.0 and sd.LC_PUSH_FRAMES == 8 and sd.LC_PUSH_HOLD_S == 1.5
  assert sd.LC_PUSH_TORQUE < 350                                     # below the nudge that starts the lane change


def test_event_is_gated_at_the_alert_only():
  src = open(os.path.join(ROOT, "openpilot/selfdrive/selfdrived/selfdrived.py")).read()
  assert "lc_push = self.lc_push.update(CS.leftBlinker, CS.rightBlinker, CS.steeringTorque," in src
  assert "self.sm['modelV2'].meta.laneChangeState == LaneChangeState.preLaneChange)" in src
  assert "if lac.active and not recent_steer_pressed and not self.CP.notCar:" in src   # the 2 s press guard is unchanged
  assert "if undershooting and turning and lac.saturated and not lc_push:" in src
  assert src.count("lc_push") == 4 and src.count("self.lc_push") == 2   # nothing else reads it
  lat = open(os.path.join(ROOT, "openpilot/selfdrive/controls/lib/latcontrol.py")).read()
  assert "and not steer_limited_by_safety and not CS.steeringPressed:" in lat and "LC_PUSH" not in lat


def test_push_toward_the_right_blinker_exempts_after_eight_frames():
  bp = sd.BlinkerPush()
  out = feed(bp, 10, -200.0)                                          # + = left, so a right push is negative
  assert out == [False] * 7 + [True] * 3


def test_push_toward_the_left_blinker():
  bp = sd.BlinkerPush()
  assert feed(bp, 8, 285.0, left=True, right=False)[-1]               # b5 1581 s
  bp = sd.BlinkerPush()
  assert not any(feed(bp, 300, -285.0, left=True, right=False))       # pushing away from the blinker


def test_hands_off_alert_fires_as_before():
  bp = sd.BlinkerPush()
  assert not any(feed(bp, 500, 0.0))
  assert not any(feed(bp, 500, -149.0))                               # a resting hand below the threshold
  assert not any(feed(bp, 500, -400.0, right=False))                  # no blinker
  assert not any(feed(bp, 500, -400.0, left=True))                    # hazard lights: both lamps, no side


def test_only_while_the_lane_change_waits():
  bp = sd.BlinkerPush()
  assert not any(feed(bp, 300, -300.0, pre=False))                    # off / starting / finishing
  assert feed(bp, 8, -300.0)[-1]
  assert not bp.update(False, True, -300.0, False)                    # the state leaves preLaneChange: at once
  assert feed(bp, 8, -300.0) == [False] * 7 + [True]                  # and the count starts again


def test_road_impact_does_not_exempt():
  bp = sd.BlinkerPush()
  out = []
  for _ in range(20):                                                 # ringing column torque, sign flips in 2-3 samples
    out += [bp.update(False, True, -tq, True) for tq in IMPACT_TQ]
    out += [bp.update(False, True, tq, True) for tq in IMPACT_TQ]
  assert not any(out)
  assert not any(feed(bp, 7, -500.0) + feed(bp, 1, -100.0) + feed(bp, 7, -500.0))   # seven frames are not enough


def test_hold_after_the_push_then_the_alert_returns():
  bp = sd.BlinkerPush()
  assert feed(bp, 30, -250.0)[-1]
  out = feed(bp, 300, 120.0)                                          # eases off / counter-steers once across (b4 1084 s)
  n = int(round(sd.LC_PUSH_HOLD_S * 100))
  assert all(out[:n - 1]) and not any(out[n - 1:])                    # 1.5 s from the last pushing frame, then as before
  assert feed(bp, 8, -250.0)[-1]                                      # a new push starts it again


def test_blinker_off_or_other_side_ends_it_at_once():
  bp = sd.BlinkerPush()
  assert feed(bp, 30, -250.0)[-1]
  assert not bp.update(False, False, -250.0, True)
  assert feed(bp, 30, -250.0)[-1]
  assert not any(feed(bp, 7, -250.0, left=True, right=False))         # blinker now left: the right push does not count
  assert bp.count == 0 and bp.age == -1


def test_logged_case_b5_1505():
  """Right blinker, preLaneChange, the driver holds 260-310 until 0.85 s before the event (1505.82-1505.92 s)."""
  bp = sd.BlinkerPush()
  feed(bp, 80, -280.0)
  out = feed(bp, 85, -60.0) + feed(bp, 12, 100.0)
  assert all(out)


def test_kill_switch(keep):
  sd.LC_PUSH_SAT_EXEMPT = False
  bp = sd.BlinkerPush()
  assert not any(feed(bp, 300, -400.0))
  assert not any(feed(bp, 300, 400.0, left=True, right=False))
  assert bp.count == 0 and bp.age == -1


def test_kill_switch_mid_hold(keep):
  bp = sd.BlinkerPush()
  assert feed(bp, 30, -250.0)[-1]
  sd.LC_PUSH_SAT_EXEMPT = False
  assert not bp.update(False, True, -250.0, True)
