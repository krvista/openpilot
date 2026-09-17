"""Phase 41: low-lane-confidence steering-rate cap (lowconf_rate_cap.py) + 41-2 lookahead cap step.

Unit tests drive LowConfRateCap directly; the integration tests drive the real Controls.state_control() through the
manual fixture of test_noncontrol_controlsd_state so the cap is proven in its place in the lateral chain (after the
6g blend / 39-2 hold, before the departure and BSM guards and the 6h-1 LP).
"""
import math

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)
from phase_tests.test_noncontrol_controlsd_state import mk_controls, mk_model, run

import openpilot.selfdrive.controls.controlsd as cd
from openpilot.selfdrive.controls.lib import lowconf_rate_cap as lc

DT = 0.01
KPD = lc.NOMINAL_K_PER_DEG      # curvature per deg of steer used by the tests (the fixture VM returns 0 -> fallback)
V = 12.0                        # m/s (43 km/h): inside the gate's speed range


def step_response(cap, lane_min, target_deg, frames, pressed=False, blinker=False, v=V):
  """Command a step of target_deg (as curvature) every frame; return the commanded-curvature trace."""
  prev = 0.0; out = []
  for _ in range(frames):
    prev = cap.update(target_deg * KPD, prev, lane_min, pressed, blinker, v, KPD)
    out.append(prev)
  return out


class TestUnit:
  def test_gate_off_passthrough(self):
    cap = lc.LowConfRateCap(DT)
    tr = step_response(cap, 0.9, 10.0, 5)
    assert tr[0] == 10.0 * KPD and not cap.active and not cap.capped

  def test_capped_at_six_deg_per_second(self):
    cap = lc.LowConfRateCap(DT)
    tr = step_response(cap, 0.2, 10.0, 50)          # 0.5 s
    assert cap.active and cap.capped
    reached_deg = tr[-1] / KPD
    assert abs(reached_deg - 0.5 * lc.LOWCONF_CAP_DPS) < 0.05    # 3.0 deg after 0.5 s, not 10
    # monotone, every frame exactly one cap step
    steps = [(b - a) / KPD for a, b in zip(tr[:-1], tr[1:])]
    assert all(abs(s - lc.LOWCONF_CAP_DPS * DT) < 1e-9 for s in steps)

  def test_below_speed_gate_passthrough(self):
    cap = lc.LowConfRateCap(DT)
    tr = step_response(cap, 0.2, 10.0, 3, v=6.0)    # 22 km/h
    assert tr[0] == 10.0 * KPD and not cap.active

  def test_driver_press_lifts_cap_at_once(self):
    cap = lc.LowConfRateCap(DT)
    step_response(cap, 0.2, 10.0, 20)
    assert cap.active
    out = cap.update(10.0 * KPD, 0.0, 0.2, True, False, V, KPD)
    assert out == 10.0 * KPD and not cap.active and cap.release_left == 0

  def test_blinker_or_lane_change_lifts_cap(self):
    cap = lc.LowConfRateCap(DT)
    out = cap.update(10.0 * KPD, 0.0, 0.2, False, True, V, KPD)
    assert out == 10.0 * KPD and not cap.active

  def test_release_ramps_instead_of_stepping(self):
    cap = lc.LowConfRateCap(DT)
    tr = step_response(cap, 0.2, 10.0, 30)          # held back: 1.8 deg of a 10 deg plan
    prev = tr[-1]; deltas = []
    for _ in range(int(lc.LOWCONF_RELEASE_S / DT) + 5):
      nxt = cap.update(10.0 * KPD, prev, 0.9, False, False, V, KPD)   # lines back: gate off
      deltas.append((nxt - prev) / KPD / DT); prev = nxt
    # first release frame is still near the 6 deg/s cap, the ramp never exceeds RELEASE_DPS, and the plan is reached
    assert deltas[0] < lc.LOWCONF_CAP_DPS + 1.0
    assert max(deltas) <= lc.LOWCONF_RELEASE_DPS + 1e-6
    assert abs(prev - 10.0 * KPD) < 1e-12
    # after the ramp the cap is fully released: a fresh 10 deg step passes in one frame
    assert cap.update(20.0 * KPD, prev, 0.9, False, False, V, KPD) == 20.0 * KPD

  def test_zero_vm_ratio_falls_back_to_nominal(self):
    cap = lc.LowConfRateCap(DT)
    out = cap.update(10.0 * KPD, 0.0, 0.2, False, False, V, 0.0)
    assert abs(out / KPD - lc.LOWCONF_CAP_DPS * DT) < 1e-9

  def test_kill_switch(self):
    cap = lc.LowConfRateCap(DT)
    assert cap.update(10.0 * KPD, 0.0, 0.2, False, False, V, KPD, cap_dps=0.0) == 10.0 * KPD

  def test_shipped_defaults(self):
    assert lc.LOWCONF_CAP_DPS == 6.0 and lc.LOWCONF_LANE_MIN == 0.3 and lc.LOWCONF_RELEASE_DPS == 30.0
    assert abs(lc.LOWCONF_MIN_SPEED - 30.0 / 3.6) < 1e-9
    assert cd.LOOKAHEAD_T_AHEAD_CAP == 0.32      # 41-2


def _model(k, probs):
  m = mk_model(fallback=k)
  m.laneLineProbs = list(probs)
  return m


class TestIntegration:
  def _settle(self, s):
    run(s, 200, _model(0.0, [0.9, 0.95, 0.95, 0.9]))

  def test_low_confidence_step_is_rate_capped_in_state_control(self):
    s = mk_controls(); s.sm['carState'].vEgo = V
    self._settle(s)
    k_step = 0.006                                            # ~15 deg at the nominal ratio
    curv, _ = run(s, 50, _model(k_step, [0.9, 0.2, 0.95, 0.9]))   # weaker inner line at 0.2 for 0.5 s
    assert s.lowconf_cap.active
    assert abs(curv) <= (0.5 * lc.LOWCONF_CAP_DPS + 0.3) * KPD    # <= ~3.3 deg worth, not 15
    # same step with confident lines is (nearly) through after 0.5 s
    s2 = mk_controls(); s2.sm['carState'].vEgo = V
    self._settle(s2)
    curv2, _ = run(s2, 50, _model(k_step, [0.9, 0.95, 0.95, 0.9]))
    assert abs(curv2) > 0.5 * k_step
    assert abs(curv2) > 2.0 * abs(curv)

  def test_driver_press_passes_plan_through(self):
    s = mk_controls(); s.sm['carState'].vEgo = V
    self._settle(s)
    s.sm['carState'].steeringPressed = True
    curv, _ = run(s, 50, _model(0.006, [0.9, 0.2, 0.95, 0.9]))
    assert not s.lowconf_cap.active and abs(curv) > 0.003

  def test_cap_resets_when_lateral_inactive(self):
    s = mk_controls(); s.sm['carState'].vEgo = V
    self._settle(s)
    run(s, 30, _model(0.006, [0.9, 0.2, 0.95, 0.9]))
    assert s.lowconf_cap.release_left > 0
    s.get_lat_active = lambda sm: False
    run(s, 1, _model(0.006, [0.9, 0.2, 0.95, 0.9]))
    assert s.lowconf_cap.release_left == 0 and not s.lowconf_cap.active

  def test_no_effect_on_confident_straight_driving(self):
    s = mk_controls(); s.sm['carState'].vEgo = V
    self._settle(s)
    for _ in range(100):
      run(s, 1, _model(0.0005 * math.sin(_ / 10.0), [0.9, 0.95, 0.95, 0.9]))
      assert not s.lowconf_cap.active and not s.lowconf_cap.capped
