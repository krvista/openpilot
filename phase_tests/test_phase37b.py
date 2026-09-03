"""Phase 37b: blindspot lane protection — ALC abort, no blinker concession toward BSM, BSM lane guard."""
import types
import numpy as np
from phase_tests.harness import Sim, run_signal
from opendbc.car.hyundai.values import CarControllerParams as P


def settle(sim, n=120, **kw):
  kw.setdefault('v', 15.0); kw.setdefault('wheel', 0.0); kw.setdefault('cmd', 0.0); kw.setdefault('tq', 0.0)
  return run_signal(sim, n, **kw)


class TestBsmLaneGuard:
  def test_pure_function(self):
    from openpilot.selfdrive.controls.lib.bsm_guard import bsm_lane_guard
    # left occupied, left line close, steering more left (negative) -> hold
    assert bsm_lane_guard(-0.002, -0.001, True, False, False, False, -0.6, 1.8, 0.9, 0.9) == (-0.001, True)
    # same but steering right -> pass
    assert bsm_lane_guard(0.000, -0.001, True, False, False, False, -0.6, 1.8, 0.9, 0.9) == (0.000, False)
    # left blinker toward it -> driver's call, pass
    assert bsm_lane_guard(-0.002, -0.001, True, False, True, False, -0.6, 1.8, 0.9, 0.9) == (-0.002, False)
    # line not close / low prob -> pass
    assert bsm_lane_guard(-0.002, -0.001, True, False, False, False, -1.4, 1.8, 0.9, 0.9)[1] is False
    assert bsm_lane_guard(-0.002, -0.001, True, False, False, False, -0.6, 1.8, 0.3, 0.9)[1] is False
    # right side mirror
    assert bsm_lane_guard(0.002, 0.001, False, True, False, False, -1.8, 0.5, 0.9, 0.9) == (0.001, True)
    assert bsm_lane_guard(0.002, 0.001, False, True, False, True, -1.8, 0.5, 0.9, 0.9) == (0.002, False)
    # kill
    assert bsm_lane_guard(-0.002, -0.001, True, False, False, False, -0.6, 1.8, 0.9, 0.9, guard_m=0.0) == (-0.002, False)


class TestBsmLaneGuardTimeout:
  def test_hold_releases_after_max_s(self):
    from openpilot.selfdrive.controls.lib.bsm_guard import BsmLaneGuard
    g = BsmLaneGuard(0.01, max_s=2.0)
    args = (True, False, False, False, -0.6, 1.8, 0.9, 0.9)
    held = [g.update(-0.002, -0.001, *args)[1] for _ in range(200)]
    assert all(held)
    assert g.update(-0.002, -0.001, *args) == (-0.002, False)      # frame 201: released
    g.update(0.0, -0.001, *args)                                    # unblocked frame resets
    assert g.update(-0.002, -0.001, *args) == (-0.001, True)

  def test_bsm_off_gap_resets_counter(self):
    # review: an exhausted counter must not latch across a BSM-off gap — the
    # guard is called every frame, and a no-BSM frame is an unblocked frame
    from openpilot.selfdrive.controls.lib.bsm_guard import BsmLaneGuard
    g = BsmLaneGuard(0.01, max_s=2.0)
    on = (True, False, False, False, -0.6, 1.8, 0.9, 0.9)
    off = (False, False, False, False, -0.6, 1.8, 0.9, 0.9)
    for _ in range(250):
      g.update(-0.002, -0.001, *on)
    assert g.update(-0.002, -0.001, *on)[1] is False                # exhausted
    g.update(-0.002, -0.001, *off)                                  # BSM gap
    assert g.frames == 0
    assert g.update(-0.002, -0.001, *on) == (-0.001, True)          # fresh episode guarded
    g.reset(); assert g.frames == 0


def _mk_cs(v=25.0, lb=False, rb=False, bsl=False, bsr=False, pressed=False, tq=0.0):
  return types.SimpleNamespace(vEgo=v, leftBlinker=lb, rightBlinker=rb, leftBlindspot=bsl, rightBlindspot=bsr,
                               steeringPressed=pressed, steeringTorque=tq, brakePressed=False)


class TestAlcAbortOnBsm:
  def _dh(self):
    from phase_tests.harness_noncontrol import FakeParams
    FakeParams().put("LaneTurnValue", "20")          # sunnypilot lane-turn helper reads these
    FakeParams().put("AutoLaneChangeTimer", 0)       # nudge mode
    from openpilot.selfdrive.controls.lib import desire_helper as dh
    return dh, dh.DesireHelper()

  def _start_left(self, dh, DH):
    from cereal import log
    LCS = log.LaneChangeState
    DH.update(_mk_cs(), True, 0.0)                       # off, no blinker
    DH.update(_mk_cs(lb=True), True, 0.0)                # blinker edge -> preLaneChange
    assert DH.lane_change_state == LCS.preLaneChange
    DH.update(_mk_cs(lb=True, pressed=True, tq=50.0), True, 0.0)   # nudge -> starting
    assert DH.lane_change_state == LCS.laneChangeStarting
    return LCS

  def test_abort_when_target_side_becomes_occupied(self):
    dh, DH = self._dh(); LCS = self._start_left(dh, DH)
    DH.update(_mk_cs(lb=True, bsl=True), True, 0.5)
    assert DH.lane_change_state == LCS.off
    from cereal import log
    assert DH.lane_change_direction == log.LaneChangeDirection.none
    # blinker still on: must NOT re-arm without a fresh blinker edge
    DH.update(_mk_cs(lb=True), True, 0.5)
    assert DH.lane_change_state == LCS.off

  def test_late_detection_does_not_abort(self):
    dh, DH = self._dh(); LCS = self._start_left(dh, DH)
    for _ in range(int(1.2 / 0.05)):                     # 1.2 s into the crossing (DT_MDL 0.05)
      DH.update(_mk_cs(lb=True), True, 0.5)
    assert DH.lane_change_state == LCS.laneChangeStarting
    DH.update(_mk_cs(lb=True, bsl=True), True, 0.5)
    assert DH.lane_change_state == LCS.laneChangeStarting   # too late to abort: runs on to finishing

  def test_other_side_bsm_does_not_abort(self):
    dh, DH = self._dh(); LCS = self._start_left(dh, DH)
    DH.update(_mk_cs(lb=True, bsr=True), True, 0.5)
    assert DH.lane_change_state == LCS.laneChangeStarting

  def test_kill_keeps_upstream_behaviour(self):
    dh, DH = self._dh()
    old = dh.LANE_CHANGE_BSM_ABORT
    try:
      dh.LANE_CHANGE_BSM_ABORT = False
      LCS = self._start_left(dh, DH)
      DH.update(_mk_cs(lb=True, bsl=True), True, 0.5)
      assert DH.lane_change_state == LCS.laneChangeStarting
    finally:
      dh.LANE_CHANGE_BSM_ABORT = old


class TestNoBlinkerConcessionTowardBsm:
  # 40 km/h (hold comp ~100 Nm): a light hand (raw 100 -> driver_tq ~0) sees the
  # blinker CEILING (0.45 vs the 0.75 rung); a firmer hand (raw 230 -> driver_tq
  # ~130 >= 120) fires the blinker ANCHOR. Toward an occupied side both stay at
  # their non-blinker behaviour; a pressed grip (raw > 350) still overrides.
  V = 11.0

  def _run(self, tq, bs_l, bs_r, n=300, cmd=0.0, wheel=0.0):
    sim = Sim(); settle(sim)
    run_signal(sim, 200, v=self.V, wheel=0.0, cmd=0.0, tq=0.0)
    tr = run_signal(sim, n, v=self.V, wheel=wheel, cmd=cmd, tq=tq, blinker=True, bs_l=bs_l, bs_r=bs_r)
    return sim, tr

  def test_ceiling_kept_toward_occupied_side(self):
    sim_bs, tr_bs = self._run(tq=100.0, bs_l=True, bs_r=False)
    sim_ok, tr_ok = self._run(tq=100.0, bs_l=False, bs_r=False)
    assert not sim_bs.s.blinker_concession and sim_ok.s.blinker_concession
    assert tr_ok['gain'][-1] <= 0.46, tr_ok['gain'][-1]            # blinker ceiling 0.45
    assert tr_bs['gain'][-1] >= 0.70, tr_bs['gain'][-1]            # 24a rung at 40 km/h

  def test_anchor_not_fired_toward_occupied_side(self):
    sim_bs, tr_bs = self._run(tq=230.0, bs_l=True, bs_r=False)
    sim_ok, tr_ok = self._run(tq=230.0, bs_l=False, bs_r=False)
    assert any(tr_ok['blinker_anchor_on']) and not any(tr_bs['blinker_anchor_on'])

  def test_bsm_on_the_other_side_keeps_concession(self):
    sim, tr = self._run(tq=100.0, bs_l=False, bs_r=True)
    sim_ok, tr_ok = self._run(tq=100.0, bs_l=False, bs_r=False)
    assert sim.s.blinker_concession
    assert abs(tr['gain'][-1] - tr_ok['gain'][-1]) <= 0.004 + 1e-9

  def test_real_grip_still_overrides_toward_occupied_side(self):
    sim, tr = self._run(tq=460.0, bs_l=True, bs_r=False, n=200, wheel=5.0)
    assert tr['gain'][-1] <= 0.10, tr['gain'][-1]

  def test_kill_restores_concession(self):
    old = P.BSM_BLINKER_NO_CONCESSION
    try:
      P.BSM_BLINKER_NO_CONCESSION = False
      sim, tr = self._run(tq=230.0, bs_l=True, bs_r=False)
    finally:
      P.BSM_BLINKER_NO_CONCESSION = old
    assert sim.s.blinker_concession and any(tr['blinker_anchor_on'])
