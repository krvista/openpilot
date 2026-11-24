"""Phase 80 (routes a8 / a9, report 111): blinker state decode, lane changes not frozen by the low-confidence blend,
blinker concession ends after the driver's release during op's own lane change, lane-turn desire not inside a lane change."""
import types

import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd
from openpilot.cereal import log
from opendbc.can import CANParser
from opendbc.car.hyundai import carstate as cs_mod
from opendbc.car.hyundai.values import CarControllerParams as P
from phase_tests.harness import Sim, run_signal
from phase_tests.test_phase37a import settle
from phase_tests.test_noncontrol_controlsd_state import mk_controls, mk_model, run
from phase_tests.test_phase49 import mk_lanes

LCS = log.LaneChangeState


# ------------------------------------------------------------------ 80a blinker state
def test_blinker_state_mapping():
  f = cs_mod.blinkers_alt_state
  assert f(0) == (False, False)
  for v in (1, 2, 3):
    assert f(v) == (True, False), v                                  # left: first 4 s / after 4 s / standstill
  for v in (4, 5, 6):
    assert f(v) == (False, True), v                                  # right: 5 used to read as left + right
  assert f(7) == (True, True)
  assert P.BLINKERS_ALT_STATE is True


def test_dbc_decodes_the_logged_bytes():
  cp = CANParser("hyundai_canfd_generated", [("BLINKERS_ALT", 0)], 1)
  seen = []
  for i, b in enumerate((0x04, 0x05, 0x00, 0x01, 0x02, 0x03, 0x06)):
    cp.update([((i + 1) * 200_000_000, [(0x400, bytes([0, 0, 0, 0, b, 0, 0, 0]), 1)])])
    seen.append(cs_mod.blinkers_alt_state(cp.vl["BLINKERS_ALT"]["BLINKER_STATE"]))
  assert seen == [(False, True), (False, True), (False, False), (True, False), (True, False), (True, False), (False, True)]


def test_carstate_source_uses_the_state_and_keeps_the_kill_path():
  src = open(cs_mod.__file__).read()
  assert 'blinkers_alt_state(cp.vl["BLINKERS_ALT"]["BLINKER_STATE"])' in src
  assert 'self.update_blinker_from_lamp(50, cp.vl["BLINKERS_ALT"]["LEFT_LAMP"]' in src


# ------------------------------------------------------------------ 80b low-confidence blend in op's own lane change
def _weak(k, state):
  m = mk_lanes(k, 0.02, 0.02)
  m.meta.laneChangeState = state
  return m


def _lc_run(state, k_new=-0.0008):
  s = mk_controls(); run(s, 200, mk_model(fallback=0.0012))
  flags = 0
  for _ in range(60):
    run(s, 1, _weak(k_new, state)); flags |= s.steer_flags
  return s.desired_curvature, flags


def test_lane_change_is_not_frozen_by_the_blend():
  assert cd.CONF_BLEND_LC_EXEMPT is True and cd.CONF_BLEND_LC_AFTER_S == 1.0
  k_lc, f_lc = _lc_run(LCS.laneChangeStarting)
  k_pre, f_pre = _lc_run(LCS.preLaneChange)
  assert not f_lc & cd.STEER_FLAG_CONF_BLEND and f_pre & cd.STEER_FLAG_CONF_BLEND
  assert k_lc < 0.0 < k_pre                                          # followed the model's unwind / still held the old value


def test_exemption_lasts_one_second_after_the_lane_change():
  s = mk_controls(); run(s, 200, mk_model(fallback=0.0012))
  run(s, 50, _weak(0.0012, LCS.laneChangeStarting))
  f = 0
  for _ in range(90):
    run(s, 1, _weak(-0.0008, LCS.off)); f |= s.steer_flags
  assert not f & cd.STEER_FLAG_CONF_BLEND
  for _ in range(40):
    run(s, 1, _weak(-0.0008, LCS.off)); f |= s.steer_flags
  assert f & (cd.STEER_FLAG_CONF_BLEND | cd.STEER_FLAG_LANE_DROPOUT)   # the ordinary restraints are back


def test_blend_kill_switch():
  old = cd.CONF_BLEND_LC_EXEMPT
  try:
    cd.CONF_BLEND_LC_EXEMPT = False
    _, f = _lc_run(LCS.laneChangeStarting)
    assert f & cd.STEER_FLAG_CONF_BLEND
  finally:
    cd.CONF_BLEND_LC_EXEMPT = old


# ------------------------------------------------------------------ 80c concession after the release
V = 60 / 3.6


def _lc(tq_fn, lc_active=True, n=400, **kw):
  sim = Sim(); settle(sim)
  run_signal(sim, 200, v=V, wheel=0.0, cmd=0.0, tq=0.0)
  tr = run_signal(sim, n, tq_fn=tq_fn, v=V, wheel=0.0, cmd=0.0, blinker=True, cc_blinker_left=True, cc_lc_active=lc_active, **kw)
  return sim, tr


def test_concession_ends_after_release_in_ops_own_lane_change():
  assert P.LC_RELEASE_FRAMES == 30 and P.LC_RELEASE_NM == 260.0
  sim, tr = _lc(lambda i: 0.0)
  assert not sim.s.blinker_concession and sim.s.lc_released and tr['gain'][-1] > 0.6
  sim, tr = _lc(lambda i: 0.0, lc_active=False)                      # blinker only / preLaneChange: unchanged
  assert sim.s.blinker_concession and tr['gain'][-1] <= 0.45 + 1e-6


def test_a_hand_brings_the_concession_back_at_once():
  sim, tr = _lc(lambda i: 0.0 if i < 300 else 450.0, pressed=lambda i: i >= 300, n=320)
  assert sim.s.blinker_concession and not sim.s.lc_released
  sim, tr = _lc(lambda i: 450.0, pressed=True)                       # never released: concession throughout
  assert sim.s.blinker_concession and tr['gain'][-1] <= 0.30


def test_release_needs_the_debounce_and_respects_bsm():
  sim, tr = _lc(lambda i: 0.0, n=20)
  assert sim.s.blinker_concession
  sim, tr = _lc(lambda i: 0.0, bs_l=True)                            # toward an occupied side: 37b-2 keeps the concession
  assert sim.s.blinker_concession


def test_concession_kill_switch():
  old = P.LC_RELEASE_FRAMES
  try:
    P.LC_RELEASE_FRAMES = 0
    sim, tr = _lc(lambda i: 0.0)
    assert sim.s.blinker_concession and tr['gain'][-1] <= 0.45 + 1e-6
  finally:
    P.LC_RELEASE_FRAMES = old


# ------------------------------------------------------------------ 80d lane-turn desire
def _cs(v=25.0, lb=False, pressed=False, tq=0.0):
  return types.SimpleNamespace(vEgo=v, leftBlinker=lb, rightBlinker=False, leftBlindspot=False, rightBlindspot=False,
                               steeringPressed=pressed, steeringTorque=tq, brakePressed=False)


def _started(kill=False):
  FakeParams().put("LaneTurnValue", "20"); FakeParams().put_bool("LaneTurnDesire", True); FakeParams().put("AutoLaneChangeTimer", 0)
  from openpilot.selfdrive.controls.lib import desire_helper as dh
  old = dh.LANE_TURN_NOT_IN_LANE_CHANGE
  dh.LANE_TURN_NOT_IN_LANE_CHANGE = not kill
  try:
    DH = dh.DesireHelper()
    DH.update(_cs(), True, 0.0); DH.update(_cs(lb=True), True, 0.0)
    DH.update(_cs(lb=True, pressed=True, tq=50.0), True, 0.0)
    assert DH.lane_change_state == LCS.laneChangeStarting
    DH.update(_cs(v=7.0, lb=True), True, 1.0)                        # speed falls below the lane-turn speed mid-change
    return DH.desire, DH.lane_change_state
  finally:
    dh.LANE_TURN_NOT_IN_LANE_CHANGE = old


def test_lane_change_keeps_its_desire_below_the_turn_speed():
  d, st = _started()
  assert st == LCS.laneChangeStarting and d == log.Desire.laneChangeLeft
  d, st = _started(kill=True)
  assert d == log.Desire.turnLeft


def test_turn_desire_still_works_outside_a_lane_change():
  FakeParams().put("LaneTurnValue", "20"); FakeParams().put_bool("LaneTurnDesire", True); FakeParams().put("AutoLaneChangeTimer", 0)
  from openpilot.selfdrive.controls.lib import desire_helper as dh
  DH = dh.DesireHelper()
  DH.update(_cs(v=5.0), True, 0.0); DH.update(_cs(v=5.0, lb=True), True, 0.0)
  assert DH.desire == log.Desire.turnLeft
