"""Phase 6g-4 op-active LDW departure detector — synthetic-history tests
against the real LaneDepartureWarning (selfdrive/controls/lib/ldw.py)."""
import types

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib import ldw as ldw_mod
from openpilot.selfdrive.controls.lib.ldw import (
  LaneDepartureWarning, OP_LDW_WINDOW, OP_LDW_CLR_M, OP_LDW_MIN_PROB,
  OP_LDW_STEP_TOL_M, OP_LDW_JUMP_M, OP_LDW_MIN_APPROACH_M, LDW_MIN_SPEED,
)


def _mk_model(left_y=0.9, right_y=1.5, l_prob=0.9, r_prob=0.9,
              lane_change=log.LaneChangeState.off, empty_y=False):
  line_l = types.SimpleNamespace(y=[] if empty_y else [-abs(left_y)])
  line_r = types.SimpleNamespace(y=[] if empty_y else [abs(right_y)])
  pad = types.SimpleNamespace(y=[0.0])
  return types.SimpleNamespace(
    laneLines=[pad, line_l, line_r, pad],
    laneLineProbs=[0.0, l_prob, r_prob, 0.0],
    meta=types.SimpleNamespace(laneChangeState=lane_change, desirePrediction=[]),
  )


def _mk_cs(v=20.0, left_blinker=False, right_blinker=False):
  return types.SimpleNamespace(vEgo=v, leftBlinker=left_blinker, rightBlinker=right_blinker)


def _mk_cc(lat_active=True):
  return types.SimpleNamespace(latActive=lat_active)


def run_frames(w, clr_seq_left, frame0=10_000, **model_kw):
  """Feed a left-clearance sequence; right line held far/steady."""
  results = []
  for i, clr in enumerate(clr_seq_left):
    m = _mk_model(left_y=clr, **model_kw)
    w.update(frame0 + i, m, _mk_cs(), _mk_cc())
    results.append((w.left, w.right))
  return results


def approach_seq(start=0.85, step=-0.03, n=OP_LDW_WINDOW):
  return [start + step * i for i in range(n)]


class TestOpActiveDeparture:
  def test_sustained_approach_fires_left_only(self):
    w = LaneDepartureWarning()
    res = run_frames(w, approach_seq())
    assert res[-1] == (True, False)

  def test_short_history_never_fires(self):
    w = LaneDepartureWarning()
    res = run_frames(w, approach_seq(n=OP_LDW_WINDOW - 1))
    assert all(r == (False, False) for r in res)

  def test_direct_short_history_no_index_error(self):
    # _op_active_side_departure on histories of every length < window
    for n in range(OP_LDW_WINDOW):
      hist = [(0.9, 0.5)] * n
      assert LaneDepartureWarning._op_active_side_departure(hist) is False

  def test_prob_gate_single_low_frame_blocks(self):
    w = LaneDepartureWarning()
    seq = approach_seq()
    for i, clr in enumerate(seq):
      prob = OP_LDW_MIN_PROB if i == 3 else 0.9  # one frame at (not above) the gate
      m = _mk_model(left_y=clr, l_prob=prob)
      w.update(10_000 + i, m, _mk_cs(), _mk_cc())
    assert (w.left, w.right) == (False, False)

  def test_not_close_enough_blocks(self):
    w = LaneDepartureWarning()
    seq = [c + OP_LDW_CLR_M for c in approach_seq()]  # same approach, far away
    res = run_frames(w, seq)
    assert res[-1] == (False, False)

  def test_relabel_jump_blocks(self):
    w = LaneDepartureWarning()
    seq = approach_seq()
    seq[3] = seq[2] - OP_LDW_JUMP_M  # lane re-identification step
    res = run_frames(w, seq)
    assert res[-1] == (False, False)

  def test_regression_blocks(self):
    w = LaneDepartureWarning()
    seq = approach_seq()
    seq[4] = seq[3] + OP_LDW_STEP_TOL_M  # one frame moving away >= tolerance
    res = run_frames(w, seq)
    assert res[-1] == (False, False)

  def test_min_total_approach(self):
    w = LaneDepartureWarning()
    # each step approaches, but total < OP_LDW_MIN_APPROACH_M
    step = -(OP_LDW_MIN_APPROACH_M / (OP_LDW_WINDOW + 2))
    res = run_frames(w, approach_seq(start=0.6, step=step))
    assert res[-1] == (False, False)

  def test_both_sides_fire_on_narrowing(self):
    w = LaneDepartureWarning()
    for i in range(OP_LDW_WINDOW):
      clr = 0.65 - 0.03 * i
      m = _mk_model(left_y=clr, right_y=clr)
      w.update(10_000 + i, m, _mk_cs(), _mk_cc())
    assert (w.left, w.right) == (True, True)

  def test_blinker_blocks_and_clears_history(self):
    w = LaneDepartureWarning()
    seq = approach_seq()
    for i, clr in enumerate(seq):
      m = _mk_model(left_y=clr)
      w.update(10_000 + i, m, _mk_cs(left_blinker=(i == 0)), _mk_cc())
    # blinker on frame 0 => 5 s cooldown covers the whole window
    assert (w.left, w.right) == (False, False)
    assert len(w._hist_l) == 0

  def test_low_speed_blocks(self):
    w = LaneDepartureWarning()
    for i, clr in enumerate(approach_seq()):
      m = _mk_model(left_y=clr)
      w.update(10_000 + i, m, _mk_cs(v=LDW_MIN_SPEED - 0.5), _mk_cc())
    assert (w.left, w.right) == (False, False)

  def test_lane_change_state_blocks(self):
    w = LaneDepartureWarning()
    res = run_frames(w, approach_seq(), lane_change=log.LaneChangeState.laneChangeStarting)
    assert res[-1] == (False, False)

  def test_empty_y_glitch_frame_no_crash(self):
    # model glitch frames can publish laneLines with empty y arrays;
    # the op-active path must not IndexError (defect fixed in ldw.py)
    w = LaneDepartureWarning()
    run_frames(w, approach_seq(n=4))
    m = _mk_model(empty_y=True)
    w.update(10_004, m, _mk_cs(), _mk_cc())  # must not raise
    assert (w.left, w.right) == (False, False)

  def test_nan_clearance_never_fires(self):
    w = LaneDepartureWarning()
    seq = approach_seq()
    seq[2] = float('nan')
    res = run_frames(w, seq)
    assert res[-1] == (False, False)

  def test_latactive_false_uses_manual_path_and_clears(self):
    w = LaneDepartureWarning()
    run_frames(w, approach_seq(n=4))
    assert len(w._hist_l) == 4
    m = _mk_model()
    w.update(10_004, m, _mk_cs(), _mk_cc(lat_active=False))
    assert len(w._hist_l) == 0 and not w.warning

  def test_kill_switch(self):
    old = ldw_mod.OP_LDW_CLR_M
    try:
      ldw_mod.OP_LDW_CLR_M = 0.0
      # module-level constant is read inside update via the module global
      w = LaneDepartureWarning()
      for i, clr in enumerate(approach_seq()):
        m = _mk_model(left_y=clr)
        w.update(10_000 + i, m, _mk_cs(), _mk_cc())
      assert (w.left, w.right) == (False, False)
    finally:
      ldw_mod.OP_LDW_CLR_M = old
