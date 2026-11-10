"""Phase 67 (report §88-89).
67a: lagd publishes at most LAGD_PUBLISH_MAX_S (the stock 0.65 s) to modeld on the CCNC profile; the learned value stays
in lateralDelayEstimate / the blocks / the cache.
67b: the orient-lead jerk budget no longer grows with the model delay (Phase 64b ref_shift).
67c: the stock modeld runner feeds the model the toggle-aware delay (model.lat_delay), like modeld_v2."""
import pathlib
import re
import types

import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd
import openpilot.selfdrive.locationd.lagd as lagd
from phase_tests.test_phase55 import model, call, settled


@pytest.fixture(autouse=True)
def _restore():
  saved = {nm: getattr(cd, nm) for nm in ("LOOKAHEAD_DELAY_EXTRA_S", "LOOKAHEAD_BUDGET_EXCLUDES_SHIFT")}
  yield
  for nm, val in saved.items():
    setattr(cd, nm, val)


def cp():
  return types.SimpleNamespace(brand="hyundai", steerControlType="angle", steerActuatorDelay=0.1, carFingerprint="X")


# ---------------------------------------------------------------- 67a
def test_published_delay_is_capped_at_the_stock_range():
  assert lagd.LAGD_PUBLISH_MAX_S == lagd.MAX_LAG == 0.65
  e = lagd.LateralLagEstimator(cp(), 0.05, **lagd.lagd_profile(cp()))
  e.reset(0.69, lagd.BLOCK_NUM_NEEDED + 2)                     # learned 0.69 s
  m = e.get_msg(True).lateralDelay
  assert str(m.status) == "estimated"
  assert m.lateralDelay == pytest.approx(0.65)                 # what modeld / controlsd read
  assert m.lateralDelayEstimate == pytest.approx(0.69)         # what is learned, logged and cached


def test_learned_value_below_the_cap_is_published_as_is():
  e = lagd.LateralLagEstimator(cp(), 0.05, **lagd.lagd_profile(cp()))
  e.reset(0.58, lagd.BLOCK_NUM_NEEDED + 2)
  assert e.get_msg(True).lateralDelay.lateralDelay == pytest.approx(0.58)


def test_publish_cap_kill_switch_and_stock():
  kw = {**lagd.lagd_profile(cp()), "publish_max": None}
  e = lagd.LateralLagEstimator(cp(), 0.05, **kw); e.reset(0.90, lagd.BLOCK_NUM_NEEDED + 2)
  assert e.get_msg(True).lateralDelay.lateralDelay == pytest.approx(0.90)       # up to the search end
  s = lagd.LateralLagEstimator(cp(), 0.05); s.reset(0.90, lagd.BLOCK_NUM_NEEDED + 2)
  assert s.publish_max == lagd.MAX_LAG and s.get_msg(True).lateralDelay.lateralDelay == pytest.approx(lagd.MAX_LAG)


def test_seed_is_inside_the_cap():
  assert lagd.LAGD_SEED_S <= lagd.LAGD_PUBLISH_MAX_S


# ---------------------------------------------------------------- 67b
def lead_out(delay, k_far=0.012, t_step=0.8):
  """A plan step far larger than the budget, inside the lead window for either delay: the output is budget-bound."""
  cd.LOOKAHEAD_DELAY_EXTRA_S = [0.3] * 5
  s = settled(20.0); s._model_lat_delay = delay
  return call(s, model(0.002, k_far, t_step=t_step)), s


def test_budget_does_not_grow_with_the_model_delay():
  base, _ = lead_out(0.3)
  seeded, _ = lead_out(0.65)
  assert seeded == pytest.approx(base, rel=1e-6)               # same clip whatever delay the model is told


def test_budget_kill_switch_restores_the_looser_clip():
  cd.LOOKAHEAD_BUDGET_EXCLUDES_SHIFT = False
  base, _ = lead_out(0.3)
  seeded, _ = lead_out(0.65)
  lead_base, lead_seeded = base - 0.002, seeded - 0.002
  assert lead_seeded > lead_base * 1.2                         # the pre-67b behaviour: ~36 % more lead allowed


def test_untold_delay_is_unchanged():
  a, _ = lead_out(0.3)
  cd.LOOKAHEAD_BUDGET_EXCLUDES_SHIFT = False
  b, _ = lead_out(0.3)
  assert a == pytest.approx(b, abs=1e-12)


# ---------------------------------------------------------------- 67c
def test_stock_runner_uses_the_toggle_aware_delay():
  src = (pathlib.Path(cd.__file__).parents[1] / "modeld" / "modeld.py").read_text()
  assert re.search(r"^LAT_DELAY_FOLLOWS_TOGGLE = True$", src, re.M)
  assert "model.lat_delay = get_lat_delay(params, sm[\"lateralDelay\"].lateralDelay)" in src
  assert "lat_delay = (model.lat_delay if LAT_DELAY_FOLLOWS_TOGGLE else sm[\"lateralDelay\"].lateralDelay) + LAT_SMOOTH_SECONDS" in src
