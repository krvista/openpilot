"""Phase 67 (report §88-89).
67a: lagd publishes at most its cap to modeld on the CCNC profile (publish_max; LAGD_SOLO_PUBLISH_MAX_S = 0.50 s on the
only profile left since Phase 89); the learned value stays in lateralDelayEstimate / the blocks / the cache.
67b: (the orient-lead jerk budget; removed with the lead in Phase 89.)
67c: the stock modeld runner feeds the model the toggle-aware delay (model.lat_delay), like modeld_v2."""
import pathlib
import re
import types

import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd
import openpilot.selfdrive.locationd.lagd as lagd


def cp():
  return types.SimpleNamespace(brand="hyundai", steerControlType="angle", steerActuatorDelay=0.1, carFingerprint="X")


# ---------------------------------------------------------------- 67a
def test_published_delay_is_capped():
  cap = lagd.LAGD_SOLO_PUBLISH_MAX_S
  e = lagd.LateralLagEstimator(cp(), 0.05, **lagd.lagd_profile(cp()))
  e.reset(cap + 0.1, lagd.BLOCK_NUM_NEEDED + 2)
  m = e.get_msg(True).lateralDelay
  assert str(m.status) == "estimated"
  assert m.lateralDelay == pytest.approx(cap)                  # what modeld / controlsd read
  assert m.lateralDelayEstimate == pytest.approx(cap + 0.1)    # what is learned, logged and cached


def test_stock_range_cap_still_selectable():
  kw = {**lagd.lagd_profile(cp()), "publish_max": lagd.MAX_LAG}
  e = lagd.LateralLagEstimator(cp(), 0.05, **kw); e.reset(0.69, lagd.BLOCK_NUM_NEEDED + 2)
  m = e.get_msg(True).lateralDelay
  assert m.lateralDelay == pytest.approx(0.65) and m.lateralDelayEstimate == pytest.approx(0.69)


def test_learned_value_below_the_cap_is_published_as_is():
  e = lagd.LateralLagEstimator(cp(), 0.05, **lagd.lagd_profile(cp()))
  e.reset(0.41, lagd.BLOCK_NUM_NEEDED + 2)
  assert e.get_msg(True).lateralDelay.lateralDelay == pytest.approx(0.41)


def test_publish_cap_kill_switch_and_stock():
  kw = {**lagd.lagd_profile(cp()), "publish_max": None, "max_lag": 1.0}
  e = lagd.LateralLagEstimator(cp(), 0.05, **kw); e.reset(0.90, lagd.BLOCK_NUM_NEEDED + 2)
  assert e.get_msg(True).lateralDelay.lateralDelay == pytest.approx(0.90)       # up to the search end
  s = lagd.LateralLagEstimator(cp(), 0.05); s.reset(0.90, lagd.BLOCK_NUM_NEEDED + 2)
  assert s.publish_max == lagd.MAX_LAG and s.get_msg(True).lateralDelay.lateralDelay == pytest.approx(lagd.MAX_LAG)


def test_seed_is_inside_the_cap():
  assert lagd.LAGD_SOLO_SEED_S <= lagd.LAGD_SOLO_PUBLISH_MAX_S


# ---------------------------------------------------------------- 67c
def test_stock_runner_uses_the_toggle_aware_delay():
  src = (pathlib.Path(cd.__file__).parents[1] / "modeld" / "modeld.py").read_text()
  assert re.search(r"^LAT_DELAY_FOLLOWS_TOGGLE = True$", src, re.M)
  assert "model.lat_delay = get_lat_delay(params, sm[\"lateralDelay\"].lateralDelay)" in src
  assert "lat_delay = (model.lat_delay if LAT_DELAY_FOLLOWS_TOGGLE else sm[\"lateralDelay\"].lateralDelay) + LAT_SMOOTH_SECONDS" in src
