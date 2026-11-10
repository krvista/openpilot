"""Phase 68 (report §90): the total steering timing follows the delay the model is told instead of being pinned at 0.6 s.
controlsd's delay shift only fills what the model delay lacks (never negative): total = max(model delay, 0.3 + E(v)).
lagd publishes the learned value up to LAGD_PUBLISH_MAX_S = 0.80 s (0.69 s on this car)."""
import types

import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.selfdrive.controls.controlsd as cd
import openpilot.selfdrive.locationd.lagd as lagd
from phase_tests.test_phase55 import call, settled
from phase_tests.test_phase60 import ramp_plan


@pytest.fixture(autouse=True)
def _restore():
  saved = {nm: getattr(cd, nm) for nm in ("LOOKAHEAD_DELAY_EXTRA_S", "LOOKAHEAD_DELAY_PIN_TOTAL")}
  yield
  for nm, val in saved.items():
    setattr(cd, nm, val)


def eff(delay, e=0.3):
  cd.LOOKAHEAD_DELAY_EXTRA_S = [e] * 5
  s = settled(20.0); s._model_lat_delay = delay
  out = call(s, ramp_plan(0.001, 0.005))
  return s._delay_extra_eff, s._delay_match, out


def total(delay, e=0.3):
  return delay + eff(delay, e)[0]


def test_shipped():
  assert cd.LOOKAHEAD_DELAY_PIN_TOTAL is False
  assert lagd.LAGD_PUBLISH_MAX_S == 0.80 and lagd.LAGD_SEED_S == 0.65


@pytest.mark.parametrize("delay, want", [(0.30, 0.60), (0.45, 0.60), (0.60, 0.60), (0.65, 0.65), (0.69, 0.69), (0.80, 0.80)])
def test_total_is_the_larger_of_model_delay_and_phase60(delay, want):
  assert total(delay) == pytest.approx(want, abs=1e-9)


def test_shift_is_never_negative_and_adds_nothing_above_0_6():
  e, dm, _ = eff(0.69)
  assert e == 0.0 and dm == 0.0                       # was -0.09 s (steering later than the learned delay)


def test_below_0_6_is_unchanged():
  for d in (0.30, 0.45, 0.57):
    cd.LOOKAHEAD_DELAY_PIN_TOTAL = False; a = eff(d)
    cd.LOOKAHEAD_DELAY_PIN_TOTAL = True; b = eff(d)
    assert a[0] == pytest.approx(b[0], abs=1e-12) and a[2] == pytest.approx(b[2], abs=1e-12)


def test_kill_switch_pins_the_total_again():
  cd.LOOKAHEAD_DELAY_PIN_TOTAL = True
  assert eff(0.69)[0] == pytest.approx(-0.09, abs=1e-9)
  assert total(0.69) == pytest.approx(0.60, abs=1e-9)


def test_phase60_off_follows_the_model_delay_only():
  assert total(0.69, e=0.0) == pytest.approx(0.69, abs=1e-9)
  assert total(0.30, e=0.0) == pytest.approx(0.30, abs=1e-9)


def test_learned_0_69_is_published():
  cp = types.SimpleNamespace(brand="hyundai", steerControlType="angle", steerActuatorDelay=0.1, carFingerprint="X")
  e = lagd.LateralLagEstimator(cp, 0.05, **lagd.lagd_profile(cp))
  e.reset(0.69, lagd.BLOCK_NUM_NEEDED + 2)
  assert e.get_msg(True).lateralDelay.lateralDelay == pytest.approx(0.69)
