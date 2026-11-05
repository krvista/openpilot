"""Phase 59b (report §69): the compensator's own share delta = request - reference leaks (LEAK_TAU_S), stops integrating the
feedback while the wheel does not follow (FREEZE_*) and is bounded per speed (GMAX_V). On the 31520b9 drives the old
compensator wound up to its 6 deg clamp against a wheel that did not move (light hand / sub-degree requests ignored)."""
import numpy as np
import pytest

from phase_tests.harness import Sim
from opendbc.car.hyundai.values import CarControllerParams as P
import opendbc.car.hyundai.carcontroller as ccmod
from phase_tests.test_phase59 import run, ramp_to

USE_MDPS_COMP = True


@pytest.fixture(autouse=True)
def _compensator_on():
  # Phase 61 ships the compensator off (MDPS_COMP_LAMBDA = 0); these tests cover it in its enabled configuration
  old = P.MDPS_COMP_LAMBDA
  P.MDPS_COMP_LAMBDA = 2.0
  yield
  P.MDPS_COMP_LAMBDA = old


def stuck(cmd, n=600, v=60 / 3.6, wheel=0.0, **ovr):
  saved = {k: getattr(P, k) for k in ovr}
  try:
    for k, val in ovr.items():
      setattr(P, k, val)
    sim = Sim(); tx = []; fl = []
    for i in range(n):
      sim.step(v=v, tq=0.0, wheel=wheel, cmd=cmd(i))
      tx.append(sim.tx_angle()); fl.append(sim.cc.steer_debug_flags)
    return np.array(tx), fl, sim
  finally:
    for k, val in saved.items():
      setattr(P, k, val)


OLD = dict(MDPS_COMP_LEAK_TAU_S=1e9, MDPS_COMP_FREEZE=False, MDPS_COMP_GMAX_V=[6.0] * 5)


class TestCompensatorWindup:
  def test_old_59_winds_up_to_the_clamp_against_a_still_wheel(self):
    tx, _, _ = stuck(ramp_to(2.0), **OLD)
    assert tx[-1] - 2.0 >= 5.5                            # the 31520b9 behaviour this phase removes

  def test_freeze_and_leak_keep_the_request_near_the_command(self):
    tx, _, _ = stuck(ramp_to(2.0))
    assert abs(tx[-1] - 2.0) <= P.MDPS_COMP_FREEZE_MIN_DEG + 0.6

  def test_gmax_per_speed(self):
    for v_kph in (35.0, 62.5, 100.0):
      tx, _, _ = stuck(ramp_to(10.0), v=v_kph / 3.6, MDPS_COMP_FREEZE=False, MDPS_COMP_LEAK_TAU_S=1e9)
      g = float(np.interp(v_kph, P.MDPS_COMP_SPEEDS_KPH, P.MDPS_COMP_GMAX_V))
      assert abs(tx[-1] - 10.0) <= g + 0.2, (v_kph, tx[-1])

  def test_leak_returns_delta_toward_zero(self):
    # freeze off: wind the share up against a still wheel, then let the wheel reach the reference — the share must decay
    saved = (P.MDPS_COMP_FREEZE,)
    try:
      P.MDPS_COMP_FREEZE = False
      sim = Sim(); tx = []
      for i in range(900):
        sim.step(v=60 / 3.6, tq=0.0, wheel=(0.0 if i < 400 else 2.0), cmd=2.0)
        tx.append(sim.tx_angle())
    finally:
      (P.MDPS_COMP_FREEZE,) = saved
    d0 = tx[399] - 2.0; d1 = tx[-1] - 2.0
    assert d0 > 2.0                                          # wound up while the wheel stood still
    assert abs(d1) < 0.4 * d0, (d0, d1)                      # ~2.5 tau later most of it has leaked away

  def test_a_following_wheel_still_reaches_the_command(self):
    w, tx, fl, _ = run(ramp_to(6.0), 800)
    assert w[-1] >= 5.2, w[-1]
