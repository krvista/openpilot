"""Phase 51 (i6nv3 route 45 Namsan tunnel, report §48): a brief light touch that silences the car's hands-on-detection
(0.05-0.09 s, 98-142 raw Nm, no EPS press) is not a grip. The torque-only yield path is debounced 0.3 s; a press, a push
above HOD_TOUCH_MAX_NM and a resting hand (continuous torque) behave as before."""
import numpy as np
import pytest

from phase_tests.harness import Sim
from opendbc.car.hyundai.values import CarControllerParams as P
import opendbc.car.hyundai.carcontroller as ccmod

V = 15.0   # m/s, 54 km/h: hands-off ceiling ~0.8, yield start ~53 driver-Nm


def settle(sim, n=300):
  for _ in range(n):
    sim.step(v=V, tq=0.0, wheel=0.0, cmd=0.0)
  return sim.tx_gain()


def run_touch(tq, frames, after=100, debounce=None):
  old = P.HOD_TOUCH_DEBOUNCE_FRAMES
  try:
    if debounce is not None:
      P.HOD_TOUCH_DEBOUNCE_FRAMES = debounce
    sim = Sim(); g0 = settle(sim)
    g, flags = [], []
    for i in range(frames + after):
      sim.step(v=V, tq=(tq if i < frames else 0.0), wheel=0.0, cmd=0.0)
      g.append(sim.tx_gain()); flags.append(sim.cc.steer_debug_flags)
    return g0, np.array(g), flags, sim
  finally:
    P.HOD_TOUCH_DEBOUNCE_FRAMES = old


class TestHodTouchExemption:
  def test_brief_light_touch_does_not_move_the_gain(self):
    g0, g, flags, sim = run_touch(150.0, frames=9)          # 0.09 s at 150 raw (hold comp ~74 at 54 km/h -> driver ~76)
    assert g.min() >= g0 - 0.005, (g0, g.min())
    assert all(f & ccmod.SF_HOD_TOUCH for f in flags[:9]) and not any(f & ccmod.SF_HOD_TOUCH for f in flags[9:])
    assert not sim.cc.driver_pressed

  def test_kill_switch_restores_the_dip(self):
    g0, g, flags, _ = run_touch(150.0, frames=9, debounce=0)
    assert g.min() < g0 - 0.02                                # pre-51: the same touch dipped the authority
    assert not any(f & ccmod.SF_HOD_TOUCH for f in flags)

  def test_sustained_torque_still_yields_after_the_debounce(self):
    g0, g, flags, _ = run_touch(150.0, frames=80)
    assert g[:30].min() >= g0 - 0.005                         # exempt through 0.3 s
    assert g[79] < g0 - 0.02                                  # then the yield proceeds
    assert all(f & ccmod.SF_HOD_TOUCH for f in flags[:30]) and not (flags[31] & ccmod.SF_HOD_TOUCH)
    _, gk, _, _ = run_touch(150.0, frames=80, debounce=0)
    assert gk[29] < g[29]                                     # pre-51 had already descended by 0.3 s
    assert gk[79] == pytest.approx(g[79 + 0], abs=0.15)       # same destination, reached later (rate-limited descent)

  def test_eps_press_is_not_exempt(self):
    # raw 200 (driver ~126 < cap) with the EPS flag forced: the press term alone must cancel the exemption
    def run(debounce):
      old = P.HOD_TOUCH_DEBOUNCE_FRAMES
      try:
        P.HOD_TOUCH_DEBOUNCE_FRAMES = debounce
        sim = Sim(); g0 = settle(sim); g, flags = [], []
        for i in range(30):
          sim.step(v=V, tq=200.0, wheel=0.0, cmd=0.0, pressed=True); g.append(sim.tx_gain()); flags.append(sim.cc.steer_debug_flags)
        return g0, np.array(g), flags
      finally:
        P.HOD_TOUCH_DEBOUNCE_FRAMES = old
    g0, g, flags = run(30); _, gk, _ = run(0)
    assert g[29] < g0 - 0.02 and np.allclose(g, gk)           # the 42/44a fast yield is untouched
    assert not any(f & ccmod.SF_HOD_TOUCH for f in flags)

  def test_driver_pressed_cancels_the_exemption(self):
    # raw 330 -> driver ~256 > 230: driver_pressed after its debounce ends the exemption; the yield then matches pre-51
    g0, g, flags, _ = run_touch(330.0, frames=40)
    _, gk, _, _ = run_touch(330.0, frames=40, debounce=0)
    assert not any(f & ccmod.SF_HOD_TOUCH for f in flags[8:40])
    assert g[39] < g0 - 0.05 and g[39] == pytest.approx(gk[39], abs=0.03)

  def test_torque_arm_band_is_not_exempt(self):
    # review: a steady push that arms the 35a torque-arm floor (>= 160 driver-Nm) must yield at once, also at highway speed
    def run(debounce):
      old = P.HOD_TOUCH_DEBOUNCE_FRAMES
      try:
        P.HOD_TOUCH_DEBOUNCE_FRAMES = debounce
        sim = Sim()
        for _ in range(300): sim.step(v=25.0, tq=0.0, wheel=0.0, cmd=0.0)
        g = [sim.tx_gain()]
        for _ in range(30): sim.step(v=25.0, tq=280.0, wheel=0.0, cmd=0.0); g.append(sim.tx_gain())   # driver ~218 at comp ~62
        return np.array(g)
      finally:
        P.HOD_TOUCH_DEBOUNCE_FRAMES = old
    g, gk = run(30), run(0)
    assert np.allclose(g, gk) and g[-1] < g[0] - 0.05

  def test_refractory_blocks_periodic_sawing(self):
    # review: 2 Hz sawing (25 on / 25 off at ~driver 100) must not ride the exemption; after the first touch the
    # refractory (1 s quiet) is never met, so the yield proceeds as pre-51 from the second cycle on
    def run(debounce):
      old = P.HOD_TOUCH_DEBOUNCE_FRAMES
      try:
        P.HOD_TOUCH_DEBOUNCE_FRAMES = debounce
        sim = Sim(); settle(sim); g = []; flags = []
        for i in range(400):
          sim.step(v=V, tq=(180.0 if (i % 50) < 25 else 0.0), wheel=0.0, cmd=0.0); g.append(sim.tx_gain()); flags.append(sim.cc.steer_debug_flags)
        return np.array(g), flags
      finally:
        P.HOD_TOUCH_DEBOUNCE_FRAMES = old
    g, flags = run(30); gk, _ = run(0)
    assert sum(1 for f in flags if f & ccmod.SF_HOD_TOUCH) <= 25           # only the first touch is exempt
    assert g[399] == pytest.approx(gk[399], abs=0.03)                       # same settled authority as pre-51

  def test_brief_push_above_max_is_not_exempt(self):
    # a push above the cap takes the pre-51 path unchanged (bit-identical to the kill run) and is never flagged
    # 345 raw: below the EPS press (350), above the cap even after the hold compensation
    g0, g, flags, _ = run_touch(345.0, frames=9)
    _, gk, _, _ = run_touch(345.0, frames=9, debounce=0)
    assert g.min() < g0 - 0.01                                # the push still yields at once
    assert np.allclose(g, gk)
    assert not any(f & ccmod.SF_HOD_TOUCH for f in flags)

  def test_resting_hand_unchanged_after_the_debounce(self):
    _, g, _, _ = run_touch(120.0, frames=300, after=0)
    _, gk, _, _ = run_touch(120.0, frames=300, after=0, debounce=0)
    assert g[299] == pytest.approx(gk[299], abs=0.02)         # same settled authority with a resting hand

  def test_touch_counter_resets_between_touches(self):
    sim = Sim(); g0 = settle(sim)
    for _ in range(3):                                        # three HOD touches 1.5 s apart (past the 1 s refractory)
      for i in range(159):
        sim.step(v=V, tq=(150.0 if i < 9 else 0.0), wheel=0.0, cmd=0.0)
        assert sim.tx_gain() >= g0 - 0.005
    assert sim.cc.hod_touch_frames == 0

  def test_defaults(self):
    assert P.HOD_TOUCH_DEBOUNCE_FRAMES == 30 and P.HOD_TOUCH_REFRACTORY_FRAMES == 100
    assert P.HOD_TOUCH_MAX_NM == pytest.approx(P.ACIGAIN_GRIP_RATE_DN_GATE_NM)
    assert ccmod.SF_HOD_TOUCH == 1 << 28
