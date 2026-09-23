"""Phase 43 (i6nv3 0x26-0x2c, report §27).
  43a  the 42a city fast-descent floor applies to a debounced press only; the driver_tq >= 160 arm is back on the 35a
       schedule (no floor below 40 km/h) — a resting hand crossing 160 driver-Nm pumped authority and shook the wheel.
  43b  driver hard acceleration (accelerator pressed, aEgo 1.5 -> 3.0 m/s^2, >= 20 km/h) scales the authority ceiling
       1.0 -> 0.5.
"""
import numpy as np

from phase_tests.harness import Sim
from opendbc.car.hyundai.values import CarControllerParams as P


def settle(sim, v, n=300, tq=100.0, **kw):
  for _ in range(n):
    sim.step(v=v, tq=tq, wheel=0.0, cmd=0.0, **kw)


class TestPhase43aNoRestingHandPumping:
  def _pulses(self, v_kph, table=None):
    """Resting hand (driver_tq ~30) with 30 ms bumps to ~200 driver-Nm twice a second — above the 160 torque arm, never
    a debounced press. Count frames where authority falls >= 5 quanta. (The harness runs < 40 km/h as passthrough, so
    the city case is exercised at 45 km/h, where the 35a torque-arm floor is still only 0.0075/frame.)"""
    old = P.ACIGAIN_GRIP_RATE_DN_TQ_ARM_FLOOR_V
    if table is not None:
      P.ACIGAIN_GRIP_RATE_DN_TQ_ARM_FLOOR_V = table
    try:
      sim = Sim(); v = v_kph / 3.6
      settle(sim, v, tq=0.0)
      g = []
      for i in range(400):
        comp = sim.cc.hold_comp_last
        bump = (i % 50) < 3
        sim.step(v=v, tq=comp + (200.0 if bump else 30.0), wheel=0.3 * np.sin(i / 7.0), cmd=np.sin(i / 9.0))
        g.append(sim.tx_gain())
      assert not sim.cc.driver_pressed
      g = np.array(g)
      return int(np.sum(np.diff(g) <= -0.02)), float(g.mean())
    finally:
      P.ACIGAIN_GRIP_RATE_DN_TQ_ARM_FLOOR_V = old

  def test_city_resting_hand_bumps_do_not_pump(self):
    fast, _ = self._pulses(45.0)
    assert fast == 0

  def test_regression_reproduces_with_42a_table_on_the_torque_arm(self):
    fast, _ = self._pulses(45.0, table=list(P.ACIGAIN_GRIP_RATE_DN_FLOOR_V))
    assert fast >= 10                                     # the test bites: 42a behaviour pumped here

  def test_highway_torque_arm_unchanged_from_35a(self):
    assert P.ACIGAIN_GRIP_RATE_DN_TQ_ARM_FLOOR_V == [0.0, 0.03]
    assert P.ACIGAIN_GRIP_RATE_DN_FLOOR_V == [0.03, 0.03]  # debounced press keeps 42a

  def test_real_press_still_fast_at_city_speed(self):
    sim = Sim(); v = 35.0 / 3.6
    settle(sim, v)
    g = []; wheel = 0.0
    for _ in range(60):
      wheel += 0.4
      sim.step(v=v, tq=450.0, wheel=wheel, cmd=0.0, pressed=True)
      g.append(sim.tx_gain())
    assert np.argmax(np.array(g) <= 0.20) / 100 <= 0.25


class TestPhase43bAccelYield:
  def _gain(self, v_kph, a, gas, frames=300):
    sim = Sim(); v = v_kph / 3.6
    settle(sim, v, tq=0.0)
    for _ in range(frames):
      sim.step(v=v, tq=0.0, wheel=0.0, cmd=0.0, a_ego=a, gas=gas)
    return sim.tx_gain()

  def test_hard_accel_with_pedal_halves_ceiling(self):
    cruise = self._gain(70.0, 0.0, False)
    hard = self._gain(70.0, 3.0, True)
    assert cruise >= 0.8
    assert hard <= 0.5 * cruise + 0.02

  def test_mild_accel_untouched(self):
    assert abs(self._gain(70.0, 1.2, True) - self._gain(70.0, 0.0, False)) <= 0.01

  def test_no_pedal_no_yield(self):
    """op-longitudinal / downhill acceleration without the driver's foot: authority unchanged."""
    assert abs(self._gain(70.0, 3.0, False) - self._gain(70.0, 0.0, False)) <= 0.01

  def test_low_speed_launch_untouched(self):
    assert abs(self._gain(15.0, 3.0, True) - self._gain(15.0, 0.0, False)) <= 0.01

  def test_recovers_after_pedal_release(self):
    sim = Sim(); v = 70.0 / 3.6
    settle(sim, v, tq=0.0)
    for _ in range(300):
      sim.step(v=v, tq=0.0, wheel=0.0, cmd=0.0, a_ego=3.0, gas=True)
    low = sim.tx_gain()
    for _ in range(300):
      sim.step(v=v, tq=0.0, wheel=0.0, cmd=0.0)
    assert sim.tx_gain() >= low + 0.3

  def test_kill(self):
    old = P.ACCEL_YIELD_SCALE_V
    try:
      P.ACCEL_YIELD_SCALE_V = [1.0, 1.0]
      g = self._gain(70.0, 3.0, True)
    finally:
      P.ACCEL_YIELD_SCALE_V = old
    assert g >= 0.8
