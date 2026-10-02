"""Phase 44a (i6nv3 0x2e-0x38, report §29): the EPS pressed flag (raw >= 350 Nm, 5-frame debounce) also opens the 42a
fast-descent floor. A 350-400 Nm lane-change push trips the EPS flag but stays under the hold-compensated
driver_pressed threshold, so before 44a authority fell at the slow table rate while the driver steered.
"""
import numpy as np

from phase_tests.harness import Sim
from opendbc.car.hyundai.values import CarControllerParams as P


def push(v_kph, raw_tq, eps_flag, frames=80, flag=None):
  """Hands-off cruise, then a push of raw_tq with the EPS pressed flag as given (wheel turning 15 deg/s)."""
  old = P.GRIP_FLOOR_EPS_PRESSED
  if flag is not None:
    P.GRIP_FLOOR_EPS_PRESSED = flag
  try:
    sim = Sim(); v = v_kph / 3.6
    for _ in range(300):
      sim.step(v=v, tq=0.0, wheel=0.0, cmd=0.0)
    g0 = sim.tx_gain(); g = []; wheel = 0.0
    for _ in range(frames):
      wheel += 0.15
      sim.step(v=v, tq=raw_tq, wheel=wheel, cmd=0.0, pressed=eps_flag)
      g.append(sim.tx_gain())
    return g0, np.array(g), sim
  finally:
    P.GRIP_FLOOR_EPS_PRESSED = old


def t_below(g, level=0.20):
  i = np.where(g <= level)[0]
  return i[0] / 100 if len(i) else None


class TestPhase44a:
  # 330 raw in the harness = the on-road 350-400 Nm band: EPS flag set, hold-compensated driver_pressed not tripped
  # (the harness hold_comp is ~190 vs ~120-140 on the car, which shifts the raw value, not the mechanism)
  def test_sub_400_push_with_eps_flag_yields_fast(self):
    g0, g, sim = push(50.0, 330.0, True)
    assert g0 >= 0.6
    assert not sim.cc.driver_pressed                       # the hold-compensated press never trips here
    t = t_below(g)
    assert t is not None and t <= 0.25                      # 0.49 s with the kill switch (on-road p50 0.47 s)

  def test_kill_restores_slow_descent(self):
    _, g_new, _ = push(50.0, 330.0, True)
    _, g_old, _ = push(50.0, 330.0, True, flag=False)
    t_new, t_old = t_below(g_new), t_below(g_old)
    assert t_old is None or t_old >= t_new + 0.15           # the change is what makes it fast

  def test_no_eps_flag_no_change(self):
    """Same torque without the EPS flag (e.g. a resting hand never reaches 350 raw): identical to the kill switch."""
    _, a, _ = push(50.0, 300.0, False)
    _, b, _ = push(50.0, 300.0, False, flag=False)
    assert np.allclose(a, b)

  def test_short_real_press_costs_a_dip_not_a_slide(self):
    """A debounced EPS press (>= 5 frames) every 2 s on a resting hand — far more often than on the road (< 0.1 s
    flags 0.49 /min) — costs a bounded dip per press and authority recovers between presses (no ratchet)."""
    def run(flag):
      old = P.GRIP_FLOOR_EPS_PRESSED; P.GRIP_FLOOR_EPS_PRESSED = flag
      try:
        sim = Sim(); v = 50.0 / 3.6
        for _ in range(300):
          sim.step(v=v, tq=0.0, wheel=0.0, cmd=0.0)
        g = []
        for i in range(600):
          on = (i % 200) < 5
          sim.step(v=v, tq=(360.0 if on else 120.0), wheel=0.2 * np.sin(i / 7.0), cmd=np.sin(i / 9.0), pressed=on)
          g.append(sim.tx_gain())
        return np.array(g)
      finally:
        P.GRIP_FLOOR_EPS_PRESSED = old
    g = run(True)
    for k in range(3):                                      # just before each press vs 1.9 s later
      assert g[k * 200 + 190] >= g[k * 200] - 0.05
    assert g.min() >= 0.35

  def test_shipped_default(self):
    assert P.GRIP_FLOOR_EPS_PRESSED is True
