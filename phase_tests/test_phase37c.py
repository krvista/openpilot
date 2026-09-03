"""Phase 37c: yield-curve start by speed (30 Nm city -> 50 Nm at >= 60 km/h; rain 30)."""
from phase_tests.harness import Sim, run_signal
from opendbc.car.hyundai.values import CarControllerParams as P


def settle(sim, n=120, **kw):
  kw.setdefault('v', 15.0); kw.setdefault('wheel', 0.0); kw.setdefault('cmd', 0.0); kw.setdefault('tq', 0.0)
  return run_signal(sim, n, **kw)


def _resting(v, dtq, kill=False, rain=False, n=400):
  """steady frames with a resting hand: raw torque = hold comp + dtq (driver domain)."""
  old = P.ACIGAIN_GRIP_START_V
  try:
    if kill:
      P.ACIGAIN_GRIP_START_V = [30.0, 30.0]
    sim = Sim(); settle(sim)
    if rain:
      run_signal(sim, P.RAIN_WIPER_ON_FRAMES + 2500, v=v, wiper=True)
    run_signal(sim, 200, v=v, wheel=0.0, cmd=0.0, tq=0.0, wiper=rain)
    comp = sim.s.hold_comp_last
    tr = run_signal(sim, n, v=v, wheel=0.0, cmd=0.0, tq=comp + dtq, wiper=rain)
  finally:
    P.ACIGAIN_GRIP_START_V = old
  return sim, tr['gain'][-1]


class TestPhase37cGripStart:
  def test_resting_hand_keeps_assist_at_72kph(self):
    _, g = _resting(v=20.0, dtq=40.0)
    _, g_kill = _resting(v=20.0, dtq=40.0, kill=True)
    assert g > g_kill + 0.05, (g, g_kill)                # 40 Nm: above the old start, below the new one (0.83 vs 0.76)
    _, g0 = _resting(v=20.0, dtq=0.0)
    assert abs(g - g0) <= 0.004 + 1e-9, (g, g0)          # ... i.e. full hands-off assist

  def test_firm_hand_unchanged_at_72kph(self):
    _, g = _resting(v=20.0, dtq=150.0)                   # well past the start in both tables
    _, g_kill = _resting(v=20.0, dtq=150.0, kill=True)
    assert abs(g - g_kill) <= 0.004 + 1e-9 and g <= 0.3, (g, g_kill)
    sim = Sim(); settle(sim)
    run_signal(sim, 200, v=20.0, wheel=0.0, cmd=0.0, tq=0.0)
    tr = run_signal(sim, 300, v=20.0, wheel=0.0, cmd=0.0, tq=460.0)   # pressed grip: 35a floor path
    assert tr['gain'][-1] <= 0.08, tr['gain'][-1]

  def test_city_unchanged_at_36kph(self):
    _, g = _resting(v=10.0, dtq=40.0)
    _, g_kill = _resting(v=10.0, dtq=40.0, kill=True)
    assert abs(g - g_kill) <= 0.004 + 1e-9, (g, g_kill)

  def test_ramp_is_monotone_between_40_and_60kph(self):
    gs = [_resting(v=v, dtq=40.0)[1] for v in (11.1, 12.5, 13.9, 15.3, 16.7)]
    assert all(b >= a - 0.004 - 1e-9 for a, b in zip(gs, gs[1:])), gs

  def test_rain_pins_start_at_30(self):
    _, g_rain = _resting(v=20.0, dtq=40.0, rain=True)
    _, g_dry = _resting(v=20.0, dtq=40.0)
    assert g_rain < g_dry - 0.05, (g_rain, g_dry)
    _, g_kill = _resting(v=20.0, dtq=40.0, kill=True)
    assert abs(g_rain - g_kill) <= 0.02, (g_rain, g_kill)   # rain start 30 == old dry start
