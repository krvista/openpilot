"""Phase 69 (report §91-92).
69a: total steering timing 0.8 s up to 70 km/h (LOOKAHEAD_DELAY_EXTRA_S 0.5 on the first three steps, third breakpoint at
     70 km/h), the learned delay from 80 km/h — a wheel at rest takes ~0.9 s to start whatever the request rate.
69b: corner-exit lead, above 70 km/h — while the command unwinds and the wheel is measured still out beyond it, the
     request moves EXIT_LEAD_GAIN x the unwinding excess further toward centre (capped by the speed cap — 0 up to 70 km/h —,
     by how far the wheel is behind and by EXIT_LEAD_MAX_FRAC of the command).
69c: curve trim runs up to CURVE_TRIM_FAST_RATE_DPS for a large residual up to 70 km/h, 2 deg/s otherwise."""
import numpy as np
import pytest

from phase_tests.harness import Sim
from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)
from phase_tests.test_phase63 import params, KILL_63A
from opendbc.car.hyundai.values import CarControllerParams as P
import opendbc.car.hyundai.carcontroller as ccmod
import openpilot.selfdrive.controls.controlsd as cd
from phase_tests.test_phase55 import call, settled
from phase_tests.test_phase60 import ramp_plan

KPH = 1 / 3.6


# ---------------------------------------------------------------- 69a controlsd
@pytest.fixture
def _cd_restore():
  saved = {nm: getattr(cd, nm) for nm in ("LOOKAHEAD_DELAY_EXTRA_S", "LOOKAHEAD_DELAY_SPEEDS_MS")}
  yield
  for nm, val in saved.items():
    setattr(cd, nm, val)


def total(v_kph, delay):
  s = settled(v_kph * KPH); s._model_lat_delay = delay
  call(s, ramp_plan(0.001, 0.005, v=v_kph * KPH), v=v_kph * KPH)
  return delay + s._delay_extra_eff


def test_shipped_table():
  assert cd.LOOKAHEAD_DELAY_EXTRA_S == [0.5, 0.5, 0.5, 0.3, 0.3]
  assert cd.LOOKAHEAD_DELAY_SPEEDS_MS == pytest.approx([35 * KPH, 47.5 * KPH, 70 * KPH, 80 * KPH, 100 * KPH])


@pytest.mark.parametrize("v_kph, delay, want", [
  (40, 0.69, 0.80), (60, 0.69, 0.80), (70, 0.69, 0.80),      # up to 70 km/h: 0.8 s whatever the model is told
  (60, 0.65, 0.80), (60, 0.30, 0.80),
  (75, 0.69, 0.70),                                          # taper: 0.3 + 0.4 = 0.70
  (80, 0.69, 0.69), (100, 0.69, 0.69),                       # from 80 km/h: the learned delay itself
  (100, 0.30, 0.60),                                         # lagd off / unlearned at speed: Phase 60's 0.6 s
])
def test_total_timing_by_speed(_cd_restore, v_kph, delay, want):
  assert total(v_kph, delay) == pytest.approx(want, abs=1e-6)


def test_kill_restores_the_learned_delay_everywhere(_cd_restore):
  cd.LOOKAHEAD_DELAY_EXTRA_S = [0.3] * 5
  assert total(50, 0.69) == pytest.approx(0.69, abs=1e-6) and total(50, 0.30) == pytest.approx(0.60, abs=1e-6)


# ---------------------------------------------------------------- carcontroller runner
def run(cmds, v, wheel_fn, tq=0.0, blinker=False, **kw):
  """cmds: per-frame command; wheel_fn(i, cmds, tx_prev) -> wheel. Returns TX, exit-lead offsets, entry offsets, trim, flags."""
  with params(**kw):
    sim = Sim(); tx_prev = 0.0
    w0 = wheel_fn(0, cmds, 0.0)
    for _ in range(300):
      sim.step(v=v, tq=0.0, wheel=w0, cmd=float(cmds[0]))
    tx, xl, eb, tr, fl = [], [], [], [], []
    for i, c in enumerate(cmds):
      sim.step(v=v, tq=tq, wheel=wheel_fn(i, cmds, tx_prev), cmd=float(c), blinker=blinker)
      tx_prev = sim.tx_angle()
      tx.append(tx_prev); xl.append(sim.cc.xl_off); eb.append(sim.cc.eb_off); tr.append(sim.cc.curve_trim); fl.append(sim.cc.steer_debug_flags)
    return np.array(tx), np.array(xl), np.array(eb), np.array(tr), fl


def exit_cmd(hi=10.0, lo=2.0, rate=6.0, pre=600, post=300):
  n = int((hi - lo) / rate * 100)
  return np.concatenate([np.full(pre, hi), np.linspace(hi, lo, n), np.full(post, lo)])


def lagging(delay_frames=80):
  return lambda i, cmds, tx: float(cmds[max(i - delay_frames, 0)])      # the wheel follows the command 0.8 s late


NO_TRIM = dict(CURVE_TRIM_RATE_DPS=0.0, ENTRY_BOOST_GAIN=0.0)            # isolate the exit lead
V = 85 * KPH                                                              # the exit lead only runs above 70 km/h


# ---------------------------------------------------------------- 69b exit lead
class TestExitLead:
  def test_request_unwinds_ahead_of_a_lagging_wheel(self):
    cmds = exit_cmd()
    tx, xl, _, _, fl = run(cmds, V, lagging(), **NO_TRIM)
    tx0, xl0, _, _, _ = run(cmds, V, lagging(), EXIT_LEAD_GAIN=0.0, **NO_TRIM)
    cap = float(np.interp(85.0, P.EXIT_LEAD_SPEEDS_KPH, P.EXIT_LEAD_CAP_DEG))
    assert np.all(xl0 == 0.0)
    assert xl.min() < -0.5 and -xl.min() <= cap + 1e-6          # toward centre (command positive), inside the speed cap
    assert np.all(xl <= 1e-9)                                   # never outward
    d = tx0 - tx                                                # how much further unwound the request is
    assert d.max() > 0.5 and d.max() <= cap + 0.2
    assert any(f & ccmod.SF_ENTRY_BOOST for f in fl)            # steerFlags bit 31 covers the exit lead too

  def test_a_wheel_that_keeps_up_gets_nothing(self):
    cmds = exit_cmd()
    _, xl, _, _, _ = run(cmds, V, lambda i, c, tx: float(c[i]), **NO_TRIM)
    assert np.abs(xl).max() == 0.0

  def test_bounded_by_how_far_the_wheel_is_behind(self):
    cmds = exit_cmd()
    behind = 1.0                                                # wheel 1 deg beyond the command throughout
    _, xl, _, _, _ = run(cmds, V, lambda i, c, tx: float(c[i]) + behind, **NO_TRIM)
    assert 0.0 < -xl.min() <= behind - P.EXIT_LEAD_WHEEL_MARGIN_DEG + 1e-6

  def test_never_crosses_centre_and_fades_with_the_command(self):
    cmds = exit_cmd(hi=10.0, lo=0.0, post=200)
    tx, xl, _, _, _ = run(cmds, V, lagging(), **NO_TRIM)
    assert np.all(np.abs(xl) <= P.EXIT_LEAD_MAX_FRAC * np.abs(cmds) + 0.08 + 1e-6)   # 0.08: the 0.15 deg command hysteresis x 0.5
    assert tx.min() >= -0.2                                     # the request stays on the command's side
    assert abs(xl[-1]) == 0.0

  def test_no_lead_while_the_command_grows(self):
    cmds = np.concatenate([np.linspace(0.0, 10.0, 150), np.full(300, 10.0)])
    _, xl, _, _, _ = run(cmds, V, lambda i, c, tx: 0.6 * float(c[i]), **NO_TRIM)
    assert np.abs(xl).max() == 0.0

  def test_bleeds_under_the_rate_floor(self):
    cmds = exit_cmd(post=500)
    _, xl, _, _, _ = run(cmds, V, lambda i, c, tx: float(c[i]) + 4.0, **NO_TRIM)   # wheel always well behind: only cap / gain act
    i = int(np.argmin(xl)) + 5
    d = np.diff(-xl[i:])                                        # magnitude can only go down from its peak ...
    assert np.all(d <= 1e-9) and -d.min() <= P.EXIT_LEAD_BLEED_DPS * 0.01 + 1e-6   # ... and no faster than the bleed

  def test_dropped_when_the_command_grows_again(self):
    # exit 10 -> 4 deg with the wheel well behind, then straight back into a tighter curve
    cmds = np.concatenate([np.full(600, 10.0), np.linspace(10.0, 4.0, 100), np.linspace(4.0, 12.0, 100), np.full(200, 12.0)])
    _, xl, _, _, _ = run(cmds, V, lambda i, c, tx: float(c[i]) + 4.0, **NO_TRIM)
    assert -xl[690:700].min() > 0.5                             # lead built during the exit
    j = 700 + int(np.argmax((cmds[700:] - 4.0) > 2.5))          # the command is back above its slow reference by then
    assert abs(xl[j + 45]) < 0.1 * -xl[690:700].min()           # gone within ~3 time constants, not bled at 0.5 deg/s
    assert np.abs(xl[-150:]).max() == 0.0

  def test_speed_table(self):
    assert P.EXIT_LEAD_SPEEDS_KPH == [70.0, 80.0, 90.0, 120.0] and P.EXIT_LEAD_CAP_DEG == [0.0, 1.5, 1.25, 1.0]
    assert P.EXIT_LEAD_GAIN == 0.2 and P.EXIT_LEAD_MAX_FRAC == 0.5

  def test_hand_blinker_low_speed_and_kill_switch(self):
    cmds = exit_cmd()
    sim = Sim(); hand = sim.cc.hold_comp_last + 400.0
    for kw, extra, v in ((dict(tq=hand), {}, V), (dict(blinker=True), {}, V), ({}, dict(EXIT_LEAD_GAIN=0.0), V), ({}, {}, 40 * KPH),
                         ({}, {}, 70 * KPH)):                    # up to 70 km/h Phase 69a's timing does the job: no exit lead
      _, xl, _, _, _ = run(cmds, v, lagging(), **kw, **extra, **NO_TRIM)
      assert np.abs(xl).max() == 0.0


# ---------------------------------------------------------------- 69c trim rate
class TestTrimRate:
  def slope(self, cmd, wheel, v_kph, **kw):
    """Largest trim growth over any 0.5 s (deg/s): the command steps to cmd, the wheel only goes to `wheel`."""
    cmds = np.concatenate([np.zeros(100), np.full(800, cmd)])
    _, _, _, tr, _ = run(cmds, v_kph * KPH, lambda i, c, tx: wheel if i >= 100 else 0.0, ENTRY_BOOST_GAIN=0.0, EXIT_LEAD_GAIN=0.0, **kw)
    return float(np.max(tr[50:] - tr[:-50]) / 0.5), tr

  def test_large_residual_ramps_at_the_fast_rate_in_town(self):
    fast, tr = self.slope(8.0, 3.0, 50.0)
    slow, tr0 = self.slope(8.0, 3.0, 50.0, CURVE_TRIM_FAST_RATE_DPS=0.0)
    assert fast == pytest.approx(P.CURVE_TRIM_FAST_RATE_DPS, abs=0.2) and slow == pytest.approx(P.CURVE_TRIM_RATE_DPS, abs=0.1)
    assert tr[-1] == pytest.approx(tr0[-1], abs=1e-6)           # same cap in the end

  def test_small_residual_keeps_2_deg_per_s(self):
    # residual 1.0 deg -> 0.3 deg beyond the 0.7 deg deadband, under CURVE_TRIM_FAST_RESID_DEG[0]
    a, tr = self.slope(8.0, 7.0, 50.0)
    b, tr0 = self.slope(8.0, 7.0, 50.0, CURVE_TRIM_FAST_RATE_DPS=0.0)
    assert tr == pytest.approx(tr0, abs=1e-9) and a <= P.CURVE_TRIM_RATE_DPS + 0.05

  def test_highway_keeps_2_deg_per_s(self):
    a, tr = self.slope(6.0, 2.0, 85.0)
    b, tr0 = self.slope(6.0, 2.0, 85.0, CURVE_TRIM_FAST_RATE_DPS=0.0)
    assert tr == pytest.approx(tr0, abs=1e-9) and a <= P.CURVE_TRIM_RATE_DPS + 0.05

  def test_taper_between_70_and_80(self):
    a, _ = self.slope(8.0, 3.0, 75.0)
    assert P.CURVE_TRIM_RATE_DPS + 0.5 < a < P.CURVE_TRIM_FAST_RATE_DPS - 0.5

  def test_comes_down_as_fast_when_the_wheel_is_beyond(self):
    # trim wound up against a stuck wheel, then the wheel jumps 3 deg past the command: the trim leaves at the fast rate too
    def wheel(i, c, tx):
      return 3.0 if i < 500 else 11.0
    _, _, _, tr, _ = run(np.full(900, 8.0), 50 * KPH, wheel, ENTRY_BOOST_GAIN=0.0, EXIT_LEAD_GAIN=0.0)
    down = float(np.max(tr[500:-50] - tr[550:]) / 0.5)
    assert P.CURVE_TRIM_RATE_DPS + 0.5 < down <= P.CURVE_TRIM_FAST_RATE_DPS + 1e-6
