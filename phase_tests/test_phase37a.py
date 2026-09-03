"""Phase 37a: high-speed recovery softening (rise cap, recovery jerk cap) + wiper-derived rain mode."""
import math
import numpy as np
from phase_tests.harness import Sim, run_signal
from opendbc.car.hyundai.values import CarControllerParams as P
from opendbc.car import DT_CTRL


def settle(sim, n=120, **kw):
  kw.setdefault('v', 15.0); kw.setdefault('wheel', 0.0); kw.setdefault('cmd', 0.0); kw.setdefault('tq', 0.0)
  return run_signal(sim, n, **kw)


def _release_with_error(v, err, n=120, **extra):
  """grip at speed (anchor + gain to floor), then release against a constant error."""
  sim = Sim()
  settle(sim)
  run_signal(sim, 200, v=v, wheel=0.0, cmd=0.0, tq=0.0, **extra)
  run_signal(sim, 100, v=v, wheel=0.0, cmd=0.0, tq=460.0, **extra)
  tr = run_signal(sim, n, v=v, wheel=0.0, cmd=err, tq=0.0, **extra)
  return sim, tr


def _max_rise(tr):
  g = tr['gain']
  return max(b - a for a, b in zip(g, g[1:]))


class TestPhase37aRiseCap:
  def test_cap_binds_at_100kph(self):
    _, tr = _release_with_error(v=27.8, err=1.5)
    assert _max_rise(tr) <= 0.012 + 1e-9, _max_rise(tr)
    assert tr['gain'][-1] > tr['gain'][0] + 0.2, "gain must still recover"

  def test_unchanged_at_60kph(self):
    _, tr = _release_with_error(v=16.7, err=1.5)
    assert _max_rise(tr) >= 0.036, _max_rise(tr)          # legacy 0.04 peak survives

  def test_kill_restores_fast_rise(self):
    old = P.ACIGAIN_RATE_UP_CAP_V
    try:
      P.ACIGAIN_RATE_UP_CAP_V = [0.04, 0.04]
      _, tr = _release_with_error(v=27.8, err=1.5)
    finally:
      P.ACIGAIN_RATE_UP_CAP_V = old
    assert _max_rise(tr) >= 0.036, _max_rise(tr)


def _vm_cap_deg_per_frame(sim, v, j):
  return math.degrees(sim.s.VM.get_steer_from_curvature(j / v**2, v, 0)) * DT_CTRL


class TestPhase37aRecoveryJerkCap:
  # the harness runs BOTH VM limiters (car + safety-baseline model), so the
  # reference for "unchanged" is the same run with the cap killed, not the
  # analytic single-VM number
  def _chase(self, v, n=150, kill=False, **extra):
    old = P.RECOVERY_JERK_CAP_V
    try:
      if kill:
        P.RECOVERY_JERK_CAP_V = [3.59, 3.59]
      sim, tr = _release_with_error(v=v, err=6.0, n=n, **extra)
    finally:
      P.RECOVERY_JERK_CAP_V = old
    a = tr['apply']
    return sim, max(abs(b - x) for x, b in zip(a, a[1:]))

  def test_recovery_bounded_at_100kph(self):
    sim, step = self._chase(v=27.8)
    _, step_kill = self._chase(v=27.8, kill=True)
    cap = _vm_cap_deg_per_frame(sim, 27.8, 2.5)
    assert step <= cap + 1e-6, (step, cap)
    assert step >= 0.8 * cap, "cap must actually be the binding limit in the chase"
    assert step < 0.85 * step_kill, (step, step_kill)

  def test_panda_limit_untouched_below_60kph(self):
    _, step = self._chase(v=16.6)                 # 59.8 km/h: below the taper's first knot
    _, step_kill = self._chase(v=16.6, kill=True)
    assert abs(step - step_kill) <= 1e-9, (step, step_kill)

  def test_outside_recovery_window_unbounded(self):
    def run(kill):
      old = P.RECOVERY_JERK_CAP_V
      try:
        if kill:
          P.RECOVERY_JERK_CAP_V = [3.59, 3.59]
        sim = Sim(); settle(sim)
        run_signal(sim, 400, v=27.8, wheel=0.0, cmd=0.0, tq=0.0)
        assert sim.s.frames_since_apply_anchor > P.RECOVERY_JERK_CAP_FRAMES
        tr = run_signal(sim, 60, v=27.8, wheel=0.0, cmd=6.0, tq=0.0)
      finally:
        P.RECOVERY_JERK_CAP_V = old
      a = tr['apply']
      return max(abs(b - x) for x, b in zip(a, a[1:]))
    assert abs(run(False) - run(True)) <= 1e-9

  def test_kill_identical_to_panda_limit(self):
    sim, step = self._chase(v=27.8, kill=True)
    assert step >= 0.8 * _vm_cap_deg_per_frame(sim, 27.8, 3.59), step


class TestPhase37aRainMode:
  def test_debounce_on_off(self):
    sim = Sim(); settle(sim)
    run_signal(sim, P.RAIN_WIPER_ON_FRAMES - 1, v=20.0, wiper=True)
    assert not sim.s.rain_active
    run_signal(sim, 1, v=20.0, wiper=True)
    assert sim.s.rain_active
    run_signal(sim, P.RAIN_WIPER_OFF_FRAMES - 1, v=20.0, wiper=False)
    assert sim.s.rain_active, "60 s off debounce"
    run_signal(sim, 1, v=20.0, wiper=False)
    assert not sim.s.rain_active

  def test_weight_ramps_no_step(self):
    sim = Sim(); settle(sim)
    ws = []
    for _ in range(P.RAIN_WIPER_ON_FRAMES + 600):
      run_signal(sim, 1, v=20.0, wiper=True); ws.append(sim.s.rain_w)
    d = max(b - a for a, b in zip(ws, ws[1:]))
    assert d <= DT_CTRL / P.RAIN_RAMP_UP_TAU_S + 1e-6, d
    assert ws[-1] > 0.8 and ws[P.RAIN_WIPER_ON_FRAMES - 2] == 0.0 and ws[P.RAIN_WIPER_ON_FRAMES - 1] > 0.0

  def test_stale_input_counts_as_off(self):
    sim = Sim(); settle(sim)
    run_signal(sim, P.RAIN_WIPER_ON_FRAMES + 50, v=20.0, wiper=True, wiper_stale=True)
    assert not sim.s.rain_active and sim.s.rain_w == 0.0

  def test_rain_tightens_rise_cap_and_jerk_cap(self):
    v = 27.8
    sim = Sim(); settle(sim)
    run_signal(sim, P.RAIN_WIPER_ON_FRAMES + 2500, v=v, wiper=True)        # rain_w -> ~0.999
    assert sim.s.rain_w > 0.99
    run_signal(sim, 100, v=v, wheel=0.0, cmd=0.0, tq=460.0, wiper=True)
    tr = run_signal(sim, 150, v=v, wheel=0.0, cmd=6.0, tq=0.0, wiper=True)
    a = tr['apply']; step = max(abs(b - x) for x, b in zip(a, a[1:]))
    assert step <= 1.02 * _vm_cap_deg_per_frame(sim, v, 1.5), step   # rain_w ~0.999 -> within 2% of the 1.5 cap
    assert step <= 0.7 * _vm_cap_deg_per_frame(sim, v, 2.5), step    # clearly below the dry cap
    # rise cap in rain: 0.008 @100 km/h
    sim3 = Sim(); settle(sim3)
    run_signal(sim3, P.RAIN_WIPER_ON_FRAMES + 2500, v=v, wiper=True)
    run_signal(sim3, 100, v=v, wheel=0.0, cmd=0.0, tq=460.0, wiper=True)
    tr3 = run_signal(sim3, 120, v=v, wheel=0.0, cmd=1.5, tq=0.0, wiper=True)
    assert _max_rise(tr3) <= 0.008 + 1e-9, _max_rise(tr3)   # rain cap 0.008 @100 km/h (quantized 0.004 grid)

  def test_kill_never_activates(self):
    old = P.RAIN_WIPER_ON_FRAMES
    try:
      P.RAIN_WIPER_ON_FRAMES = 10**9
      sim = Sim(); settle(sim)
      run_signal(sim, 1200, v=20.0, wiper=True)
    finally:
      P.RAIN_WIPER_ON_FRAMES = old
    assert not sim.s.rain_active and sim.s.rain_w == 0.0


class TestPhase37aDbcAndParser:
  def test_wiper_message_in_generated_dbc(self):
    from opendbc import get_generated_dbcs
    dbc = get_generated_dbcs()["hyundai_canfd_generated"]
    assert "BO_ 860 CCNC_WIPER: 24" in dbc and "FRONT_WIPER_ON : 144|1@1+" in dbc

  def test_wiper_bit_decodes(self):
    from phase_tests.harness import make_cp
    from opendbc.car.hyundai.carstate import CarState
    from opendbc.car import structs, Bus
    CP = make_cp(); CP_SP = structs.CarParamsSP()
    cp = CarState(CP, CP_SP).get_can_parsers(CP, CP_SP)[Bus.pt]
    assert 0x35c in cp.message_states and cp.message_states[0x35c].ignore_alive
    dat = bytearray(24); dat[18] = 0x49          # bits 0, 3, 6 set
    cp.update([(1_000_000, [(0x35c, bytes(dat), cp.bus)])])
    vl = cp.vl["CCNC_WIPER"]
    assert vl["FRONT_WIPER_ON"] == 1 and vl["WIPER_AUX_BIT3"] == 1 and vl["WIPER_STALK_PULSE"] == 1
    assert cp.ts_nanos["CCNC_WIPER"]["FRONT_WIPER_ON"] == 1_000_000
    dat[18] = 0x00
    cp.update([(2_000_000, [(0x35c, bytes(dat), cp.bus)])])
    assert cp.vl["CCNC_WIPER"]["FRONT_WIPER_ON"] == 0
