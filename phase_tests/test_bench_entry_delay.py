"""Bench tool check (report 117 / 119 item 6): E179 LT-05 measures the curve entry DELAY as the time shift of the rate of
change, so a command that is smaller early in the entry (the lane terms pushing outward) is not read as 'late'. The old
measure - when each signal passes 20 % of the MODEL's peak - is kept here only to show what it did with the same signals."""
import importlib.util
import os

import numpy as np
import pytest

_spec = importlib.util.spec_from_file_location(
  "e179", os.path.join(os.path.dirname(__file__), "..", "tools", "i6nv3_bench", "probes", "qa", "e179_next_drive_check.py"))
e179 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(e179)

E0, PEAK = 600, 2.0e-3      # entry frame, the model's peak curvature


def model(n=1500, t0=E0, rise=250, tex=0.0):
  """Curve entry: 0 until t0, PEAK after `rise` frames (smooth step). tex adds the model's own texture as four sines of
  0.4-1.6 Hz, each tex * PEAK: at 0.03 the rate of change minus its 1 s mean is 0.75 of the rate's sd - the logged entries
  of b4 / b6 have 0.74 (IQR 0.66..0.87, n 55)."""
  x = np.clip((np.arange(n) - t0) / rise, 0.0, 1.0); y = PEAK * x * x * (3 - 2 * x); t = np.arange(n) / 100.0
  return y + sum(tex * PEAK * np.sin(2 * np.pi * f * t + ph) for f, ph in ((0.4, 0.3), (0.7, 2.1), (1.1, 4.0), (1.6, 5.5)))


def delayed(x, frames):
  return np.interp(np.arange(len(x)) - frames, np.arange(len(x)), x)


def old_t20(x, pk):
  i = np.flatnonzero(x[E0 - 150:E0 + 400] >= 0.2 * pk)
  return (i[0] - 150) * 10.0


def timing(km, kc, kw, s=1.0):
  j = E0 + int(np.argmax(np.abs(km[E0:E0 + 400])))
  return e179.entry_timing(*(e179.box(x, 20) for x in (km, kc, kw)), E0, j, s)


@pytest.mark.parametrize("scale", [1.0, 0.8, 0.5])
def test_delay_does_not_depend_on_the_amplitude(scale):
  km = model(); kc = scale * delayed(km, 3.0); kw = 0.9 * scale * delayed(km, 27.0)
  tw, tc, cw, cc, *_, pc, pw = timing(km, kc, kw)
  assert tc == pytest.approx(30.0, abs=3.0) and tw == pytest.approx(270.0, abs=5.0)
  assert cw > 0.99 and cc > 0.99
  assert pc == pytest.approx(scale, abs=0.02) and pw == pytest.approx(0.9 * scale, abs=0.02)


def lane_term(n):
  """outward lane correction that comes in over 1.2 s and fades over 4 s; the logged mean over the entry is 0.17e-3 (report 118)"""
  t = np.arange(n) - E0
  return 0.25e-3 * np.clip(t / 120.0, 0.0, 1.0) * np.clip(1.0 - (t - 120) / 400.0, 0.0, 1.0)


def test_outward_lane_term_early_in_the_entry_is_an_amplitude_not_a_delay():
  """The case of routes aa-b6: the command is the model 30 ms later minus an outward lane correction."""
  km = model(tex=0.03); kc = delayed(km, 3.0) - lane_term(len(km))
  assert old_t20(kc, PEAK) - old_t20(km, PEAK) >= 100.0   # the old measure: 'the command is > 100 ms late'
  tw, tc, _, cc, ow, oc, early_c, *_ = timing(km, kc, delayed(kc, 24.0))
  assert tc == pytest.approx(30.0, abs=15.0) and tw - tc == pytest.approx(240.0, abs=6.0)
  assert early_c < 0.85                                   # and the smaller early command is reported as what it is
  assert tw <= e179.LT05_WHEEL_OK and tc < e179.LT05_CMD_FAIL


def test_known_limit_a_textureless_entry_cannot_separate_an_additive_term_from_a_delay():
  """The shift is found from the texture the model and the command share. On an ideal smooth step with nothing but the
  additive term, the term reshapes the onset and the estimate moves with it - logged entries are not like that (see model())."""
  km = model(tex=0.0); kc = delayed(km, 3.0) - lane_term(len(km))
  assert timing(km, kc, delayed(kc, 24.0))[1] > 100.0


def test_a_real_delay_is_still_found_and_fails():
  km = model(); kc = delayed(km, 9.0); kw = delayed(km, 36.0)
  tw, tc, *_ = timing(km, kc, kw)
  assert tc == pytest.approx(90.0, abs=3.0) and tw == pytest.approx(360.0, abs=5.0)
  assert tc > e179.LT05_CMD_FAIL and tw > e179.LT05_WHEEL_FAIL


def test_sign_offset_and_noise():
  rng = np.random.default_rng(1); km = -model() + 0.2e-3
  kc = delayed(km, 3.0) + 0.1e-3 + rng.normal(0, 1e-5, len(km)); kw = delayed(km, 27.0) - 0.15e-3 + rng.normal(0, 1e-5, len(km))
  tw, tc, *_ = timing(km, kc, kw, s=-1.0)
  assert tc == pytest.approx(30.0, abs=10.0) and tw == pytest.approx(270.0, abs=15.0)


def test_shift_on_the_search_limit_is_flagged():
  km = model(); d = np.diff(e179.box(km, 20)[E0 - 130:E0 + 400])
  far = np.diff(e179.box(delayed(km, 150.0), 20)[E0 - 130:E0 + 400])
  assert e179.shift_ms(d, far)[2] and not e179.shift_ms(d, d)[2]
  assert e179.shift_ms(d, d)[0] == pytest.approx(0.0, abs=1e-6)


def test_lt05_row_on_a_synthetic_route(tmp_path):
  """Twenty entries through load() -> curve_entries(): PASS with the baseline delays, FAIL when the wheel is 100 ms later."""
  def route(wheel_frames):
    n = 20 * 1500 + 600; km = np.zeros(n)
    for k in range(20):
      a = 300 + k * 1500
      km[a:a + 1500] = (model(1500, 600, 250) - model(1500, 1100, 250)) * (1 if k % 2 else -1)
    t = np.arange(n) / 100.0
    d = dict(T=t, v=np.full(n, 70.0), k_model=km, k_ctrl=0.85 * delayed(km, 3.0), k_meas=0.8 * delayed(km, wheel_frames),
             tx_active=np.full(n, 2), cb_lvl2=np.full(n, 2), mdps_active=np.full(n, 2), lat=np.ones(n, bool),
             pressed=np.zeros(n, bool), ang=np.zeros(n))
    p = str(tmp_path / f"r{wheel_frames}.npz"); np.savez(p, **d)
    return p
  for frames, want in ((27.0, "PASS"), (37.0, "FAIL")):
    e179.ROWS.clear(); o = e179.load(route(frames)); e179.curve_entries(o)
    r = [x for x in e179.ROWS if x[0] == "LT-05"][0]
    assert len(o["E"]) == 20 and r[3] == want, r
    assert np.median(o["E"][:, 2]) == pytest.approx(frames * 10.0, abs=6.0) and np.median(o["E"][:, 3]) == pytest.approx(30.0, abs=4.0)
  e179.ROWS.clear()
