"""Phase 54 (report §51): soundd ambient -> output-gain curve 22 dB 15 % / 30 dB 20 % / 42 dB 35 % / 48 dB 60 % / 56 dB 100 %,
log-domain interpolation, flat outside. Upstream gave 6.7 % / 9.5 % / 27 % / 49 % / 100 % at the same points."""
import math

import numpy as np
import pytest

import phase_tests.harness_noncontrol  # noqa: F401 (stubs msgq)
import openpilot.selfdrive.ui.soundd as sd


def calc(db):
  return sd.Soundd.calculate_volume(None, db)


def upstream(db):
  v = ((db - sd.AMBIENT_DB) / sd.DB_SCALE) * (sd.MAX_VOLUME - sd.MIN_VOLUME) + sd.MIN_VOLUME
  return math.pow(sd.VOLUME_BASE, min(max(v, sd.MIN_VOLUME), sd.MAX_VOLUME) - 1)


def test_user_points():
  for db, g in ((22.0, 0.15), (30.0, 0.20), (42.0, 0.35), (48.0, 0.60), (56.0, 1.00)):
    assert calc(db) == pytest.approx(g)


def test_flat_outside_and_bounded():
  assert calc(0.0) == pytest.approx(0.15) and calc(21.5) == pytest.approx(0.15)   # very quiet stop / night
  assert 0.15 < calc(26.0) < 0.20
  assert calc(70.0) == pytest.approx(1.0)
  vals = [calc(x) for x in np.arange(0, 90, 0.5)]
  assert all(b >= a - 1e-12 for a, b in zip(vals, vals[1:])) and max(vals) <= 1.0


def test_log_interpolation_between_points():
  # midway in dB between 30 and 42 -> geometric mean of 0.20 and 0.35
  assert calc(36.0) == pytest.approx(math.sqrt(0.20 * 0.35))


def test_louder_than_upstream_where_alerts_sound():
  for db in (28.0, 35.0, 40.0):           # ambient at alert onsets p10/p50/p90 (routes 43/45)
    assert calc(db) > upstream(db)


def test_initial_volume():
  assert sd.VOLUME_INITIAL == pytest.approx(0.20)


def test_kill_restores_upstream():
  old = sd.VOLUME_CURVE_I6N
  try:
    sd.VOLUME_CURVE_I6N = False
    for db in (29.8, 41.6, 48.1):
      assert calc(db) == pytest.approx(upstream(db))
  finally:
    sd.VOLUME_CURVE_I6N = old
