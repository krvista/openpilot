"""Phase 54 (report §51): soundd output-gain floor 20 % — quiet-cabin alerts (green-light chime, prompts) were at 9.5 %
(-20 dB, route 45 ambient p10 30 dB). Louder cabins keep the ambient curve."""
import math

import pytest

import phase_tests.harness_noncontrol  # noqa: F401 (stubs msgq)
import openpilot.selfdrive.ui.soundd as sd


def curve(db):   # the unmodified ambient curve
  v = ((db - sd.AMBIENT_DB) / sd.DB_SCALE) * (sd.MAX_VOLUME - sd.MIN_VOLUME) + sd.MIN_VOLUME
  return math.pow(sd.VOLUME_BASE, min(max(v, sd.MIN_VOLUME), sd.MAX_VOLUME) - 1)


def calc(db):
  return sd.Soundd.calculate_volume(None, db)


def test_quiet_cabin_starts_at_20_percent():
  for db in (0.0, 20.0, 29.8, 33.0):
    assert calc(db) == pytest.approx(0.20)
  assert curve(29.8) < 0.10                      # route 45 p10 was 9.5 % before


def test_louder_cabin_follows_the_curve():
  for db in (41.6, 48.1, 56.0, 70.0):
    assert curve(db) > 0.20 and calc(db) == pytest.approx(curve(db))
  assert calc(70.0) == pytest.approx(sd.MAX_VOLUME)


def test_monotonic_and_bounded():
  vals = [calc(db) for db in range(0, 90)]
  assert all(b >= a for a, b in zip(vals, vals[1:]))
  assert min(vals) == pytest.approx(0.20) and max(vals) <= sd.MAX_VOLUME


def test_floor_constant_and_kill():
  assert sd.OUTPUT_VOLUME_FLOOR == pytest.approx(0.20)
  old = sd.OUTPUT_VOLUME_FLOOR
  try:
    sd.OUTPUT_VOLUME_FLOOR = 0.0
    assert calc(29.8) == pytest.approx(curve(29.8))
  finally:
    sd.OUTPUT_VOLUME_FLOOR = old
