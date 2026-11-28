"""Phase 81 (report 112): cruise engagement no longer releases the parking latch, boot grace for the RX-check flag,
GPS parking signature cleared once the car has been on the road."""
import os
import types

import pytest

from opendbc.car.hyundai.values import CarControllerParams as P
from phase_tests.harness import Sim

KPH = 1 / 3.6
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(autouse=True)
def params():
  saved = (P.PARKING_V2_CRUISE_FRAMES, P.PARKING_V2_GPS_CLEAR_ON_ROAD)
  yield
  P.PARKING_V2_CRUISE_FRAMES, P.PARKING_V2_GPS_CLEAR_ON_ROAD = saved


def drive(s, n, **kw):
  kw.setdefault("wheel", 0.0); kw.setdefault("cmd", kw["wheel"])
  for _ in range(n):
    s.step(**kw)
  return bool(s.cc.parking_mode_active)


# ------------------------------------------------------------------ 81a
def test_cruise_does_not_release_the_latch():
  assert P.PARKING_V2_CRUISE_FRAMES == 0
  s = Sim()
  assert drive(s, 320, v=12 * KPH, wheel=305.0)
  assert drive(s, 600, v=18 * KPH, cruise_enabled=True)              # car park, cruise on, straight wheel: still latched
  assert not drive(s, 320, v=35 * KPH, cruise_enabled=True)          # 30 km/h for 3 s releases as before


# ------------------------------------------------------------------ 81c
def test_start_above_30_spends_the_gps_signature():
  assert P.PARKING_V2_GPS_CLEAR_ON_ROAD is True
  s = Sim()
  assert not drive(s, 320, v=50 * KPH, gps_fix_seen=False)           # controller comes up on the road, no fix yet
  assert not drive(s, 600, v=12 * KPH, gps_fix_seen=False)           # slowing down is not a car park
  assert drive(s, 320, v=12 * KPH, wheel=305.0, gps_fix_seen=False)  # the wheel rule still arms


def test_gps_clear_needs_the_full_three_seconds_and_has_a_kill():
  s = Sim()
  drive(s, 200, v=50 * KPH, gps_fix_seen=False)                      # 2 s above 30: not yet
  assert drive(s, 320, v=12 * KPH, gps_fix_seen=False)
  P.PARKING_V2_GPS_CLEAR_ON_ROAD = False
  s = Sim()
  drive(s, 320, v=50 * KPH, gps_fix_seen=False)
  assert drive(s, 320, v=12 * KPH, gps_fix_seen=False)               # old behaviour


def test_underground_start_still_latches():
  s = Sim()
  assert drive(s, 320, v=12 * KPH, gps_fix_seen=False)


# ------------------------------------------------------------------ 81b
def test_rx_check_flag_waits_for_the_safety_mode_grace():
  import openpilot.selfdrive.selfdrived.selfdrived as sd
  assert sd.RX_CHECK_BOOT_GRACE is True
  src = open(os.path.join(ROOT, "openpilot/selfdrive/selfdrived/selfdrived.py")).read()
  assert "rx_invalid = pandaState.safetyRxChecksInvalid and (self.safety_mode_grace_over() or not RX_CHECK_BOOT_GRACE)" in src
  assert "(safety_mismatch and self.safety_mode_grace_over()) or rx_invalid or self.mismatch_counter >= 200" in src
  grace = lambda frame, init: sd.SelfdriveD.safety_mode_grace_over(   # noqa: E731
    types.SimpleNamespace(sm=types.SimpleNamespace(frame=frame), init_frame=init))
  assert not grace(1164, 900) and grace(1301, 900)                   # a9: flag at 11.64 s is inside the grace; later it counts
