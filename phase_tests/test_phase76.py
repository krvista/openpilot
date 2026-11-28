"""Phase 76 (route a7, report 105): parking mode entry / exit rules, the cruise release, and fewer 'Steering Paused'
prompts. Drives the real car controller (harness) and the real LateralInfo."""
import types

import pytest

from phase_tests.harness import Sim
from opendbc.car.hyundai.values import CarControllerParams as P
from openpilot.sunnypilot.selfdrive.selfdrived import lateral_info as li

KPH = 1 / 3.6


@pytest.fixture(autouse=True)
def params():
  keep = {k: getattr(P, k) for k in dir(P) if k.startswith("PARKING_V2") or k == "ROAD_STOP_NO_PARKING"}
  mod = {k: getattr(li, k) for k in ("PAUSED_PROMPT_SKIP_REASONS", "PAUSED_PROMPT_PARKING_NEEDS_CRUISE")}
  yield
  for k, v in keep.items():
    setattr(P, k, v)
  for k, v in mod.items():
    setattr(li, k, v)


def drive(s, n, **kw):
  for _ in range(n):
    s.step(**kw)
  return s.cc.parking_mode_active


def test_shipped_values():
  assert P.PARKING_V2 and P.PARKING_V2_WHEEL_DEG == 300.0 and P.PARKING_V2_GPS_SIGNATURE
  assert abs(P.PARKING_V2_EXIT_MS - 30 * KPH) < 1e-9 and P.PARKING_V2_EXIT_FRAMES == 300
  assert P.PARKING_V2_LANE_PROB == 0.5 and P.PARKING_V2_CRUISE_FRAMES == 0   # Phase 81a (was 100)


def test_boot_door_and_creep_no_longer_arm():
  s = Sim()
  assert not drive(s, 400, v=10 * KPH)                               # controller start at low speed, GPS already fixed
  assert not drive(s, 1500, v=5 * KPH, lead_dist=None)               # 15 s lead-less crawl (the old creep signature)
  assert not drive(s, 400, v=20 * KPH, wheel=280.0, cmd=280.0)       # 280 deg is below the new threshold
  P.PARKING_V2 = False
  s = Sim()
  assert drive(s, 400, v=10 * KPH)                                   # the old boot signature


def test_wheel_reverse_park_arm_after_three_seconds():
  for kw in (dict(wheel=305.0, cmd=305.0), dict(gear='reverse'), dict(gear='park')):
    s = Sim()
    drive(s, 100, v=40 * KPH)
    assert not drive(s, 290, v=12 * KPH, **kw)
    assert drive(s, 20, v=12 * KPH, **kw), kw


def test_wheel_event_above_30_or_in_an_earlier_stretch_does_not_arm():
  s = Sim()
  drive(s, 100, v=40 * KPH)
  drive(s, 100, v=12 * KPH, wheel=305.0, cmd=305.0)                  # 1 s only
  drive(s, 100, v=35 * KPH)                                          # stretch over: the signature goes with it
  assert not drive(s, 600, v=12 * KPH)


def test_gps_signature_until_first_fix_or_release():
  s = Sim()
  assert drive(s, 320, v=12 * KPH, gps_fix_seen=False)               # underground start
  assert not drive(s, 320, v=40 * KPH, gps_fix_seen=False)           # released by speed ...
  assert not drive(s, 600, v=12 * KPH, gps_fix_seen=False)           # ... and the signature is spent (cold start outdoors)
  s = Sim()
  drive(s, 100, v=12 * KPH, gps_fix_seen=False)
  assert not drive(s, 600, v=12 * KPH, gps_fix_seen=True)            # fix before the 3 s: no latch
  P.PARKING_V2_GPS_SIGNATURE = False
  s = Sim()
  assert not drive(s, 600, v=12 * KPH, gps_fix_seen=False)


def latched():
  s = Sim()
  drive(s, 100, v=40 * KPH)
  assert drive(s, 320, v=12 * KPH, wheel=305.0, cmd=305.0)
  return s


def test_exit_at_30_kph_for_three_seconds():
  s = latched()
  assert drive(s, 290, v=30.5 * KPH)
  assert not drive(s, 20, v=30.5 * KPH)
  s = latched()
  assert drive(s, 1000, v=29 * KPH)                                  # 29 km/h never releases by speed


def test_exit_on_lane_lines_at_half_confidence():
  s = latched()
  assert drive(s, 290, v=20 * KPH, lane_prob_min=0.55)
  assert not drive(s, 20, v=20 * KPH, lane_prob_min=0.55)
  s = latched()
  assert drive(s, 600, v=20 * KPH, lane_prob_min=0.45)
  s = latched()
  assert drive(s, 600, v=20 * KPH, lane_prob_min=0.9, wheel=40.0, cmd=40.0)   # wheel not straight


def test_cruise_engagement_releases_after_one_second():
  P.PARKING_V2_CRUISE_FRAMES = 100                                    # the Phase 76 rule (off since Phase 81a)
  s = latched()
  assert drive(s, 90, v=18 * KPH, cruise_enabled=True)
  assert not drive(s, 20, v=18 * KPH, cruise_enabled=True)
  assert not drive(s, 600, v=18 * KPH, cruise_enabled=True)          # stays released at low speed
  assert drive(s, 320, v=12 * KPH, wheel=305.0, cmd=305.0, cruise_enabled=True)   # a new big turn arms again
  for kw in (dict(v=10 * KPH), dict(v=18 * KPH, wheel=45.0, cmd=45.0), dict(v=18 * KPH, wheel=0.0, cmd=15.0)):
    s = latched()
    assert drive(s, 400, cruise_enabled=True, **kw), kw              # too slow / wheel turned / request far from the wheel
  P.PARKING_V2_CRUISE_FRAMES = 0
  s = latched()
  assert drive(s, 400, v=18 * KPH, cruise_enabled=True)


def _prompts(reason, cruise, v=40.0, n=400):
  inf = li.LateralInfo(); out = 0
  CS = types.SimpleNamespace(vEgo=v * KPH, steeringPressed=False, cruiseState=types.SimpleNamespace(enabled=cruise))
  sp = types.SimpleNamespace(lateralControlPaused=True, lateralPausedReason=reason, gatewayFlags=0)
  for _ in range(n):
    out += li.EventNameSP.lateralPaused in inf.update(CS, sp, True)
  return out


def test_no_prompt_for_driver_made_reasons_and_parking_needs_cruise():
  assert [_prompts(r, False) for r in (5, 6, 10)] == [0, 0, 0]
  assert [_prompts(r, True) for r in (5, 6, 10)] == [0, 0, 0]
  assert _prompts(7, False) == 1 and _prompts(1, False) == 1         # scenario gate, camera
  assert _prompts(4, False) == 0 and _prompts(4, True) == 1          # parking: only with cruise engaged
  assert _prompts(4, True, n=140) == 0                               # ... and only once the cruise release had its 1 s
  li.PAUSED_PROMPT_SKIP_REASONS = (); li.PAUSED_PROMPT_PARKING_NEEDS_CRUISE = False
  assert _prompts(5, False) == 1 and _prompts(4, False) == 1
