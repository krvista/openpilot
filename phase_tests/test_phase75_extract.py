"""Phase 75 (report §103, H10): tools/i6nv3_bench/route_extract.py v6 - MADS state, panda permissions, button events, gear,
lead, cruise main, latGranted / passive reason, FCA_ESA bits of ADAS_CMD_35_10ms. A synthetic rlog (frames packed from
the car's DBC) goes through the same code path as a route: file -> LogReader -> Extractor -> npz."""
import importlib.util
import os
from types import SimpleNamespace

import numpy as np
import pytest
import zstandard

from opendbc.can.packer import CANPacker
from openpilot.cereal import log

_spec = importlib.util.spec_from_file_location(
  "route_extract", os.path.join(os.path.dirname(__file__), "..", "tools", "i6nv3_bench", "route_extract.py"))
rx = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rx)

PACKER = CANPacker("hyundai_canfd_generated")
T0 = 1000.0   # s, log time of the first message
N = 300       # carState frames (3 s)


def ev(t, which, n=None):
  e = log.Event.new_message()
  e.logMonoTime = int(round(t * 1e9))
  return e, (e.init(which) if n is None else e.init(which, n))


def can_frames(e_list, frames):
  for c, (addr, dat, src) in zip(e_list, frames, strict=True):
    c.address, c.dat, c.src = addr, dat, src


def frame(name, src, values):
  addr, dat, _ = PACKER.make_can_msg(name, 0, values)
  return addr, dat, src


def build(v6=True):
  """3 s: MADS enables at 1 s, the panda grants lateral at 1.2 s (latGranted follows), LFA button at 0.5 s, a lead from
  2 s, FCA_ESA active for 2.0-2.5 s, passive reason 9 (not granted) for 1.0-1.2 s, gear park -> drive at 0.3 s."""
  msgs = []
  e, x = ev(T0 - 0.005, "initData"); x.gitCommit = "0123456789abcdef"; msgs.append(e)
  for i in range(N):
    t = T0 + i * 0.01; rel = i * 0.01
    e, s = ev(t, "carState")
    s.vEgo = 10.0; s.steeringAngleDeg = 2.0; s.cruiseState.enabled = rel >= 2.0; s.cruiseState.available = rel >= 0.8
    s.gearShifter = "drive" if rel >= 0.3 else "park"
    if i in (50, 55):
      b = s.init("buttonEvents", 1); b[0].type = "lkas"; b[0].pressed = i == 50
    if i == 80:
      b = s.init("buttonEvents", 2); b[0].type = "mainCruise"; b[0].pressed = True; b[1].type = "mainCruise"; b[1].pressed = False
    msgs.append(e)
    e, c = ev(t + 0.001, "carControl"); c.latActive = rel >= 1.0; c.actuators.steeringAngleDeg = 3.0; msgs.append(e)
    e, sp = ev(t + 0.002, "carStateSP"); sp.gatewayFlags = 17; sp.steerFlags = 4
    if v6:
      sp.lateralPausedReason = 9 if 1.0 <= rel < 1.2 else 0
    msgs.append(e)
    if v6:
      e, cs = ev(t + 0.001, "carControlSP"); cs.latGranted = rel >= 1.2; msgs.append(e)
      e, ss = ev(t + 0.003, "selfdriveStateSP")
      ss.mads.state = "enabled" if rel >= 1.0 else "disabled"; ss.mads.enabled = rel >= 1.0; ss.mads.active = rel >= 1.2
      ss.mads.available = True; msgs.append(e)
      if i % 10 == 5:   # 10 Hz, first one at 0.05 s
        e, ps = ev(t, "pandaStates", 2)
        ps[0].safetyModel = "hyundaiCanfd"; ps[0].controlsAllowed = rel >= 2.0; ps[0].controlsAllowedLateral = rel >= 1.2
        ps[1].safetyModel = "silent"   # says nothing about the car
        msgs.append(e)
      if i % 5 == 2:    # 20 Hz
        e, rs = ev(t, "radarState"); rs.leadOne.present = rel >= 2.0
        rs.leadOne.dRel = 25.0 if rel >= 2.0 else 0.0; rs.leadOne.vRel = -1.5 if rel >= 2.0 else 0.0; msgs.append(e)
    esa = 2 if 2.0 <= rel < 2.5 else 0
    fr = [frame("ADAS_CMD_35_10ms", 1, {"ADAS_StrAnglReqVal": -12.3, "ADAS_ACIAnglTqRedcGainVal": 0.6, "ADAS_ActvACISta": 2,
                                        "ADAS_ActvACILvl2Sta": 1, "FCA_ESA_ActvSta": esa,
                                        "FCA_ESA_TqBstGainVal": 0.5 if esa else 0.0}),
          frame("LKAS_ALT", 2, {"ADAS_StrAnglReqVal": 4.5, "ADAS_ACIAnglTqRedcGainVal": 0.4, "LKAS_ANGLE_ACTIVE": 2,
                                "LKAS_BYTE13": 0xA5, "LFA_BUTTON": 1}),
          frame("ADAS_CMD_35_10ms", 0, {"FCA_ESA_ActvSta": 3})]   # other bus: not the MDPS side, ignored
    e, cn = ev(t + 0.004, "can", len(fr)); can_frames(cn, fr); msgs.append(e)
    fr = [frame("LKAS_ALT", 0, {"ADAS_StrAnglReqVal": -7.0, "ADAS_ACIAnglTqRedcGainVal": 1.0, "LKAS_ANGLE_ACTIVE": 2})]
    e, sc = ev(t + 0.005, "sendcan", 1); can_frames(sc, fr); msgs.append(e)
  return msgs


def at(d, key, rel):
  return d[key][int(np.searchsorted(d["T"], rel + 0.0051 + 1e-6))]


@pytest.fixture(scope="module")
def route(tmp_path_factory):
  """The synthetic route as two rlog segments on disk, extracted with main() like a real route."""
  d = tmp_path_factory.mktemp("logs")
  msgs = build()
  half = len(msgs) // 2
  for seg, part in enumerate((msgs[:half], msgs[half:])):
    with open(d / f"000_000000ff--abc--{seg}--rlog.zst", "wb") as f:
      f.write(zstandard.compress(b"".join(m.to_bytes() for m in part)))
  out = str(d / "out.npz")
  rx.main("000000ff", out, str(d))
  return np.load(out, allow_pickle=True)


def test_version_and_grid(route):
  assert int(route["version"]) == rx.VERSION == 6
  assert len(route["T"]) == N and str(route["commit"]) == "012345678"
  n = len(route["T"])
  for k in route.files:
    if k not in ("commit", "passive", "events", "buttons", "version"):
      assert route[k].shape == (n,), k


def test_mads_state(route):
  assert at(route, "mads_state", 0.5) == 0 and at(route, "mads_state", 1.5) == 2
  assert not at(route, "mads_enabled", 0.9) and at(route, "mads_enabled", 1.1)
  assert not at(route, "mads_active", 1.1) and at(route, "mads_active", 1.3)
  assert route["mads_available"][1:].all() and not route["mads_available"][0]   # unknown until the first message


def test_panda_permissions_skip_the_silent_panda_and_are_unknown_before_the_first_message(route):
  assert at(route, "panda_ctrl", 0.0) == -1 and at(route, "panda_lat", 0.0) == -1   # first pandaStates at 0.05 s
  assert at(route, "panda_lat", 1.0) == 0 and at(route, "panda_lat", 1.4) == 1
  assert at(route, "panda_ctrl", 1.4) == 0 and at(route, "panda_ctrl", 2.2) == 1


def test_panda_allowed_cases():
  def P(model, c, lat):
    return SimpleNamespace(safetyModel=model, controlsAllowed=c, controlsAllowedLateral=lat)
  assert rx.panda_allowed([]) == (-1, -1)
  assert rx.panda_allowed([P("silent", True, True), P("noOutput", True, True)]) == (-1, -1)
  assert rx.panda_allowed([P("hyundaiCanfd", False, True)]) == (0, 1)
  assert rx.panda_allowed([P("hyundaiCanfd", True, True), P("hyundaiCanfd", True, False)]) == (1, 0)
  assert rx.panda_allowed([SimpleNamespace(safetyModel="hyundaiCanfd", controlsAllowed=True)]) == (1, 1)   # old schema


def test_buttons_gear_and_cruise_main(route):
  b = route["buttons"]
  lkas, main = 6, 8   # car.capnp ButtonEvent.Type, as in the extractor's docstring
  assert b.shape == (4, 3)
  np.testing.assert_allclose(b, [[0.505, lkas, 1], [0.555, lkas, 0], [0.805, main, 1], [0.805, main, 0]], atol=1e-6)
  assert at(route, "gear", 0.1) == 1 and at(route, "gear", 0.5) == 2   # park, drive
  assert not at(route, "cruise_avail", 0.7) and at(route, "cruise_avail", 0.9)
  assert not at(route, "cruise", 1.9) and at(route, "cruise", 2.1)


def test_lead(route):
  assert not at(route, "lead", 1.5) and at(route, "lead_d", 1.5) == 0.0
  assert at(route, "lead", 2.5) and at(route, "lead_d", 2.5) == 25.0 and at(route, "lead_v", 2.5) == -1.5


def test_phase75_fields(route):
  assert not at(route, "lat_granted", 1.1) and at(route, "lat_granted", 1.3)
  assert at(route, "lat_reason", 0.9) == 0 and at(route, "lat_reason", 1.1) == 9 and at(route, "lat_reason", 1.3) == 0
  assert (route["gw_flags"] == 17).all() and (route["sf_cc"] == 4).all()
  assert at(route, "lat", 1.1) and not at(route, "lat", 0.9)


def test_fca_esa_bits_come_from_the_mdps_side_frame(route):
  assert at(route, "cb_esa", 1.0) == 0 and at(route, "cb_esa", 2.2) == 2 and at(route, "cb_esa", 2.7) == 0
  assert at(route, "cb_esa_gain", 2.2) == pytest.approx(0.5) and at(route, "cb_esa_gain", 1.0) == 0.0
  # the fields that were there before are untouched by the two new ones
  assert at(route, "cb_req", 1.0) == pytest.approx(-12.3) and at(route, "cb_gain", 1.0) == pytest.approx(0.6)
  assert at(route, "cb_aci", 1.0) == 2 and at(route, "cb_lvl2", 1.0) == 1


def test_camera_and_own_lkas_alt(route):
  assert at(route, "cam_ang", 1.0) == pytest.approx(4.5) and at(route, "cam_gain", 1.0) == pytest.approx(0.4)
  assert at(route, "cam_active", 1.0) == 2 and at(route, "cam_b13", 1.0) == 0xA5 and int(at(route, "cam_b7", 1.0)) & 1
  assert at(route, "wire", 1.0) == pytest.approx(-7.0) and at(route, "gain", 1.0) == pytest.approx(1.0)
  assert at(route, "tx_active", 1.0) == 2 and at(route, "tx_b7", 1.0) == 0


@pytest.mark.parametrize("name, values, parse, want", [
  ("ADAS_CMD_35_10ms", {"ADAS_StrAnglReqVal": 81.9, "ADAS_ACIAnglTqRedcGainVal": 1.0, "ADAS_ActvACISta": 15,
                        "ADAS_ActvACILvl2Sta": 0, "FCA_ESA_ActvSta": 1, "FCA_ESA_TqBstGainVal": 1.0},
   "parse_cb", (81.9, 1.0, 0, 15, 1, 1.0)),
  ("ADAS_CMD_35_10ms", {"ADAS_StrAnglReqVal": -81.9, "ADAS_ActvACILvl2Sta": 15, "FCA_ESA_ActvSta": 3},
   "parse_cb", (-81.9, 0.0, 15, 0, 3, 0.0)),
  ("LKAS_ALT", {"ADAS_StrAnglReqVal": -176.7, "ADAS_ACIAnglTqRedcGainVal": 0.2, "LKAS_ANGLE_ACTIVE": 1, "LKAS_BYTE13": 7,
                "LKAS_BYTE9_HIDDEN": 9}, "parse_lkas_alt", (-176.7, 0.2, 1, 0, 0, 7, 9)),
  ("MDPS", {"LKA_ANGLE_ACTIVE": 2, "LKA_ANGLE_FAULT": 1, "STEERING_OUT_TORQUE": -3.2, "STEERING_ANGLE": -45.6,
            "STEERING_COL_TORQUE": -120}, "parse_mdps", (2, 1, -3.2, -45.6, -120)),
])
def test_parsers_agree_with_the_dbc(name, values, parse, want):
  _, dat, _ = PACKER.make_can_msg(name, 0, values)
  assert getattr(rx, parse)(bytes(dat)) == pytest.approx(want, abs=1e-9)


def test_log_without_the_new_messages_keeps_defaults():
  d = rx.extract(m.as_reader() for m in build(v6=False))
  n = len(d["T"])
  assert n == N
  assert (d["mads_state"] == -1).all() and not d["mads_enabled"].any() and not d["mads_active"].any()
  assert (d["panda_ctrl"] == -1).all() and (d["panda_lat"] == -1).all()
  assert d["lat_granted"].all() and (d["lat_reason"] == 0).all()
  assert not d["lead"].any() and (d["lead_d"] == 0).all()
  assert d["buttons"].shape == (4, 3)   # carState.buttonEvents exist in every log
  assert (d["gw_flags"] == 17).all() and d["wire"][100] == pytest.approx(-7.0)


def test_message_of_an_older_schema_keeps_defaults():
  """carStateSP / carControlSP read with a schema that has no Phase 75 (or Phase 72) fields."""
  ex = rx.Extractor()
  cs = log.Event.new_message(); cs.logMonoTime = int(5e9); cs.init("carState")
  old_sp = SimpleNamespace(which=lambda: "carStateSP", logMonoTime=int(5e9), carStateSP=SimpleNamespace(steerFlags=3))
  old_cc = SimpleNamespace(which=lambda: "carControlSP", logMonoTime=int(5e9), carControlSP=SimpleNamespace())
  for m in (old_sp, old_cc, cs.as_reader()):
    ex.feed(m)
  d = ex.result()
  assert d["sf_cc"][0] == 3 and d["gw_flags"][0] == 0 and d["lat_reason"][0] == 0 and not d["lat_paused"][0]
  assert d["lat_granted"][0]


def test_no_car_state_gives_no_result(tmp_path, capsys):
  assert rx.extract([]) is None
  rx.main("00000000", str(tmp_path / "x.npz"), str(tmp_path))
  assert "no carState" in capsys.readouterr().out and not (tmp_path / "x.npz").exists()


def test_truncated_last_segment_does_not_lose_the_route(tmp_path):
  msgs = build()
  raw = zstandard.compress(b"".join(m.to_bytes() for m in msgs))
  (tmp_path / "000_000000fe--abc--0--rlog.zst").write_bytes(raw)
  (tmp_path / "000_000000fe--abc--1--rlog.zst").write_bytes(raw[:len(raw) // 3])   # cut mid-stream
  out = str(tmp_path / "o.npz")
  rx.main("000000fe", out, str(tmp_path))
  assert len(np.load(out, allow_pickle=True)["T"]) >= N
