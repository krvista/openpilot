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


def build(v6=True, v8=True, v9=True):
  """3 s: MADS enables at 1 s, the panda grants lateral at 1.2 s (latGranted follows), LFA button at 0.5 s, a lead from
  2 s, FCA_ESA active for 2.0-2.5 s, passive reason 9 (not granted) for 1.0-1.2 s, gear park -> drive at 0.3 s."""
  msgs = []
  e, x = ev(T0 - 0.005, "initData"); x.gitCommit = "0123456789abcdef"; x.gitBranch = "i6nv3"; x.dirty = False
  pr = x.params.init("entries", 2)
  pr[0].key = "GitDiff"; pr[0].value = b""; pr[1].key = "UpdaterCurrentDescription"; pr[1].value = b"v / i6nv3 / 0123456 / Nov 26"
  msgs.append(e)
  for i in range(N):
    t = T0 + i * 0.01; rel = i * 0.01
    e, s = ev(t, "carState")
    s.vEgo = 10.0; s.steeringAngleDeg = 2.0; s.cruiseState.enabled = rel >= 2.0; s.cruiseState.available = rel >= 0.8
    s.gearShifter = "drive" if rel >= 0.3 else "park"
    if i in (50, 55):
      b = s.init("buttonEvents", 1); b[0].type = "lkas"; b[0].pressed = i == 50
    if i == 80:
      b = s.init("buttonEvents", 2); b[0].type = "mainCruise"; b[0].pressed = True; b[1].type = "mainCruise"; b[1].pressed = False
    s.canValid = rel >= 0.05
    msgs.append(e)
    e, c = ev(t + 0.001, "carControl"); c.latActive = rel >= 1.0; c.actuators.steeringAngleDeg = 3.0; msgs.append(e)
    e, sp = ev(t + 0.002, "carStateSP"); sp.gatewayFlags = 17; sp.steerFlags = 4
    if v6:
      sp.lateralPausedReason = 9 if 1.0 <= rel < 1.2 else 0
    msgs.append(e)
    if v6:
      e, cs = ev(t + 0.001, "carControlSP"); cs.latGranted = rel >= 1.2; cs.gpsFixSeen = rel >= 2.1; msgs.append(e)
      e, ss = ev(t + 0.003, "selfdriveStateSP")
      ss.mads.state = "enabled" if rel >= 1.0 else "disabled"; ss.mads.enabled = rel >= 1.0; ss.mads.active = rel >= 1.2
      ss.mads.available = True; msgs.append(e)
      if i % 10 == 5:   # 10 Hz, first one at 0.05 s
        e, ps = ev(t, "pandaStates", 2)
        ps[0].safetyModel = "hyundaiCanfd"; ps[0].controlsAllowed = rel >= 2.0; ps[0].controlsAllowedLateral = rel >= 1.2
        ps[0].safetyRxChecksInvalid = 0.3 <= rel < 0.5
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
    if v8:
      # v8: 0x400 state (left 1 -> 2 at 1 s, off at 2 s, hazard-like 7 from 2.5 s) on buses 1 and 0, the lamps (0x3E3) flashing
      st = 1 if 0.5 <= rel < 1.0 else 2 if 1.0 <= rel < 2.0 else 7 if rel >= 2.5 else 0
      blk = bytearray(8); blk[4] = st | 0x10; lamp = bytearray(16); lamp[11] = (0x20 if st and i % 100 < 50 else 0) | (0x80 if st == 7 else 0)
      fr = [(0x400, bytes(blk), 1), (0x400, bytes(blk), 0), (0x3E3, bytes(lamp), 0)]
      if i == 0:
        fr = fr[:1]   # the first frame only on bus 1: the lower bus still wins afterwards
      e, cn = ev(t + 0.006, "can", len(fr)); can_frames(cn, fr); msgs.append(e)
      if i % 5 == 0:   # 20 Hz model, one late frame at 2 s
        e, md = ev(t + (0.03 if i == 200 else 0.0), "modelV2"); md.frameDropPerc = 3.0 if i == 200 else 0.0
        md.meta.desireState = [0.1, 0.0, 0.2, 0.6, 0.1, 0.0, 0.0, 0.0] if rel >= 1.0 else [1.0] + [0.0] * 7
        md.meta.laneChangeDirection = "left" if rel >= 1.0 else "none"; md.meta.laneChangeState = "laneChangeStarting" if rel >= 1.0 else "off"
        if v9:   # lane lines at the car -1.7 / +1.8 m (right line 0.1 m further out from 2 s), stds, probs, road edges, frame id
          md.frameId = 700 + i // 5; ll = md.init("laneLines", 4); re = md.init("roadEdges", 2)
          for ln, y in zip(ll, (-5.0, -1.7, 1.8 if rel < 2.0 else 1.9, 5.2), strict=True):
            ln.y = [y, y + 0.5]
          re[0].y = [-6.0, -6.5]; re[1].y = [7.0, 7.5]
          md.laneLineStds = [0.5, 0.05, 0.07, 0.6]; md.laneLineProbs = [0.2, 0.9, 0.8, 0.3]; md.roadEdgeStds = [0.9, 1.1]
        msgs.append(e)
        if v9:
          # locationd of this fork: liveLocationKalman (no livePose in the schema). Yaw rate 0.05 rad/s from 1 s, valid from 0.5 s
          e, k = ev(t + 0.002, "liveLocationKalman")
          k.angularVelocityCalibrated.value = [0.001, -0.002, 0.05 if rel >= 1.0 else 0.0]; k.angularVelocityCalibrated.valid = rel >= 0.5
          k.angularVelocityDevice.value = [9.0, 9.0, 9.0]   # the device frame is not what yaw is read from
          k.calibratedOrientationNED.value = [0.03, -0.01, 1.5]; msgs.append(e)
          # road camera encoder index: 20 fps, the file changes at 1.5 s (segment 1 starts at frame index 0), shutter 60 ms earlier
          e, x = ev(t + 0.004, "narrowRoadEncodeIdx"); x.frameId = 700 + i // 5; x.segmentNum = int(rel >= 1.5)
          x.segmentId = i // 5 - (30 if rel >= 1.5 else 0); x.timestampSof = int(round((t - 0.06) * 1e9)); msgs.append(e)
          e, vp = ev(t + 0.003, "vehicleParameters"); vp.steerRatio = 15.0; vp.roll = 0.02; vp.stiffnessFactor = 0.97; msgs.append(e)
      if i % 50 == 10:
        e, dv = ev(t, "deviceState"); dv.freeSpacePercent = 12.5 - rel; dv.cpuTempC = [50.0, 61.0]; dv.memoryUsagePercent = 60
        msgs.append(e)
        e, g = ev(t, "gpsLocationExternal"); g.hasFix = rel >= 2.0; g.latitude = 37.0; msgs.append(e)
      if i % 10 == 0:
        e, ss = ev(t, "selfdriveState"); ss.active = True
        if 1.5 <= rel < 2.0:
          ss.alertText1 = "Steering Paused"; ss.alertText2 = "Parking Mode"; ss.alertStatus = "userPrompt"
        msgs.append(e)
  return msgs


V8_NOT_SERIES = ("alerts", "events_sp", "invalid", "commits", "segs", "seg_err", "commit_full", "branch", "dirty",
                 "git_diff_len", "build_desc", "lane_turn")


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
  assert int(route["version"]) == rx.VERSION == 9
  assert len(route["T"]) == N and str(route["commit"]) == "012345678"
  n = len(route["T"])
  for k in route.files:
    if k not in ("commit", "passive", "events", "buttons", "version", *V8_NOT_SERIES):
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
  d = rx.extract(m.as_reader() for m in build(v6=False, v8=False))
  n = len(d["T"])
  # v9 defaults: no locationd message, no lane lines, no encoder index
  assert not d["yaw"].any() and (d["yaw_valid"] == -1).all() and not d["pose_roll"].any() and not d["lp_roll"].any()
  assert not d["ll_y_l"].any() and not d["ll_std_r"].any() and (d["model_frame"] == -1).all()
  assert (d["vid_seg"] == -1).all() and (d["vid_idx"] == -1).all() and (d["vid_t"] == -1).all()
  # v8 defaults: no 0x400 / 0x3E3 / deviceState / GPS / model message
  assert (d["blink_state"] == -1).all() and (d["lamp_l"] == -1).all() and (d["free_pct"] == -1).all()
  assert (d["gps_fix"] == -1).all() and (d["gps_seen"] == -1).all() and (d["rx_invalid"] == -1).all()
  assert not d["des_lc"].any() and d["alerts"].shape == (0, 4) and d["events_sp"].shape == (0, 2) and len(d["segs"]) == 0
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


def test_v7_factory_camera_lane_lines():
  d = bytearray(24)
  raw_l, raw_r = (-454) & 0x3FFF, 430                                  # -1.799 m / +1.704 m
  v = int.from_bytes(d, "little") | (raw_l << 29) | (raw_r << 101) | (3 << 24) | (2 << 96)
  left, right, ql, qr = rx.parse_fcam(v.to_bytes(24, "little"))
  assert left == pytest.approx(-454 * 0.0039625) and right == pytest.approx(430 * 0.0039625) and (ql, qr) == (3, 2)


def test_v8_blinker_state_and_lamps(route):
  """0x400 BLINKER_STATE from the lowest bus that carries it, the 0x3E3 lamp bits as they flash."""
  assert at(route, "blink_state", 0.2) == 0 and at(route, "blink_state", 0.7) == 1 and at(route, "blink_state", 1.5) == 2
  assert at(route, "blink_state", 2.2) == 0 and at(route, "blink_state", 2.7) == 7
  assert at(route, "lamp_l", 1.2) == 1 and at(route, "lamp_l", 1.7) == 0 and at(route, "lamp_l", 2.2) == 0   # flashing / off
  assert at(route, "lamp_r", 1.2) == 0 and at(route, "lamp_r", 2.7) == 1
  assert set(np.unique(route["blink_state"])) <= {-1, 0, 1, 2, 7}   # bit 4 of the byte is not part of the state


def test_v8_model_desire_and_timing(route):
  assert at(route, "des_lc", 0.5) == 0.0 and at(route, "des_lc", 1.5) == pytest.approx(0.7)
  assert at(route, "des_turn", 1.5) == pytest.approx(0.2) and at(route, "lc_dir", 1.5) == 1 and at(route, "lcs", 1.5) == 2
  assert route["frame_drop"].max() == pytest.approx(3.0) and route["model_dt"].max() == pytest.approx(0.08, abs=1e-6)
  assert at(route, "model_dt", 1.0) == pytest.approx(0.05, abs=1e-6)


def test_v8_health_series(route):
  assert not route["can_valid"][0] and route["can_valid"][10:].all()
  assert at(route, "free_pct", 0.05) == -1 and at(route, "free_pct", 0.2) == pytest.approx(12.4)
  assert at(route, "cpu_temp", 1.0) == pytest.approx(61.0) and at(route, "mem_pct", 1.0) == 60
  assert at(route, "gps_fix", 1.0) == 0 and at(route, "gps_fix", 2.2) == 1
  assert at(route, "gps_seen", 2.0) == 0 and at(route, "gps_seen", 2.2) == 1
  assert at(route, "rx_invalid", 0.2) == 0 and at(route, "rx_invalid", 0.4) == 1 and at(route, "rx_invalid", 0.6) == 0


def test_v8_alert_texts_and_build_identity(route):
  a = route["alerts"]
  assert a.shape == (1, 4) and a[0][0] == pytest.approx(1.505, abs=1e-6)
  assert tuple(a[0][1:]) == ("Steering Paused", "Parking Mode", "userPrompt")
  assert str(route["commit_full"]) == "0123456789abcdef" and str(route["branch"]) == "i6nv3" and int(route["dirty"]) == 0
  assert int(route["git_diff_len"]) == 0 and str(route["build_desc"]).endswith("Nov 26")
  assert list(route["commits"]) == ["0123456789abcdef"] and list(route["segs"]) == [0, 1] and len(route["seg_err"]) == 0


def test_v8_truncated_segment_is_listed(tmp_path):
  plain = b"".join(m.to_bytes() for m in build())
  (tmp_path / "000_000000fd--abc--0--rlog.zst").write_bytes(zstandard.compress(plain))
  (tmp_path / "000_000000fd--abc--1--rlog.zst").write_bytes(zstandard.compress(plain[:len(plain) // 3 + 5]))   # cut inside a message
  out = str(tmp_path / "o.npz")
  rx.main("000000fd", out, str(tmp_path))
  d = np.load(out, allow_pickle=True)
  assert list(d["segs"]) == [0, 1] and list(d["seg_err"]) == [1]


def test_v9_yaw_rate_comes_from_live_location_kalman(route):
  """Up to v8 yaw was read from livePose only - a message this schema does not have - and stayed 0 on every route."""
  assert "livePose" not in log.Event.schema.union_fields and "liveLocationKalman" in log.Event.schema.union_fields
  assert at(route, "yaw", 0.5) == pytest.approx(0.0, abs=1e-9) and at(route, "yaw", 2.0) == pytest.approx(np.degrees(0.05))
  assert at(route, "yaw_valid", 0.3) == 0 and at(route, "yaw_valid", 0.7) == 1 and route["yaw_valid"][0] == -1
  # yaw [rad/s] / v [m/s] is a curvature with the sign of k_meas: 0.05 / 10 m/s
  assert np.radians(at(route, "yaw", 2.0)) / (route["v"][250] / 3.6) == pytest.approx(5.0e-3)
  assert at(route, "pose_roll", 1.0) == pytest.approx(0.03) and at(route, "pose_pitch", 1.0) == pytest.approx(-0.01)
  assert at(route, "lp_roll", 1.0) == pytest.approx(0.02) and at(route, "lp_stiff", 1.0) == pytest.approx(0.97)
  assert at(route, "lp_sr", 1.0) == pytest.approx(15.0)


def test_v9_live_pose_is_read_where_a_log_has_it():
  ex = rx.Extractor()
  cs = log.Event.new_message(); cs.logMonoTime = int(5e9); cs.init("carState")
  pose = SimpleNamespace(which=lambda: "livePose", logMonoTime=int(4.99e9),
                         livePose=SimpleNamespace(angularVelocityDevice=SimpleNamespace(z=0.02, valid=True)))
  for m in (pose, cs.as_reader()):
    ex.feed(m)
  d = ex.result()
  assert d["yaw"][0] == pytest.approx(np.degrees(0.02)) and d["yaw_valid"][0] == 1


def test_v9_model_lane_lines_at_the_car(route):
  assert at(route, "ll_y_l", 1.0) == pytest.approx(-1.7) and at(route, "ll_y_r", 1.0) == pytest.approx(1.8)
  assert at(route, "ll_y_r", 2.5) == pytest.approx(1.9)
  assert at(route, "ll_std_l", 1.0) == pytest.approx(0.05) and at(route, "ll_std_r", 1.0) == pytest.approx(0.07)
  assert at(route, "ll_p_l", 1.0) == pytest.approx(0.9) and at(route, "ll_p_r", 1.0) == pytest.approx(0.8)
  assert at(route, "re_y_l", 1.0) == pytest.approx(-6.0) and at(route, "re_y_r", 1.0) == pytest.approx(7.0)
  assert at(route, "re_std_l", 1.0) == pytest.approx(0.9) and at(route, "re_std_r", 1.0) == pytest.approx(1.1)
  assert at(route, "lane_min", 1.0) == pytest.approx(0.8)       # unchanged since v1: the weaker of the two lane lines
  assert at(route, "model_frame", 1.0) == 720


def test_v9_road_camera_video_alignment(route):
  """vid_seg / vid_idx name the frame inside the road camera file, vid_t is its shutter time on the T axis."""
  assert at(route, "vid_seg", 1.0) == 0 and at(route, "vid_idx", 1.0) == 20 and at(route, "vid_frame", 1.0) == 720
  assert at(route, "vid_seg", 1.6) == 1 and at(route, "vid_idx", 1.6) == 2 and at(route, "vid_frame", 1.6) == 732
  assert at(route, "vid_t", 1.0) == pytest.approx(1.0 - 0.06 + 0.005, abs=1e-6)   # T starts at the initData message, 5 ms early
  assert at(route, "vid_frame", 1.0) == at(route, "model_frame", 1.0)


def test_v9_log_of_v8_content_keeps_the_v8_series():
  d = rx.extract(m.as_reader() for m in build(v9=False))
  assert at(d, "des_lc", 1.5) == pytest.approx(0.7) and not d["yaw"].any() and (d["vid_seg"] == -1).all()
  assert not d["ll_y_l"].any() and (d["model_frame"] == -1).all() and at(d, "lane_min", 1.0) == 1.0
