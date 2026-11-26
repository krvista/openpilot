"""One-pass route extractor (09-23; v2 10-02; v6 11-23; v8 11-26): read a route's rlogs ONCE and save every 100 Hz series the i6nv3 probes
need, so routes can be deleted from the disk-limited container right after extraction and analysed offline.

Saved (carState-rate grid, T relative to the first log message):
  v (km/h), a_ego, ang, rate, pressed, tq_s, tq_eps, lb, rb, gas, brake, bsl, bsr, lat, apply, accel_cmd, wire (LKAS_ALT
  angle), gain (ACI), tx_active (LKAS_ALT LKAS_ANGLE_ACTIVE), k_model, k_ctrl, k_meas, gap (steerCmdGapDeg), lane_min, lcs,
  yaw (deg/s), mdps_active (0xEA LKA_ANGLE_ACTIVE), mdps_fault, mdps_tq (STEERING_OUT_TORQUE), tx_reject (0x110 echoes
  with src >= 192 per frame), sd_active (selfdriveState.active), gps_lat/lon,
  sf_cc (carStateSP.steerFlags, CarController SF_* bits) / sf_ctl (controlsState.steerFlags, STEER_FLAG_* bits) — Phase 46,
  0 on older builds
  events: list of (t, name) for onroadEvents transitions; commit
  v3 (11-21, stock-LFA capture drives): cam_ang / cam_gain / cam_active = the factory camera's OWN LKAS_ALT (bus 2: request
  angle, ACI gain, LKAS_ANGLE_ACTIVE) and cam_b7 / cam_b8 / cam_b13 / cam_b9lo (its other bytes) - in a capture drive this is
  what steers the car; mdps_ang (MDPS STEERING_ANGLE), mdps_col (column torque), cruise (cruiseState.enabled), passive
  (CarParams.passive)
  v4 (11-21): cb_req / cb_gain / cb_lvl2 / cb_aci = ADAS_CMD_35_10ms (0xCB, bus 1) - the angle request, ACI gain and state the
  MDPS actually RECEIVES from the vehicle's ADAS controller (op's LKAS_ALT on the camera bus is only an input to that
  controller); lfa_sym / hda_icon = ADAS_CMD_31_50ms (0x1E0, bus 1) HDA_LFA_SymSta (0 off, 1 standby, 2 active, 3) and HDA_ICON
  - the factory LFA / HDA state; lane_l / lane_r = LEFT / RIGHT_LANE_LINE of the camera's CAM_0x362 (bus 2), lane_tx_l /
  lane_tx_r = the same two fields in the copy op sends to the ADAS controller (-1 when op sends none); tx_b7 = byte 7 of
  the LKAS_ALT op transmits (bit 0 = LFA button bit: op's 'MADS <-> cluster sync pulse'; cam_b7 bit 0 = the camera's)
  v5 (11-22, Phase 72 drives): gw_flags = carStateSP.gatewayFlags (GW_* bits of the car controller: 1 valid, 2 factory LFA
  on, 4 active, 8 relayed, 16 solo profile, 32 button pulse, 64 follow band, 128 hold-off paused; 0 in pre-72 logs),
  lat_paused = carStateSP.lateralControlPaused
  v6 (11-23, Phase 75): mads_state (selfdriveStateSP.mads.state: 0 disabled, 1 paused, 2 enabled, 3 softDisabling,
  4 overriding; -1 = no message) / mads_enabled / mads_active / mads_available; panda_ctrl / panda_lat = pandaStates
  controlsAllowed / controlsAllowedLateral over the pandas in a car safety mode (1 all allow, 0 one does not, -1 unknown);
  buttons = carState.buttonEvents as rows (t, type, pressed), type = car.capnp ButtonEvent.Type (5 cancel, 6 lkas,
  8 mainCruise, 9 setCruise, 10 resumeCruise, ...); gear (carState.gearShifter: 1 park, 2 drive, 3 neutral, 4 reverse);
  lead / lead_d / lead_v = radarState.leadOne present / dRel (m) / vRel (m/s); cruise_avail (cruiseState.available);
  lat_granted (carControlSP.latGranted; True in pre-75 logs), lat_reason (carStateSP.lateralPausedReason, car controller
  PR_*: 1 camera stale, 2 LFA fault, 3 reverse, 4 parking, 5 angle, 6 low-speed grip, 7 scenario, 8 envelope, 9 not
  granted, 10 wheel outrun; 0 = not passive / pre-75); cb_esa / cb_esa_gain = FCA_ESA_ActvSta / FCA_ESA_TqBstGainVal of
  ADAS_CMD_35_10ms (emergency steering assist state and torque boost gain); version = 6 (absent before v6).
  The camera's own LKAS_ALT active / gain / byte 13 are cam_active / cam_gain / cam_b13 (since v3).
  v7 (11-24): fcam_l / fcam_r = the factory camera's own lane-line positions (FR_CMR_03_50ms 0x1B5 Info_LftLnPosVal /
  Info_RtLnPosVal, m from the car; bus 1 preferred) with fcam_ql / fcam_qr (quality 0-7; -1 = no message) - the car's real
  lane position is -(fcam_l + fcam_r) / 2 (+ = right of centre, as E163); lp_sr / lp_ao / lp_aoa = liveParameters steerRatio /
  angleOffsetDeg / angleOffsetAverageDeg; cal_roll / cal_pitch / cal_yaw = liveCalibration rpyCalib (rad).
  v8 (11-26, next-drive check E179 / report 117): blink_state = BLINKERS_ALT (0x400, lowest bus seen) BLINKER_STATE (byte 4
  bits 0-2: 0 off, 1-3 left, 4-6 right; -1 = no message), lamp_l / lamp_r = body controller 0x3E3 byte 11 bit 5 / bit 7
  (the lamps themselves, flashing; -1 = no message); des_turn / des_lc = modelV2.meta.desireState turn (1 + 2) / lane change
  (3 + 4) share, lc_dir = meta.laneChangeDirection (0 none, 1 left, 2 right), frame_drop = modelV2.frameDropPerc,
  model_dt = time since the previous modelV2 (s); can_valid = carState.canValid; free_pct = deviceState.freeSpacePercent
  (-1 = no message), cpu_temp (max core, C), mem_pct; gps_fix = gpsLocationExternal.hasFix, gps_seen =
  carControlSP.gpsFixSeen (both -1 = no message; the schema default of gps_seen is True, so pre-76 logs read 1);
  rx_invalid = any panda safetyRxChecksInvalid (-1 = no message); alerts = rows (t, alertText1, alertText2, status) of
  selfdriveState alert text changes (non-empty), events_sp = (t, name) of onroadEventsSP transitions; invalid = rows
  (service, frames with valid False, first t, last t); build: commit_full / branch / dirty (-1 unknown) / git_diff_len
  (length of the GitDiff param, -1 unknown) / build_desc (UpdaterCurrentDescription) from the FIRST initData, commits = every
  distinct gitCommit over all initData (a route that spans a reboot), lane_turn = "LaneTurnDesire,LaneTurnValue" params;
  segs = segment numbers read by main() (empty from extract()), seg_err = those that raised (truncated).
  Fields a log does not have keep their defaults. extract(msgs) / parse_* are importable (phase_tests/test_phase75_extract.py).
Usage: PYTHONPATH=... python tools/i6nv3_bench/route_extract.py <route> <out.npz> [log dir]
"""
import glob
import sys
import warnings

import numpy as np

from openpilot.tools.lib.logreader import LogReader

VERSION = 8
LOG_DIR = "/home/user/drivelog/drivelog"
NO_CAR_SAFETY = ("silent", "noOutput")   # pandas in these modes say nothing about the car (as the MADS mismatch check)


def _s14(raw):
  raw &= 0x3FFF
  return raw - 0x4000 if raw & 0x2000 else raw


def parse_mdps(dd):
  """MDPS (0xEA): LKA_ANGLE_ACTIVE, LKA_ANGLE_FAULT, STEERING_OUT_TORQUE, STEERING_ANGLE (deg), STEERING_COL_TORQUE."""
  a16 = dd[12] | (dd[13] << 8)
  return (dd[18] & 0x3, (dd[18] >> 5) & 1, ((dd[8] | (dd[9] << 8)) & 0xFFF) * 0.1 - 204.8,
          (a16 - 0x10000 if a16 & 0x8000 else a16) * 0.1, ((dd[10] | (dd[11] << 8)) & 0x1FFF) - 4095)


def _sbits(d, start, size):
  v = (int.from_bytes(d, "little") >> start) & ((1 << size) - 1)
  return v - (1 << size) if v & (1 << (size - 1)) else v


def parse_fcam(dd):
  """FR_CMR_03_50ms (0x1B5): left / right lane-line position (m), left / right quality."""
  return _sbits(dd, 29, 14) * 0.0039625, _sbits(dd, 101, 14) * 0.0039625, dd[3] & 7, dd[12] & 7


def parse_cb(dd):
  """ADAS_CMD_35_10ms (0xCB): ADAS_StrAnglReqVal (deg), ACI gain, ADAS_ActvACILvl2Sta, ADAS_ActvACISta,
  FCA_ESA_ActvSta, FCA_ESA_TqBstGainVal (v6)."""
  return _s14(dd[4] | (dd[5] << 8)) * 0.1, dd[6] * 0.004, dd[3] >> 4, dd[3] & 0xF, dd[7] & 0x3, dd[8] * 0.004


def parse_lkas_alt(dd):
  """LKAS_ALT (0x110): angle (deg), ACI gain, LKAS_ANGLE_ACTIVE, byte 7, byte 8, byte 13 (0 if short), byte 9 low nibble."""
  return (_s14((dd[10] >> 2) | (dd[11] << 6)) * 0.1, dd[12] * 0.004, (dd[9] >> 4) & 0x3, dd[7], dd[8],
          dd[13] if len(dd) > 13 else 0, dd[9] & 0xF)


def panda_allowed(panda_states):
  """(controlsAllowed, controlsAllowedLateral) over the pandas in a car safety mode: 1 all, 0 not all, -1 no such panda."""
  car = [ps for ps in panda_states if str(ps.safetyModel) not in NO_CAR_SAFETY]
  if not car:
    return -1, -1
  return int(all(ps.controlsAllowed for ps in car)), int(all(getattr(ps, "controlsAllowedLateral", True) for ps in car))


class Extractor:
  """feed() every log message in order, then result() -> dict of arrays (None without carState)."""

  def __init__(self):
    (self.cs, self.cc, self.wr, self.ct, self.md, self.lp, self.gp, self.mp, self.rj, self.sd, self.sp, self.cam, self.cbm,
     self.e0m, self.lnc, self.lnt) = ([] for _ in range(16))
    self.mads, self.pnd, self.btn, self.ld, self.ccsp = [], [], [], [], []   # v6
    self.fcam, self.lpar, self.cal = [], [], []   # v7
    self.fcam_bus1 = False
    # v8
    self.blk, self.lamp, self.mdl, self.dev, self.fix, self.rx, self.alerts, self.events_sp = ([] for _ in range(8))
    self.blk_bus = None; self.prev_alert = None; self.prev_ev_sp = set(); self.last_model = None
    self.invalid = {}; self.commits = []; self.build = dict(commit_full="", branch="", dirty=-1, git_diff_len=-1, build_desc="", lane_turn="")
    self.segs, self.seg_err = [], []
    self.passive = False
    self.events = []; self.prev_ev = set()
    self.t0 = None; self.commit = ""

  def feed(self, m):
    w = m.which(); t = m.logMonoTime * 1e-9
    if self.t0 is None:
      self.t0 = t
    if not getattr(m, "valid", True):
      c = self.invalid.setdefault(w, [0, t - self.t0, t - self.t0]); c[0] += 1; c[2] = t - self.t0
    if w == "carState":
      s = m.carState
      self.cs.append((t, s.vEgo * 3.6, s.aEgo, s.steeringAngleDeg, s.steeringRateDeg, s.steeringPressed, s.steeringTorque,
                      s.steeringTorqueEps, s.leftBlinker, s.rightBlinker, s.gasPressed, s.brakePressed, s.leftBlindspot,
                      s.rightBlindspot, s.cruiseState.enabled, s.cruiseState.available, int(s.gearShifter.raw),
                      getattr(s, "canValid", True)))
      for b in s.buttonEvents:
        self.btn.append((t - self.t0, int(b.type.raw), bool(b.pressed)))
    elif w == "carControl":
      c = m.carControl
      self.cc.append((t, c.latActive, c.actuators.steeringAngleDeg, c.actuators.accel))
    elif w == "controlsState":
      x = m.controlsState
      self.ct.append((t, x.desiredCurvature, x.curvature, getattr(x, "steerCmdGapDeg", 0.0), getattr(x, "steerFlags", 0)))
    elif w == "carStateSP":
      self.sp.append((t, getattr(m.carStateSP, "steerFlags", 0), getattr(m.carStateSP, "gatewayFlags", 0),
                      getattr(m.carStateSP, "lateralControlPaused", False), getattr(m.carStateSP, "lateralPausedReason", 0)))
    elif w == "carControlSP":
      self.ccsp.append((t, getattr(m.carControlSP, "latGranted", True), getattr(m.carControlSP, "gpsFixSeen", -1)))
    elif w == "selfdriveStateSP":
      x = m.selfdriveStateSP.mads
      self.mads.append((t, int(x.state.raw), x.enabled, x.active, x.available))
    elif w == "pandaStates":
      self.pnd.append((t, *panda_allowed(m.pandaStates)))
      self.rx.append((t, any(getattr(ps, "safetyRxChecksInvalid", False) for ps in m.pandaStates)))
    elif w == "radarState":
      x = m.radarState.leadOne
      self.ld.append((t, x.present, x.dRel, x.vRel))
    elif w == "modelV2":
      d = m.modelV2
      lm = min(d.laneLineProbs[1], d.laneLineProbs[2]) if len(d.laneLineProbs) >= 4 else 1.0
      self.md.append((t, d.action.desiredCurvature, lm, int(d.meta.laneChangeState.raw)))
      ds = list(d.meta.desireState)
      self.mdl.append((t, ds[1] + ds[2] if len(ds) > 4 else 0.0, ds[3] + ds[4] if len(ds) > 4 else 0.0,
                       int(d.meta.laneChangeDirection.raw), d.frameDropPerc, t - self.last_model if self.last_model else 0.0))
      self.last_model = t
    elif w == "livePose":
      self.lp.append((t, np.degrees(m.livePose.angularVelocityDevice.z)))
    elif w == "selfdriveState":
      self.sd.append((t, m.selfdriveState.active))
      a = (m.selfdriveState.alertText1, m.selfdriveState.alertText2, str(m.selfdriveState.alertStatus))
      if a != self.prev_alert:
        if a[0] or a[1]:
          self.alerts.append((t - self.t0, *a))
        self.prev_alert = a
    elif w == "onroadEventsSP":
      cur = {str(e.name) for e in m.onroadEventsSP.events}
      for nm in cur - self.prev_ev_sp:
        self.events_sp.append((t - self.t0, nm))
      self.prev_ev_sp = cur
    elif w == "deviceState":
      x = m.deviceState
      self.dev.append((t, x.freeSpacePercent, max(list(x.cpuTempC) + [0.0]), x.memoryUsagePercent))
    elif w == "onroadEvents":
      cur = {str(e.name) for e in m.onroadEvents}
      for nm in cur - self.prev_ev:
        self.events.append((t - self.t0, nm))
      self.prev_ev = cur
    elif w in ("gpsLocationExternal", "gpsLocation"):
      g = getattr(m, w)
      if g.latitude != 0.0:
        self.gp.append((t, g.latitude, g.longitude))
      if w == "gpsLocationExternal":
        self.fix.append((t, bool(getattr(g, "hasFix", False))))
    elif w == "initData":
      x = m.initData
      if x.gitCommit not in self.commits:
        self.commits.append(x.gitCommit)
      if not self.commit:
        self.commit = x.gitCommit[:9]
        b = self.build; b["commit_full"] = x.gitCommit; b["branch"] = x.gitBranch; b["dirty"] = int(bool(x.dirty))
        try:
          P = {e.key: bytes(e.value) for e in x.params.entries}
          b["git_diff_len"] = len(P["GitDiff"]) if "GitDiff" in P else -1
          b["build_desc"] = P.get("UpdaterCurrentDescription", b"").decode(errors="replace")
          b["lane_turn"] = ",".join(P.get(k, b"").decode(errors="replace") for k in ("LaneTurnDesire", "LaneTurnValue"))
        except Exception:  # noqa: BLE001 - params are optional
          pass
    elif w in ("liveParameters", "vehicleParameters"):       # the schema's name differs by version
      lp = getattr(m, w)
      self.lpar.append((t, lp.steerRatio, lp.angleOffsetDeg, lp.angleOffsetAverageDeg))
    elif w in ("liveCalibration", "extrinsicsCalibration"):
      r = list(getattr(m, w).rpyCalib)
      if len(r) == 3:
        self.cal.append((t, *r))
    elif w == "carParams":
      self.passive = bool(m.carParams.passive)
    elif w == "can":
      nrej = 0
      for c in m.can:
        if c.address == 234 and c.src < 128 and len(c.dat) >= 24:
          self.mp.append((t, *parse_mdps(bytes(c.dat))))
        elif c.address == 272 and c.src >= 192:
          nrej += 1
        elif c.address == 203 and c.src == 1 and len(c.dat) >= 9:
          self.cbm.append((t, *parse_cb(bytes(c.dat))))
        elif c.address == 480 and c.src == 1 and len(c.dat) >= 8:
          bits = int.from_bytes(bytes(c.dat)[:8], "little")
          self.e0m.append((t, (bits >> 47) & 3, (bits >> 31) & 1))
        elif c.address == 272 and c.src == 2 and len(c.dat) >= 14:
          self.cam.append((t, *parse_lkas_alt(bytes(c.dat))))
        elif c.address == 437 and c.src in (0, 1, 2) and len(c.dat) >= 20:
          if c.src == 1 or not self.fcam_bus1:
            self.fcam_bus1 = self.fcam_bus1 or c.src == 1
            self.fcam.append((t, *parse_fcam(bytes(c.dat))))
        elif c.address == 866 and c.src == 2 and len(c.dat) >= 8:
          self.lnc.append((t, c.dat[7] & 3, (c.dat[7] >> 4) & 3))
        elif c.address == 0x400 and c.src in (0, 1) and len(c.dat) >= 5:
          if self.blk_bus is None or c.src < self.blk_bus:
            self.blk_bus = c.src
          if c.src == self.blk_bus:
            self.blk.append((t, c.dat[4] & 7))
        elif c.address == 0x3E3 and c.src < 128 and len(c.dat) >= 12:
          self.lamp.append((t, (c.dat[11] >> 5) & 1, (c.dat[11] >> 7) & 1))
      if nrej:
        self.rj.append((t, nrej))
    elif w == "sendcan":
      for c in m.sendcan:
        if c.address == 272 and len(c.dat) >= 13:
          self.wr.append((t, *parse_lkas_alt(bytes(c.dat))[:4]))
        elif c.address == 866 and len(c.dat) >= 8:
          self.lnt.append((t, c.dat[7] & 3, (c.dat[7] >> 4) & 3))

  def result(self):
    CS = np.array(self.cs, float)
    if not len(CS):
      return None
    T = CS[:, 0]; t0 = self.t0

    def res(a, col, default=0.0, kind="interp"):
      A = np.array(a, float)
      if not len(A):
        return np.full(len(T), default)
      if kind in ("hold", "since"):
        i = np.searchsorted(A[:, 0], T, side="right") - 1
        y = A[np.clip(i, 0, len(A) - 1), col]
        return np.where(i < 0, default, y) if kind == "since" else y   # since: the default until the first sample
      return np.interp(T, A[:, 0], A[:, col])
    rej = np.zeros(len(T))
    if self.rj:
      RJ = np.array(self.rj, float); idx = np.clip(np.searchsorted(T, RJ[:, 0]), 0, len(T) - 1); np.add.at(rej, idx, RJ[:, 1])
    cc, wr, ct, md, lp, gp, mp, sd, sp = self.cc, self.wr, self.ct, self.md, self.lp, self.gp, self.mp, self.sd, self.sp
    cam, cbm, e0m, lnc, lnt = self.cam, self.cbm, self.e0m, self.lnc, self.lnt
    return dict(
      T=T - t0, v=CS[:, 1], a_ego=CS[:, 2], ang=CS[:, 3], rate=CS[:, 4], pressed=CS[:, 5] > 0.5,
      tq_s=CS[:, 6], tq_eps=CS[:, 7], lb=CS[:, 8] > 0.5, rb=CS[:, 9] > 0.5, gas=CS[:, 10] > 0.5,
      brake=CS[:, 11] > 0.5, bsl=CS[:, 12] > 0.5, bsr=CS[:, 13] > 0.5,
      lat=res(cc, 1, kind="hold") > 0.5, apply=res(cc, 2), accel_cmd=res(cc, 3),
      wire=res(wr, 1), gain=res(wr, 2), tx_active=res(wr, 3, kind="hold"),
      k_ctrl=res(ct, 1), k_meas=res(ct, 2), gap=res(ct, 3), k_model=res(md, 1),
      lane_min=res(md, 2, 1.0), lcs=res(md, 3, kind="hold"), yaw=res(lp, 1),
      mdps_active=res(mp, 1, kind="hold"), mdps_fault=res(mp, 2, kind="hold"), mdps_tq=res(mp, 3),
      tx_reject=rej, sd_active=res(sd, 1, kind="hold") > 0.5,
      gps_lat=res(gp, 1), gps_lon=res(gp, 2), commit=np.array(self.commit),
      sf_cc=res(sp, 1, kind="hold").astype(np.int64), sf_ctl=res(ct, 4, kind="hold").astype(np.int64),
      gw_flags=res(sp, 2, kind="hold").astype(np.int64), lat_paused=res(sp, 3, kind="hold") > 0.5,
      cam_ang=res(cam, 1), cam_gain=res(cam, 2, kind="hold"), cam_active=res(cam, 3, kind="hold"),
      cam_b7=res(cam, 4, kind="hold"), cam_b8=res(cam, 5, kind="hold"), cam_b13=res(cam, 6, kind="hold"),
      cam_b9lo=res(cam, 7, kind="hold"), mdps_ang=res(mp, 4), mdps_col=res(mp, 5),
      cb_req=res(cbm, 1), cb_gain=res(cbm, 2), cb_lvl2=res(cbm, 3, kind="hold"), cb_aci=res(cbm, 4, kind="hold"),
      lfa_sym=res(e0m, 1, -1.0, kind="hold"), hda_icon=res(e0m, 2, -1.0, kind="hold"),
      lane_l=res(lnc, 1, -1.0, kind="hold"), lane_r=res(lnc, 2, -1.0, kind="hold"),
      lane_tx_l=res(lnt, 1, -1.0, kind="hold"), lane_tx_r=res(lnt, 2, -1.0, kind="hold"),
      tx_b7=res(wr, 4, kind="hold"),
      cruise=CS[:, 14] > 0.5, passive=np.array(self.passive),
      events=np.array(self.events, dtype=object),
      # v6
      version=np.array(VERSION), cruise_avail=CS[:, 15] > 0.5, gear=CS[:, 16].astype(np.int64),
      mads_state=res(self.mads, 1, -1.0, kind="since").astype(np.int64), mads_enabled=res(self.mads, 2, kind="since") > 0.5,
      mads_active=res(self.mads, 3, kind="since") > 0.5, mads_available=res(self.mads, 4, kind="since") > 0.5,
      panda_ctrl=res(self.pnd, 1, -1.0, kind="since").astype(np.int64),
      panda_lat=res(self.pnd, 2, -1.0, kind="since").astype(np.int64),
      buttons=np.array(self.btn, float).reshape(-1, 3),
      lead=res(self.ld, 1, kind="since") > 0.5, lead_d=res(self.ld, 2, kind="since"), lead_v=res(self.ld, 3, kind="since"),
      lat_granted=res(self.ccsp, 1, 1.0, kind="since") > 0.5, lat_reason=res(sp, 4, kind="since").astype(np.int64),
      cb_esa=res(cbm, 5, kind="since").astype(np.int64), cb_esa_gain=res(cbm, 6, kind="since"),
      # v7
      fcam_l=res(self.fcam, 1, kind="since"), fcam_r=res(self.fcam, 2, kind="since"),
      fcam_ql=res(self.fcam, 3, -1.0, kind="since").astype(np.int64), fcam_qr=res(self.fcam, 4, -1.0, kind="since").astype(np.int64),
      lp_sr=res(self.lpar, 1, kind="since"), lp_ao=res(self.lpar, 2, kind="since"), lp_aoa=res(self.lpar, 3, kind="since"),
      cal_roll=res(self.cal, 1, kind="since"), cal_pitch=res(self.cal, 2, kind="since"), cal_yaw=res(self.cal, 3, kind="since"),
      # v8
      blink_state=res(self.blk, 1, -1.0, kind="since").astype(np.int64),
      lamp_l=res(self.lamp, 1, -1.0, kind="since").astype(np.int64), lamp_r=res(self.lamp, 2, -1.0, kind="since").astype(np.int64),
      des_turn=res(self.mdl, 1, kind="since"), des_lc=res(self.mdl, 2, kind="since"),
      lc_dir=res(self.mdl, 3, kind="since").astype(np.int64), frame_drop=res(self.mdl, 4, kind="since"),
      model_dt=res(self.mdl, 5, kind="since"), can_valid=CS[:, 17] > 0.5,
      free_pct=res(self.dev, 1, -1.0, kind="since"), cpu_temp=res(self.dev, 2, kind="since"), mem_pct=res(self.dev, 3, kind="since"),
      gps_fix=res(self.fix, 1, -1.0, kind="since").astype(np.int64),
      gps_seen=res(self.ccsp, 2, -1.0, kind="since").astype(np.int64),
      rx_invalid=res(self.rx, 1, -1.0, kind="since").astype(np.int64),
      alerts=np.array(self.alerts, dtype=object).reshape(-1, 4), events_sp=np.array(self.events_sp, dtype=object).reshape(-1, 2),
      invalid=np.array([(k, *c) for k, c in sorted(self.invalid.items())], dtype=object).reshape(-1, 4),
      commits=np.array(self.commits, dtype=str), segs=np.array(self.segs, np.int64), seg_err=np.array(self.seg_err, np.int64),
      **{k: np.array(x) for k, x in self.build.items()})


def extract(msgs):
  ex = Extractor()
  for m in msgs:
    ex.feed(m)
  return ex.result()


def main(r, out, log_dir=LOG_DIR):
  files = sorted(glob.glob(f"{log_dir}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
  ex = Extractor()
  for f in files:
    ex.segs.append(int(f.split("--")[-2]))
    try:
      with warnings.catch_warnings(record=True) as wn:   # LogReader only warns on a truncated segment
        warnings.simplefilter("always")
        for m in LogReader(f):
          ex.feed(m)
      if any("Corrupted" in str(x.message) for x in wn):
        ex.seg_err.append(ex.segs[-1])
    except Exception as e:  # noqa: BLE001 - a truncated last segment must not lose the route
      ex.seg_err.append(ex.segs[-1])
      print(f"  {f.split('/')[-1]}: {type(e).__name__}: {e}")
  d = ex.result()
  if d is None:
    print(f"== {r}: no carState"); return
  np.savez_compressed(out, **d)
  print(f"== {r}: {len(d['T']) / 6000:.1f} min, commit {ex.commit} ({ex.build['branch']}, dirty {ex.build['dirty']}), "
        f"segments {len(ex.segs)} (truncated {ex.seg_err}), lat active {np.mean(d['lat']) * 100:.0f}%, events {len(d['events'])}")


if __name__ == "__main__":
  main(*sys.argv[1:4])
