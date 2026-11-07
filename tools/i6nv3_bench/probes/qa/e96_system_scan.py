"""E96: whole-system scan of a route's raw rlogs (beyond steering): alert / event onsets, engagement changes, device
thermal / CPU / memory, model frame drops / execution time, control-loop timing gaps, panda faults / CAN / safety,
calibration and live parameters, steering faults, longitudinal (FCW, accel tracking), process restarts, error logs."""
import glob, sys, pickle
from collections import Counter, defaultdict
import numpy as np
from openpilot.tools.lib.logreader import LogReader
D, OUT, r = sys.argv[1], sys.argv[2], sys.argv[3]
fs = sorted(glob.glob(f"{D}/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
R = defaultdict(list); C = Counter(); EV = Counter(); AL = Counter(); ERR = Counter(); PROC = Counter()
prev_ev = set(); prev_alert = None; prev_en = None; last = {}; running = {}
for f in fs:
  try:
    for m in LogReader(f):
     try:
      w = m.which(); t = m.logMonoTime * 1e-9
      if w in ("controlsState", "carControl", "carState", "modelV2", "deviceState", "pandaStates"):
        if w in last and w in ("controlsState", "carControl", "carState"):
          R[w + "_gap"].append(t - last[w])
        last[w] = t
      if w == "onroadEvents":
        cur = {str(e.name) for e in m.onroadEvents}
        for e in cur - prev_ev:
          EV[e] += 1
        prev_ev = cur
      elif w == "onroadEventsSP":
        for e in m.onroadEventsSP:
          EV["SP:" + str(e.name)] += 0   # presence only
      elif w == "selfdriveState":
        ss = m.selfdriveState
        a = ss.alertType
        if a and a != prev_alert:
          AL[a] += 1
        prev_alert = a
        if prev_en is not None and ss.enabled != prev_en:
          C["engage" if ss.enabled else "disengage"] += 1
        prev_en = ss.enabled
      elif w == "deviceState":
        d = m.deviceState
        R["temp"].append(d.maxTempC); R["cpu"].append(np.mean(list(d.cpuUsagePercent)) if len(d.cpuUsagePercent) else np.nan)
        R["mem"].append(d.memoryUsagePercent); C["thermal_" + str(d.thermalStatus)] += 1; R["fan"].append(d.fanSpeedPercentDesired)
        R["free"].append(d.freeSpacePercent)
      elif w == "modelV2":
        R["drop"].append(m.modelV2.frameDropPerc); R["mexec"].append(m.modelV2.modelExecutionTime)
      elif w == "pandaStates":
        for p in m.pandaStates:
          for fl in p.faults:
            C["panda_fault_" + str(fl)] += 1
          if p.safetyRxChecksInvalid: C["rx_checks_invalid"] += 1
          C["safety_" + str(p.safetyModel)] += 1
          R["tx_blocked"].append(p.safetyTxBlocked); R["rx_invalid"].append(p.safetyRxInvalid)
          for bi, cs_ in enumerate((p.canState0, p.canState1, p.canState2)):
            R[f"bus{bi}_err"].append(cs_.totalErrorCnt); R[f"bus{bi}_rxlost"].append(cs_.totalRxLostCnt); R[f"bus{bi}_txlost"].append(cs_.totalTxLostCnt)
            R[f"bus{bi}_busoff"].append(cs_.busOffCnt)
          C["harness_" + str(p.harnessStatus)] += 1; R["heartbeat_lost"].append(p.heartbeatLost); R["spi_err"].append(p.spiErrorCount)
      elif w == "liveCalibration":
        lc = m.liveCalibration; C["calstatus_" + str(lc.calStatus)] += 1; R["rpy"].append(list(lc.rpyCalib))
      elif w == "liveParameters":
        lp = m.liveParameters; R["sr"].append(lp.steerRatio); R["stiff"].append(lp.stiffnessFactor); R["aoff"].append(lp.angleOffsetDeg)
        R["aoff_avg"].append(lp.angleOffsetAverageDeg); C["lp_valid" if lp.valid else "lp_invalid"] += 1
      elif w == "carState":
        cs = m.carState
        if cs.steerFaultTemporary: C["steerFaultTemporary"] += 1
        if cs.steerFaultPermanent: C["steerFaultPermanent"] += 1
        if cs.espDisabled: C["espDisabled"] += 1
        if cs.vEgo > 1: R["v"].append(cs.vEgo); R["a"].append(cs.aEgo)
      elif w == "carControl":
        cc = m.carControl
        if cc.longActive: C["longActive"] += 1
        if cc.latActive: C["latActive"] += 1
        C["cc_frames"] += 1
      elif w == "longitudinalPlan":
        if m.longitudinalPlan.fcw: C["fcw"] += 1
      elif w == "managerState":
        for p in m.managerState.processes:
          if p.shouldBeRunning:
            if running.get(p.name, True) and not p.running:
              PROC[p.name] += 1
            running[p.name] = p.running
      elif w == "errorLogMessage":
        try:
          import json
          j = json.loads(m.errorLogMessage)
          msg = str(j.get("msg", ""))[:70]
        except Exception:  # noqa: BLE001
          msg = str(m.errorLogMessage)[:70]
        ERR[msg] += 1
     except Exception as e:  # noqa: BLE001
      C["msg_error_" + type(e).__name__ + ":" + str(e)[:40]] += 1
  except Exception as e:  # noqa: BLE001
    C["read_error"] += 1
pickle.dump(dict(R={k: np.array(v, dtype=object if k == "rpy" else float) for k, v in R.items()}, C=C, EV=EV, AL=AL, ERR=ERR, PROC=PROC, segs=len(fs)),
            open(f"{OUT}/sys_{r}.pkl", "wb"))
print(r, "done", len(fs), flush=True)
