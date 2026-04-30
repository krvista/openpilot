#!/usr/bin/env python3
"""
Analyze rlog/qlog files in `drivelog/` for process crashes, errors, and
unexpected disengagements. Produces:

  debug_logs/findings.json   machine-readable per-event records
  debug_logs/REPORT.md       human-readable summary

Single-pass over rlog (~0.1 s/segment); falls back to qlog when rlog is
missing for a segment.

Usage:
  python tools/scripts/analyze_drivelog.py
  python tools/scripts/analyze_drivelog.py --routes 0fb02cc3a5abcc2f_0000000b
  python tools/scripts/analyze_drivelog.py --drivelog ./drivelog --out ./debug_logs
"""
import argparse
import json
import os
import re
import sys
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field
from pathlib import Path

import capnp
import zstandard as zstd

REPO_ROOT = Path(__file__).resolve().parents[2]
SCHEMA = REPO_ROOT / "cereal" / "log.capnp"

capnp.remove_import_hook()
log = capnp.load(str(SCHEMA))

# Decode EventName enum (id -> name)
EVENT_NAME = {v: k for k, v in log.OnroadEvent.EventName.schema.enumerants.items()}

# These events fire when controlsd intentionally disengages or refuses to engage.
# See cereal/log.capnp OnroadEvent and selfdrive/selfdrived/events.py.
DISENGAGE_FLAGS = ("immediateDisable", "softDisable", "userDisable", "preEnable")

# Common "noisy but mostly benign" daemon errors we still want to count
BENIGN_PATTERNS = [
  re.compile(r"iso-tp query response pending"),
  re.compile(r"got vin with request"),
]

# F1.3 — extra ACC-stop pathways. selfdrived can disengage on these even if they
# are not in our existing DISENGAGE_FLAGS bucket (some are observed indirectly
# via SOFT_DISABLE / IMMEDIATE_DISABLE flags), but we want a per-event count to
# rule out causes other than the audio chain we already fixed.
EXTRA_DISENGAGE_EVENTS = {
  "accFaulted", "canError", "canBusMissing", "cruiseDisabled", "espActive",
  "commIssue", "commIssueAvgFreq", "radarFault", "cameraMalfunction",
  "cameraFrameRate", "modeldLagging",
}

# F1.2 — modeld burst threshold. modeld emits "skipping model eval. Dropped X
# frames" warnings; small numbers (1-3) are filtered/normal. Anything ≥10 is a
# significant freeze (10 frames @ 20 Hz = 0.5 s; 117 frames = ~5.85 s) and we
# want surrounding deviceState/procLog context to classify the cause.
MODELD_BURST_THRESHOLD = 10
MODELD_DROP_RE = re.compile(r"Dropped (\d+) frames")


@dataclass
class Finding:
  route: str
  segment: int
  log_mono_time_ns: int
  kind: str               # process_crash | error_log | disengage | panda_fault | thermal | mem_pressure | model_drop
  daemon: str = ""
  detail: str = ""
  severity: str = "info"  # info | warning | error | critical
  extra: dict = field(default_factory=dict)


def decompress(path: Path) -> bytes:
  with open(path, "rb") as f:
    raw = f.read()
  return zstd.ZstdDecompressor().stream_reader(raw).read()


def parse_log_message(text: str) -> dict | None:
  try:
    return json.loads(text)
  except (json.JSONDecodeError, TypeError):
    return None


def discover(drivelog_dir: Path):
  """Group files by route -> {seg_idx: {qlog, rlog}}."""
  routes: dict[str, dict[int, dict[str, Path]]] = defaultdict(lambda: defaultdict(dict))
  for fp in sorted(drivelog_dir.iterdir()):
    if not fp.is_file():
      continue
    m = re.match(r"^([0-9a-f]+_[0-9a-f]+)--([0-9a-f]+)--(\d+)--(qlog|rlog)\.zst$", fp.name)
    if not m:
      continue
    route_short = m.group(1)
    seg = int(m.group(3))
    kind = m.group(4)
    routes[route_short][seg][kind] = fp
  return routes


def scan_segment(route: str, seg: int, path: Path, source: str,
                 route_state: dict | None = None) -> tuple[list[Finding], dict, dict]:
  """Walk one log file and emit findings + summary stats.

  source: 'rlog' or 'qlog'
  route_state: dict carried across segments of the same route. Holds the
    last-known carParams snapshot (since carParams typically appears only
    once at recording start, in seg 0). Returned (possibly mutated) for the
    next segment to consume.
  """
  findings: list[Finding] = []
  stats = {
    "events_total": 0,
    "msg_types": Counter(),
    "fingerprint": None,
    "branch": None,
    "version": None,
    "device": None,
    "duration_s": 0.0,
    "errorLogMessage_count": 0,
    "logMessage_error_count": 0,
    "logMessage_warning_count": 0,
    "extra_disengage_events": Counter(),
  }
  if route_state is None:
    route_state = {}

  data = decompress(path)
  events = list(log.Event.read_multiple_bytes(data))
  if not events:
    return findings, stats, route_state

  stats["events_total"] = len(events)
  t_start = events[0].logMonoTime
  t_end = events[-1].logMonoTime
  stats["duration_s"] = max(0.0, (t_end - t_start) / 1e9)

  # State trackers
  prev_running: dict[str, bool] = {}
  prev_panda_fault: dict[int, str] = {}
  prev_self_enabled = None
  fired_event_seen: set[str] = set()
  last_thermal = None
  last_mem_pct = None
  # F1.1 — last seen pandaState snapshot (per panda index) and recent timeseries.
  # Captured at the moment a controlsMismatch event fires so we can compare each
  # field against the carParams snapshot in route_state and pinpoint which one
  # diverged.
  last_panda_states: list[dict] = []
  # F1.2 — last seen deviceState. Attached to a finding when modeld reports a
  # large dropped-frame burst, so we can tell thermal-throttle from mem
  # pressure from CPU starvation without having to re-walk the rlog.
  last_device_state: dict = {}
  # F1.1 — controlsMismatch dedup. The event can fire many times per frame
  # window; we only keep the first one per segment to avoid drowning the
  # report.
  controls_mismatch_logged = False

  for evt in events:
    typ = evt.which()
    stats["msg_types"][typ] += 1
    t = evt.logMonoTime

    if typ == "carParams":
      try:
        stats["fingerprint"] = evt.carParams.carFingerprint
      except Exception:
        pass
      # F1.1 — snapshot carParams.safetyConfigs and alternativeExperience for
      # later comparison against pandaStates when a controlsMismatch fires.
      # carParams is published once at start; persist it in route_state so
      # later segments (which may not contain another carParams) still have
      # the reference values.
      try:
        sc_snap = []
        for sc in evt.carParams.safetyConfigs:
          sc_snap.append({
            "safetyModel": str(sc.safetyModel),
            "safetyParam": int(sc.safetyParam),
          })
        route_state["cp_snapshot"] = {
          "safetyConfigs": sc_snap,
          "alternativeExperience": int(evt.carParams.alternativeExperience),
          "carFingerprint": str(evt.carParams.carFingerprint),
        }
      except Exception:
        pass
    elif typ == "initData":
      try:
        stats["version"] = evt.initData.version
        stats["device"] = evt.initData.deviceType.raw if hasattr(evt.initData.deviceType, "raw") else str(evt.initData.deviceType)
        stats["branch"] = evt.initData.gitBranch
      except Exception:
        pass

    elif typ == "managerState":
      for proc in evt.managerState.processes:
        name = proc.name
        running = proc.running
        prev = prev_running.get(name)
        if prev is True and not running:
          # Process stopped while previously running
          findings.append(Finding(
            route=route, segment=seg, log_mono_time_ns=t,
            kind="process_crash", daemon=name,
            detail=f"exitCode={proc.exitCode} shouldBeRunning={proc.shouldBeRunning}",
            severity="critical" if proc.shouldBeRunning else "warning",
            extra={
              "exitCode": int(proc.exitCode),
              "shouldBeRunning": bool(proc.shouldBeRunning),
              "pid": int(proc.pid),
            },
          ))
        prev_running[name] = running

    elif typ == "errorLogMessage":
      stats["errorLogMessage_count"] += 1
      msg_text = evt.errorLogMessage
      parsed = parse_log_message(msg_text)
      daemon = ""
      msg = msg_text[:300]
      level = ""
      if parsed:
        ctx = parsed.get("ctx", {})
        daemon = ctx.get("daemon", "") if isinstance(ctx, dict) else ""
        # `msg` field may itself be a string or a nested dict (like fingerprint event)
        m = parsed.get("msg")
        if isinstance(m, dict):
          msg = json.dumps(m)[:300]
        else:
          msg = str(m)[:300]
        level = parsed.get("level", parsed.get("levelname", ""))

      if any(p.search(msg) for p in BENIGN_PATTERNS):
        sev = "info"
      elif level == "ERROR":
        sev = "error"
      else:
        sev = "warning"

      findings.append(Finding(
        route=route, segment=seg, log_mono_time_ns=t,
        kind="error_log", daemon=daemon,
        detail=msg, severity=sev,
        extra={"level": level},
      ))

      # F1.2 — modeld burst context probe. modeld emits "skipping model eval.
      # Dropped X frames" when vipc has gaps. For X >= MODELD_BURST_THRESHOLD
      # we emit a separate finding annotated with the latest deviceState so we
      # can see at a glance whether the freeze coincided with thermal=red or
      # high CPU/mem (which would point at camerad starvation rather than
      # modeld itself).
      bm = MODELD_DROP_RE.search(msg)
      if bm and "modeld" in (daemon or ""):
        n_drop = int(bm.group(1))
        if n_drop >= MODELD_BURST_THRESHOLD:
          ds = last_device_state
          # Classify the burst based on attached telemetry.
          tags: list[str] = []
          if ds.get("thermalStatus") == "red":
            tags.append("thermal=red")
          if (ds.get("memoryUsagePercent") or 0) > 85:
            tags.append(f"mem={ds['memoryUsagePercent']}%")
          cpu = ds.get("cpuUsagePercent") or []
          if cpu and max(cpu) >= 90:
            tags.append(f"cpu_max={max(cpu)}%")
          tag_str = ", ".join(tags) if tags else "no-thermal/cpu/mem-flag"
          findings.append(Finding(
            route=route, segment=seg, log_mono_time_ns=t,
            kind="modeld_burst_context", daemon=daemon or "modeld",
            detail=f"dropped={n_drop} frames ({n_drop/20.0:.2f}s freeze) ; {tag_str}",
            severity="error" if n_drop >= 50 else "warning",
            extra={
              "dropped_frames": n_drop,
              "freeze_seconds": round(n_drop / 20.0, 2),
              "deviceState": ds,
            },
          ))

    elif typ == "logMessage":
      parsed = parse_log_message(evt.logMessage)
      if parsed:
        lvl = parsed.get("levelname") or parsed.get("level") or ""
        if lvl == "ERROR":
          stats["logMessage_error_count"] += 1
          ctx = parsed.get("ctx", {}) if isinstance(parsed.get("ctx"), dict) else {}
          findings.append(Finding(
            route=route, segment=seg, log_mono_time_ns=t,
            kind="error_log", daemon=ctx.get("daemon", ""),
            detail=str(parsed.get("msg", ""))[:300], severity="error",
            extra={"level": lvl, "filename": parsed.get("filename", "")},
          ))
        elif lvl == "WARNING":
          stats["logMessage_warning_count"] += 1

    elif typ == "onroadEvents":
      for ev in evt.onroadEvents:
        try:
          name = str(ev.name)
        except Exception:
          name = "?"

        # F1.3 — count every occurrence of an extra-disengage-pathway event,
        # regardless of whether a disengage flag is set on this particular
        # frame. selfdrived may pulse e.g. canError repeatedly before the
        # state machine actually disengages, so the per-segment count is a
        # better signal than the single flagged emission we capture below.
        if name in EXTRA_DISENGAGE_EVENTS:
          stats["extra_disengage_events"][name] += 1

        # Only report a given disengage event once per segment (de-noise)
        flagged = [f for f in DISENGAGE_FLAGS if getattr(ev, f, False)]
        if flagged and name not in fired_event_seen:
          fired_event_seen.add(name)
          sev = "critical" if "immediateDisable" in flagged else "warning"
          findings.append(Finding(
            route=route, segment=seg, log_mono_time_ns=t,
            kind="disengage", daemon="controlsd",
            detail=f"{name} ({','.join(flagged)})", severity=sev,
            extra={"event": name, "flags": flagged},
          ))

          # F1.1 — when a controlsMismatch immediateDisable lands, dump a
          # comparison between the latest pandaStates snapshot and the
          # carParams snapshot to identify which exact field
          # (safetyModel / safetyParam / alternativeExperience) diverged.
          # This is the diagnostic our previous pass was missing — we knew
          # WHICH event fired but not WHY.
          if name == "controlsMismatch" and not controls_mismatch_logged:
            controls_mismatch_logged = True
            cp_snap = route_state.get("cp_snapshot")
            mismatch_fields: list[str] = []
            rx_invalid = []
            if cp_snap and last_panda_states:
              for i, ps in enumerate(last_panda_states):
                if i < len(cp_snap["safetyConfigs"]):
                  cp_sc = cp_snap["safetyConfigs"][i]
                  if ps["safetyModel"] != cp_sc["safetyModel"]:
                    mismatch_fields.append(
                      f"panda{i}.safetyModel: panda={ps['safetyModel']} cp={cp_sc['safetyModel']}"
                    )
                  if ps["safetyParam"] != cp_sc["safetyParam"]:
                    mismatch_fields.append(
                      f"panda{i}.safetyParam: panda={ps['safetyParam']} cp={cp_sc['safetyParam']}"
                    )
                if ps["alternativeExperience"] != cp_snap["alternativeExperience"]:
                  mismatch_fields.append(
                    f"panda{i}.alternativeExperience: panda={ps['alternativeExperience']} cp={cp_snap['alternativeExperience']}"
                  )
                if ps.get("safetyRxChecksInvalid"):
                  rx_invalid.append(f"panda{i}")
            detail_bits = []
            if mismatch_fields:
              detail_bits.append("FIELD_MISMATCH=" + " | ".join(mismatch_fields))
            if rx_invalid:
              detail_bits.append("safetyRxChecksInvalid=" + ",".join(rx_invalid))
            if not detail_bits:
              detail_bits.append("no field mismatch found at trigger time (likely mismatch_counter>=200)")
            findings.append(Finding(
              route=route, segment=seg, log_mono_time_ns=t,
              kind="controlsMismatch_context", daemon="selfdrived",
              detail=" ; ".join(detail_bits),
              severity="critical",
              extra={
                "cp": cp_snap,
                "panda": last_panda_states,
              },
            ))

    elif typ == "selfdriveState":
      try:
        enabled = evt.selfdriveState.enabled
      except Exception:
        enabled = None
      if prev_self_enabled is True and enabled is False:
        findings.append(Finding(
          route=route, segment=seg, log_mono_time_ns=t,
          kind="disengage", daemon="selfdrived",
          detail="selfdriveState.enabled True->False",
          severity="warning",
        ))
      prev_self_enabled = enabled

    elif typ == "pandaStates":
      # F1.1 — refresh the last-seen snapshot per panda. Used by the
      # controlsMismatch probe in the onroadEvents branch above.
      new_snapshot: list[dict] = []
      for i, ps in enumerate(evt.pandaStates):
        try:
          fault = str(ps.faultStatus)
        except Exception:
          fault = "?"
        new_snapshot.append({
          "safetyModel": str(ps.safetyModel),
          "safetyParam": int(ps.safetyParam),
          "alternativeExperience": int(ps.alternativeExperience),
          "safetyRxChecksInvalid": bool(getattr(ps, "safetyRxChecksInvalid", False)),
          "controlsAllowed": bool(getattr(ps, "controlsAllowed", False)),
          "faultStatus": fault,
        })
        prev = prev_panda_fault.get(i)
        if fault != "none" and prev != fault:
          findings.append(Finding(
            route=route, segment=seg, log_mono_time_ns=t,
            kind="panda_fault", daemon=f"panda{i}",
            detail=f"faultStatus={fault} faults={list(ps.faults) if ps.faults else []}",
            severity="error",
          ))
        if getattr(ps, "heartbeatLost", False) and prev != "heartbeatLost":
          findings.append(Finding(
            route=route, segment=seg, log_mono_time_ns=t,
            kind="panda_fault", daemon=f"panda{i}",
            detail="heartbeatLost",
            severity="error",
          ))
        prev_panda_fault[i] = fault
      last_panda_states = new_snapshot

    elif typ == "deviceState":
      try:
        thermal = str(evt.deviceState.thermalStatus)
      except Exception:
        thermal = None
      try:
        mem_pct = int(evt.deviceState.memoryUsagePercent)
      except Exception:
        mem_pct = None
      # F1.2 — capture cpu / gpu / mem / freeSpace / freeMem so we can attach
      # them to a modeld burst finding (cause classifier: thermal vs mem vs
      # cpu starvation).
      try:
        cpu_pct = list(evt.deviceState.cpuUsagePercent)
      except Exception:
        cpu_pct = []
      try:
        gpu_pct = int(evt.deviceState.gpuUsagePercent)
      except Exception:
        gpu_pct = None
      try:
        free_mb = int(evt.deviceState.memoryUsagePercent)  # placeholder; capnp doesn't expose freeMb directly
      except Exception:
        free_mb = None
      last_device_state = {
        "thermalStatus": thermal,
        "memoryUsagePercent": mem_pct,
        "cpuUsagePercent": cpu_pct,
        "gpuUsagePercent": gpu_pct,
        "logMonoTime": t,
      }
      if thermal == "red" and last_thermal != "red":
        findings.append(Finding(
          route=route, segment=seg, log_mono_time_ns=t,
          kind="thermal", daemon="thermald",
          detail=f"thermalStatus={thermal}", severity="error",
        ))
      last_thermal = thermal
      if mem_pct is not None and mem_pct > 90 and (last_mem_pct or 0) <= 90:
        findings.append(Finding(
          route=route, segment=seg, log_mono_time_ns=t,
          kind="mem_pressure", daemon="deviceState",
          detail=f"memoryUsagePercent={mem_pct}", severity="warning",
        ))
      last_mem_pct = mem_pct

  # Convert msg_types Counter for JSON
  stats["msg_types"] = dict(stats["msg_types"])
  stats["extra_disengage_events"] = dict(stats["extra_disengage_events"])
  return findings, stats, route_state


def per_route_summary(route: str, segments: list[int], all_findings: list[Finding], stats_by_seg: dict[int, dict]) -> dict:
  by_kind = Counter(f.kind for f in all_findings)
  by_daemon = Counter(f.daemon for f in all_findings if f.daemon)
  by_severity = Counter(f.severity for f in all_findings)
  durations = [stats_by_seg[s]["duration_s"] for s in segments if s in stats_by_seg]
  total_dur_min = sum(durations) / 60.0
  fp = next((stats_by_seg[s]["fingerprint"] for s in segments if stats_by_seg.get(s, {}).get("fingerprint")), None)
  ver = next((stats_by_seg[s]["version"] for s in segments if stats_by_seg.get(s, {}).get("version")), None)
  br = next((stats_by_seg[s]["branch"] for s in segments if stats_by_seg.get(s, {}).get("branch")), None)
  return {
    "route": route,
    "segments": len(segments),
    "duration_min": round(total_dur_min, 1),
    "fingerprint": fp,
    "version": ver,
    "branch": br,
    "by_kind": dict(by_kind),
    "by_severity": dict(by_severity),
    "top_daemons": dict(by_daemon.most_common(10)),
  }


def write_report(findings: list[Finding], route_summaries: list[dict], stats_by_route: dict, out_dir: Path):
  out_dir.mkdir(parents=True, exist_ok=True)
  # findings.json
  (out_dir / "findings.json").write_text(json.dumps({
    "route_summaries": route_summaries,
    "findings": [asdict(f) for f in findings],
  }, indent=2))

  # REPORT.md
  lines: list[str] = []
  lines.append("# sunny-release-tizi drivelog 분석 결과\n")
  lines.append(f"총 라우트 {len(route_summaries)}개, 사건 {len(findings)}건\n")
  lines.append("\n## 라우트별 요약\n")
  lines.append("| route | 세그 | 시간(분) | fingerprint | 사건 종류 | 심각도 |")
  lines.append("|---|---|---|---|---|---|")
  for rs in route_summaries:
    kinds = ", ".join(f"{k}={v}" for k, v in rs["by_kind"].items())
    sevs = ", ".join(f"{k}={v}" for k, v in rs["by_severity"].items())
    lines.append(f"| `{rs['route']}` | {rs['segments']} | {rs['duration_min']} | {rs['fingerprint'] or '?'} | {kinds} | {sevs} |")

  # Cross-route patterns
  all_daemons = Counter(f.daemon for f in findings if f.daemon)
  all_kinds = Counter(f.kind for f in findings)
  lines.append("\n## 전체 패턴 (5개 라우트 합산)\n")
  lines.append("### 사건 종류별\n")
  for k, v in all_kinds.most_common():
    lines.append(f"- **{k}**: {v}건")
  lines.append("\n### 데몬별 (Top 15)\n")
  for d, v in all_daemons.most_common(15):
    lines.append(f"- `{d}`: {v}건")

  # Critical findings
  critical = [f for f in findings if f.severity == "critical"]
  if critical:
    lines.append(f"\n## Critical 사건 ({len(critical)}건)\n")
    by_route = defaultdict(list)
    for f in critical:
      by_route[f.route].append(f)
    for route, evts in sorted(by_route.items()):
      lines.append(f"\n### {route}\n")
      for f in evts[:30]:
        ts_min = (f.log_mono_time_ns - stats_by_route.get(route, {}).get("t0", 0)) / 1e9 / 60.0
        lines.append(f"- seg {f.segment} (~{ts_min:.1f} min) **{f.kind}** `{f.daemon}` — {f.detail}")
      if len(evts) > 30:
        lines.append(f"- ... +{len(evts)-30} more")

  # Top error messages (de-duplicated)
  error_counter: Counter = Counter()
  for f in findings:
    if f.kind == "error_log":
      key = (f.daemon, f.detail[:120])
      error_counter[key] += 1
  if error_counter:
    lines.append(f"\n## 반복되는 에러 메시지 Top 30\n")
    lines.append("| 횟수 | daemon | message |")
    lines.append("|---|---|---|")
    for (d, msg), n in error_counter.most_common(30):
      esc = msg.replace("|", "\\|").replace("\n", " ")
      lines.append(f"| {n} | `{d}` | {esc} |")

  # Disengage events (cross-route)
  disengages = [f for f in findings if f.kind == "disengage" and "controlsd" in f.daemon]
  if disengages:
    de_counter = Counter(f.extra.get("event", f.detail) for f in disengages if isinstance(f.extra, dict))
    lines.append(f"\n## Disengage 이벤트 분포\n")
    for ev, n in de_counter.most_common():
      lines.append(f"- `{ev}`: {n}건")

  # Process crash details
  crashes = [f for f in findings if f.kind == "process_crash"]
  if crashes:
    lines.append(f"\n## 프로세스 종료/크래시 ({len(crashes)}건)\n")
    cc = Counter(f.daemon for f in crashes)
    for d, n in cc.most_common(20):
      ec_samples = [f.extra.get("exitCode") for f in crashes if f.daemon == d]
      lines.append(f"- `{d}`: {n}건  (exitCode 분포: {Counter(ec_samples).most_common(5)})")

  # F1.1 — controlsMismatch root-cause probe. One row per controlsMismatch
  # event with the diff between panda's actual safety state and CP's expected
  # safety state at trigger time.
  cm_ctx = [f for f in findings if f.kind == "controlsMismatch_context"]
  if cm_ctx:
    lines.append(f"\n## controlsMismatch 컨텍스트 (F1.1, {len(cm_ctx)}건)\n")
    for f in cm_ctx:
      lines.append(f"\n### {f.route} seg {f.segment}\n")
      lines.append(f"- 트리거 시점 mismatch: {f.detail}")
      cp = f.extra.get("cp") or {}
      panda = f.extra.get("panda") or []
      if cp:
        sc = cp.get("safetyConfigs") or []
        sc_str = ", ".join(f"{s.get('safetyModel')}/{s.get('safetyParam')}" for s in sc)
        lines.append(f"- CP: alternativeExperience={cp.get('alternativeExperience')}, "
                     f"safetyConfigs=[{sc_str}], fingerprint={cp.get('carFingerprint')}")
      for i, ps in enumerate(panda):
        lines.append(f"- panda{i}: safetyModel={ps.get('safetyModel')} "
                     f"safetyParam={ps.get('safetyParam')} "
                     f"alternativeExperience={ps.get('alternativeExperience')} "
                     f"safetyRxChecksInvalid={ps.get('safetyRxChecksInvalid')} "
                     f"controlsAllowed={ps.get('controlsAllowed')}")

  # F1.2 — modeld burst contexts. One row per ≥10-frame drop with the latest
  # deviceState attached so the cause classifier (thermal / mem / cpu) is
  # immediately readable.
  burst_ctx = [f for f in findings if f.kind == "modeld_burst_context"]
  if burst_ctx:
    lines.append(f"\n## modeld 프레임 burst 컨텍스트 (F1.2, {len(burst_ctx)}건)\n")
    lines.append("| route | seg | 드랍 | 추정 freeze | tag |")
    lines.append("|---|---|---|---|---|")
    for f in burst_ctx:
      ex = f.extra
      lines.append(f"| `{f.route}` | {f.segment} | {ex.get('dropped_frames')} | "
                   f"{ex.get('freeze_seconds')} s | {f.detail.split(' ; ')[-1]} |")
    burst_tag_counter = Counter(f.detail.split(" ; ")[-1] for f in burst_ctx)
    lines.append("\n버스트 분류 합계:")
    for tag, n in burst_tag_counter.most_common():
      lines.append(f"- {tag}: {n}건")

  # F1.3 — extra disengage event totals (across all segments and routes).
  # These events can chain into SOFT_DISABLE / IMMEDIATE_DISABLE in
  # selfdrived; counting them rules out (or surfaces) ACC drop causes
  # beyond the audio chain.
  extra_total: Counter = Counter()
  for route, info in stats_by_route.items():
    for seg_stats in info.get("segments", {}).values():
      for evname, n in (seg_stats.get("extra_disengage_events") or {}).items():
        extra_total[evname] += n
  if extra_total:
    lines.append(f"\n## 추가 disengage 경로 이벤트 카운트 (F1.3)\n")
    lines.append("| 이벤트 | 횟수 |")
    lines.append("|---|---|")
    for ev, n in extra_total.most_common():
      lines.append(f"| `{ev}` | {n} |")
  else:
    lines.append("\n## 추가 disengage 경로 이벤트 카운트 (F1.3)\n")
    lines.append("(0건 — Phase E 의 audio 픽스 외 추가 ACC-drop 경로는 본 데이터셋에 없음)")

  (out_dir / "REPORT.md").write_text("\n".join(lines) + "\n")


def parse_args():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument("--drivelog", default=str(REPO_ROOT / "drivelog"), help="drivelog directory")
  p.add_argument("--out", default=str(REPO_ROOT / "debug_logs"), help="output directory")
  p.add_argument("--routes", nargs="*", help="restrict to these route prefixes (substring match)")
  p.add_argument("--limit-segs", type=int, default=None, help="cap segments per route (debug)")
  return p.parse_args()


def main():
  args = parse_args()
  drivelog = Path(args.drivelog)
  out_dir = Path(args.out)
  if not drivelog.exists():
    sys.exit(f"drivelog dir not found: {drivelog}")

  routes = discover(drivelog)
  if args.routes:
    routes = {r: v for r, v in routes.items() if any(rt in r for rt in args.routes)}

  if not routes:
    sys.exit("no matching routes")

  print(f"Found {len(routes)} route(s):")
  for r, segs in routes.items():
    print(f"  {r}  ({len(segs)} segments)")

  all_findings: list[Finding] = []
  route_summaries: list[dict] = []
  stats_by_route: dict[str, dict] = {}

  for route, segmap in routes.items():
    print(f"\n=== {route} ===")
    seg_indices = sorted(segmap.keys())
    if args.limit_segs:
      seg_indices = seg_indices[: args.limit_segs]
    stats_by_seg: dict[int, dict] = {}
    route_findings: list[Finding] = []
    t0 = None
    # Carries the carParams snapshot (and any other route-scoped state) from
    # one segment to the next so that probes which need carParams (e.g. F1.1
    # controlsMismatch comparison) still work in segments where carParams is
    # absent — which is most of them, since carParams is published once at
    # recording start.
    route_state: dict = {}

    for seg in seg_indices:
      files = segmap[seg]
      path = files.get("rlog") or files.get("qlog")
      source = "rlog" if files.get("rlog") else "qlog"
      if not path:
        continue
      try:
        f_list, stats, route_state = scan_segment(route, seg, path, source, route_state)
      except Exception as e:
        print(f"  seg {seg:3d}: FAILED ({source}): {e}")
        continue
      stats_by_seg[seg] = stats
      route_findings.extend(f_list)
      if t0 is None and f_list:
        t0 = min(f.log_mono_time_ns for f in f_list)
      print(f"  seg {seg:3d} [{source}]: {stats['events_total']} evts, "
            f"errLog={stats['errorLogMessage_count']} "
            f"err={stats['logMessage_error_count']} warn={stats['logMessage_warning_count']} "
            f"-> {len(f_list)} findings")

    # Use earliest time as zero for relative timestamps
    if t0 is None and stats_by_seg:
      t0 = min(s["events_total"] and 0 for s in stats_by_seg.values())
    summary = per_route_summary(route, seg_indices, route_findings, stats_by_seg)
    route_summaries.append(summary)
    stats_by_route[route] = {"t0": t0 or 0, "segments": stats_by_seg}
    all_findings.extend(route_findings)

  print(f"\nTotal findings: {len(all_findings)}")
  write_report(all_findings, route_summaries, stats_by_route, out_dir)
  print(f"\nWrote {out_dir}/REPORT.md and {out_dir}/findings.json")


if __name__ == "__main__":
  main()
