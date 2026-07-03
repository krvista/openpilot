#!/usr/bin/env python3
"""On-device model-replay harness for the Ioniq 6 N steering-model comparison.

Runs a set of candidate driving models over the SAME recorded camera frames of a
local drive segment and dumps each model's modelV2 stream. Because every model
sees identical pixels, the resulting comparison (compare_models.py) is a clean
A/B with no car-build / road / weather confound.

WHY this exists: the drivelog uploaded to the cloud is qlog+rlog only (no pixels),
and the sandbox has no GPU, so catalog models cannot be re-inferred off-device.
This must run on the comma device (QCOM GPU), which has both the recorded frames
(/data/media/0/realdata/<dongle>|<route>--<seg>/) and the model runtime.

Model switching reuses sunnypilot's own mechanism: setting the param
`ModelManager_DownloadIndex=<idx>` makes modelmanagerd download the bundle and
publish it as `ModelManager_ActiveBundle` (exactly what the UI does). modeld then
loads that custom model on next start; replay_process starts a fresh modeld each
run, so it always picks up the just-activated bundle.

USAGE (on device, openpilot venv):
    # stop onroad first so modelmanagerd/params aren't fighting us:
    #   tmux kill-session -t comma  (or however you stop the stack)
    cd /data/openpilot
    python3 tools/ioniq6n_model_replay/replay_models.py \
        --seg-dir "/data/media/0/realdata/99b215d21bbf8735|00000003--021caa3877--12" \
        --indices 67,63,58,53 \
        --start 0 --end 200 \
        --out-dir /data/model_replay_out

    # then pull /data/model_replay_out/*.zst off the device and run:
    #   python3 tools/ioniq6n_model_replay/compare_models.py /data/model_replay_out

Pick a segment with representative driving (steady city/hwy, lane lines visible),
not the boot segment. --end caps the frame count (200 frames = ~10 s @ 20 Hz);
keep it modest — each model re-runs the whole range.
"""
import argparse
import copy
import json
import os
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

# NOTE: modeld pins itself to RT core 7 via config_realtime_process(); under replay
# from an SSH shell whose cpuset cgroup doesn't own that core, os.sched_setaffinity
# raises EINVAL and crashes the modeld child. The fix lives in common/realtime.py
# (set_core_affinity now swallows OSError) because modeld runs in a separate process
# and a monkeypatch here would not reach it.

from openpilot.common.params import Params
from openpilot.tools.lib.logreader import LogReader, save_log
from openpilot.tools.lib.framereader import FrameReader
from openpilot.selfdrive.test.process_replay.process_replay import get_process_config, replay_process
from openpilot.sunnypilot.models.helpers import get_active_bundle


def trim_logs(logs, start_frame, end_frame, frs_types, include_all_types):
  """Same trim/setup logic as selfdrive/test/process_replay/model_replay.py, vendored
  here because that module imports matplotlib at top level, which the device's
  python env doesn't ship."""
  all_msgs = []
  cam_state_counts = defaultdict(int)
  for msg in sorted(logs, key=lambda m: m.logMonoTime):
    if msg.which() in frs_types:
      cam_state_counts[msg.which()] += 1
    if any(cam_state_counts[state] >= start_frame for state in frs_types):
      all_msgs.append(msg)
    if all(cam_state_counts[state] == end_frame for state in frs_types):
      break

  if len(include_all_types) != 0:
    other_msgs = [m for m in logs if m.which() in include_all_types]
    all_msgs.extend(other_msgs)

  return all_msgs


def _bundle_filenames(obj):
  """Recursively collect artifact/metadata filenames from a bundle dict. The server
  catalog cache (ModelManager_ModelsCache) uses snake_case 'file_name' (fetcher.py),
  while capnp to_dict bundles use camelCase 'fileName' — accept both."""
  out = []
  if isinstance(obj, dict):
    for k, v in obj.items():
      if k in ("fileName", "file_name") and isinstance(v, str):
        out.append(v)
      else:
        out.extend(_bundle_filenames(v))
  elif isinstance(obj, list):
    for v in obj:
      out.extend(_bundle_filenames(v))
  return out


def is_fully_cached(params: Params, idx: int) -> tuple[bool, list[str]]:
  """True iff every artifact/metadata file of bundle `idx` already exists in model_root.
  Prevents requesting a model that would need a download: modelmanagerd can stall on a
  failing download, wedging staging for every subsequent model until a reboot."""
  from openpilot.system.hardware.hw import Paths
  cache = params.get("ModelManager_ModelsCache")
  bundles = (cache or {}).get("bundles", []) if isinstance(cache, dict) else []
  bundle = next((b for b in bundles if b.get("index") == idx), None)
  if bundle is None:
    return False, [f"bundle idx={idx} not in ModelsCache"]
  root = Paths.model_root()
  names = _bundle_filenames(bundle)
  if not names:
    # can't verify -> do NOT vacuously pass (a stalled download wedges the daemon)
    return False, [f"bundle idx={idx}: no filenames found in cache entry"]
  missing = [f for f in names if not os.path.exists(os.path.join(root, f))]
  return len(missing) == 0, missing


def stage_model(params: Params, idx: int, timeout: float = 180.0) -> dict | None:
  """Switch the active model to bundle `idx` and block until it is staged.

  Returns the active-bundle dict on success, None on timeout / failure.
  """
  cur = get_active_bundle(params)
  cur_idx = cur.index if cur else None
  if cur_idx == idx:
    return cur.to_dict() if hasattr(cur, "to_dict") else dict(cur)

  cached, missing = is_fully_cached(params, idx)
  if not cached:
    print(f"  [stage] idx={idx} NOT fully cached (missing: {missing[:3]}{'...' if len(missing) > 3 else ''})")
    print(f"  [stage] skipping — download it first via the UI model selector, then rerun.")
    return None

  print(f"  [stage] requesting model idx={idx} (was {cur_idx}) ...")
  params.put("ModelManager_DownloadIndex", idx)
  t0 = time.monotonic()
  while time.monotonic() - t0 < timeout:
    # daemon removes DownloadIndex and publishes ActiveBundle when done
    if params.get("ModelManager_DownloadIndex") is None:
      ab = get_active_bundle(params)
      if ab is not None and ab.index == idx:
        print(f"  [stage] active idx={idx} '{ab.displayName}' after {time.monotonic()-t0:.0f}s")
        return ab.to_dict() if hasattr(ab, "to_dict") else dict(ab)
    time.sleep(2.0)
  print(f"  [stage] TIMEOUT staging idx={idx} — is modelmanagerd running? skipping.")
  return None


def build_frame_readers(seg_dir: Path, end_frame: int) -> dict:
  road = seg_dir / "fcamera.hevc"
  wide = seg_dir / "ecamera.hevc"
  if not road.exists():
    raise FileNotFoundError(f"missing road camera: {road}")
  frs = {"roadCameraState": FrameReader(str(road), pix_fmt="nv12", cache_size=end_frame)}
  if wide.exists():
    frs["wideRoadCameraState"] = FrameReader(str(wide), pix_fmt="nv12", cache_size=end_frame)
  else:
    print(f"  [warn] no wide camera {wide}; model expecting big_img may degrade")
  return frs


def build_modeld_config():
  """process_replay only ships a config for the STOCK python modeld
  (selfdrive.modeld.modeld), which loads fixed model files and ignores the
  sunnypilot ModelManager bundle entirely — so every bundle replays identically
  with degenerate output. The real driving model on this device is the sunnypilot
  process `modeld_tinygrad` (a bash->modeld.py wrapper in sunnypilot/modeld_v2).
  It has the same camera/calibration/carState inputs and publishes modelV2, so we
  reuse the stock config's pubs/callbacks and just retarget proc_name to the
  sunnypilot process (ProcessContainer resolves it via managed_processes)."""
  cfg = copy.deepcopy(get_process_config("modeld"))
  cfg.proc_name = "modeld_tinygrad"
  return cfg


def replay_one(seg_dir: Path, start_frame: int, end_frame: int, frs: dict, custom_params: dict):
  rlog = seg_dir / "rlog.zst"
  if not rlog.exists():
    rlog = seg_dir / "rlog"
  lr = list(LogReader(str(rlog)))

  cam_states = {"roadCameraState", "wideRoadCameraState"}
  # modeld's inputs are camera + encodeIdx + carParams + carState/carControl +
  # calibration/deviceState/liveDelay/driverMonitoringState. It does NOT subscribe to
  # `can` (500+ Hz), which would otherwise dominate the replay queue for zero benefit.
  extra = {"roadEncodeIdx", "wideRoadEncodeIdx", "carParams", "carState", "carControl",
           "liveCalibration", "deviceState", "liveDelay", "driverMonitoringState"}
  logs = trim_logs(lr, start_frame, end_frame, cam_states, extra)

  # seed calibration + deviceState at t0 so modeld initializes deterministically
  for s in ("liveCalibration", "deviceState"):
    try:
      msg = next(m for m in lr if m.which() == s).as_builder()
      msg.logMonoTime = lr[0].logMonoTime
      logs.insert(1, msg.as_reader())
    except StopIteration:
      print(f"  [warn] no {s} in log; modeld may use defaults")

  # process_replay isolates params under an OpenpilotPrefix, so the ModelManager bundle
  # we activated on the real params is invisible to the replayed modeld. Inject it via
  # custom_params (model FILES live at the unprefixed /data/media/0/models, so they are
  # still found). Without this modeld would fall back to the default bundle.
  msgs = replay_process(build_modeld_config(), logs, frs, custom_params=custom_params)
  return [m for m in msgs if m.which() in ("modelV2", "drivingModelData")]


def run_models(args, indices):
  """Worker: stage + replay + save for each index IN THIS PROCESS. The dispatcher calls
  this with a single index per subprocess so all memory (frame cache + GPU) is released
  between models."""
  seg_dir = Path(args.seg_dir)
  out_dir = Path(args.out_dir)
  out_dir.mkdir(parents=True, exist_ok=True)

  params = Params()
  original = get_active_bundle(params)
  original_idx = original.index if original else None
  print(f"original active model idx={original_idx}")

  frs = build_frame_readers(seg_dir, args.end)

  manifest = []
  for idx in indices:
    existing = list(out_dir.glob(f"{idx:03d}_*.zst"))
    if existing:
      print(f"  [skip] idx={idx} already done: {existing[0].name}")
      manifest.append((idx, existing[0].stem, "skipped"))
      continue
    ab = stage_model(params, idx)
    if ab is None:
      manifest.append((idx, None, "stage_failed"))
      continue
    name = str(ab.get("internalName") or ab.get("displayName") or idx).replace("/", "_").replace(" ", "_")
    # snapshot the just-activated bundle from the REAL params to inject into the
    # replay's isolated params (see replay_one). ModelManager_ActiveBundle is a JSON-type
    # param, so params.get returns a dict and params.put expects a dict (NOT a JSON
    # string) — pass it through verbatim. modeld derives the runner from the bundle, so
    # ModelRunnerTypeCache does not need injecting.
    custom_params = {}
    ab_param = params.get("ModelManager_ActiveBundle")
    if ab_param is not None:
      custom_params["ModelManager_ActiveBundle"] = ab_param
    try:
      msgs = replay_one(seg_dir, args.start, args.end, frs, custom_params)
    except Exception as e:
      import traceback
      print(f"  [replay] idx={idx} FAILED: {e}")
      # replay_process masks the real error in its cleanup (stop() on a half-started
      # container); print the full chain so the ORIGINAL exception is visible.
      traceback.print_exc()
      manifest.append((idx, name, f"replay_failed:{type(e).__name__}"))
      continue
    mv2 = [m for m in msgs if m.which() == "modelV2"]
    n = len(mv2)
    if n == 0:
      print(f"  [replay] idx={idx} produced 0 modelV2 msgs (modeld likely crashed — see log above). NOT saving.")
      manifest.append((idx, name, "empty:modeld_crash?"))
      continue
    # sanity: check the PLAN (position.y), not action.desiredCurvature — the latter is
    # engagement-gated and always 0 under replay. A live model has a varying plan.
    pys = [m.modelV2.position.y[10] for m in mv2 if len(m.modelV2.position.y) > 10]
    py_std = float(np.std(pys)) if len(pys) > 1 else 0.0
    flag = "  <-- FLAT plan, model may not have run!" if py_std < 1e-4 else ""
    print(f"  [check] idx={idx} plan position.y std={py_std:.3e}{flag}")
    out = out_dir / f"{idx:03d}_{name}.zst"
    save_log(str(out), msgs)
    print(f"  [saved] {out.name}: {n} modelV2 msgs")
    manifest.append((idx, name, f"ok:{n}"))

  # restore original model
  if original_idx is not None:
    print(f"restoring original model idx={original_idx}")
    stage_model(params, original_idx)

  print("\n=== manifest ===")
  for idx, name, status in manifest:
    print(f"  idx={idx} {name} -> {status}")
  return manifest


def list_cache_status():
  """Print which catalog bundles are fully cached (safe to replay without download)."""
  params = Params()
  cache = params.get("ModelManager_ModelsCache")
  bundles = (cache or {}).get("bundles", []) if isinstance(cache, dict) else []
  ready = []
  for b in bundles:
    idx = b.get("index")
    ok, missing = is_fully_cached(params, idx)
    if ok:
      ready.append(idx)
      print(f"  CACHED  idx={idx:>3}  {b.get('display_name') or b.get('short_name')}")
  print(f"\nfully cached indices: {','.join(str(i) for i in sorted(ready))}")
  print("(models not listed need a UI download first)")


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--check", action="store_true", help="list fully-cached model indices and exit")
  ap.add_argument("--seg-dir", help="local segment dir with rlog.zst + fcamera.hevc + ecamera.hevc")
  ap.add_argument("--indices", help="comma-separated bundle indices, e.g. 67,63,58,53")
  ap.add_argument("--start", type=int, default=0)
  ap.add_argument("--end", type=int, default=200)
  ap.add_argument("--out-dir", default="/data/model_replay_out")
  args = ap.parse_args()

  if args.check:
    list_cache_status()
    return
  if not args.seg_dir or not args.indices:
    ap.error("--seg-dir and --indices are required (or use --check)")

  indices = [int(x) for x in args.indices.split(",") if x.strip()]
  out_dir = Path(args.out_dir)
  out_dir.mkdir(parents=True, exist_ok=True)

  if len(indices) > 1:
    # Isolate each model in its own process. A modeld replay holds ~2 GB (decoded frame
    # cache + GPU host buffers); forking a second modeld from a process that large OOMs
    # (os.fork -> Errno 12). A fresh interpreter per model releases everything between
    # runs, so `--indices 67,63,58,53` works in one command.
    import subprocess
    import sys
    for idx in indices:
      if list(out_dir.glob(f"{idx:03d}_*.zst")):
        print(f"\n===== model {idx}: already done, skipping =====", flush=True)
        continue
      print(f"\n===== model {idx} (isolated subprocess) =====", flush=True)
      subprocess.run([sys.executable, os.path.abspath(__file__),
                      "--seg-dir", args.seg_dir, "--indices", str(idx),
                      "--start", str(args.start), "--end", str(args.end),
                      "--out-dir", args.out_dir])
    (out_dir / "run_meta.txt").write_text(
      f"seg_dir={args.seg_dir}\nstart={args.start}\nend={args.end}\nindices={indices}\n")
    print(f"\nall models done -> run: python3 {os.path.dirname(os.path.abspath(__file__))}/compare_models.py {args.out_dir}")
    return

  run_models(args, indices)
  (out_dir / "run_meta.txt").write_text(
    f"seg_dir={args.seg_dir}\nstart={args.start}\nend={args.end}\nindices={indices}\n")


if __name__ == "__main__":
  main()
