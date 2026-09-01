# Ioniq 6 N — on-device driving-model comparison

Clean A/B of driving models by replaying each one over the **same recorded camera
frames** of one local drive segment, then scoring the steering-relevant fields of
its `modelV2` output. Same pixels in → any metric difference is the **model**, not
the road / weather / car-tuning build.

## Why on-device

- The cloud drivelog is **qlog + rlog only (no camera pixels)** → models can't be
  re-inferred off-device.
- Catalog models ship as **QCOM-compiled `_tinygrad.pkl`** (no portable ONNX) and
  the dev sandbox has **no GPU** → CPU re-inference is impossible.
- The comma device has both the recorded frames and the QCOM model runtime, so the
  replay must run there.

Only **current-architecture (2026 split vision+policy)** bundles are replayable with
the current `modeld`. Older 2024–2025 single-model bundles have a different I/O and
will `replay_failed` (harness skips them, keeps going).

## Files

| file | where it runs | what it does |
|------|---------------|--------------|
| `replay_models.py`  | **device (GPU)** | switch model → replay one segment → dump `modelV2` per model |
| `compare_models.py` | anywhere (numpy+capnp) | score the dumped `modelV2` streams into a comparison table |

## 1. Run the replay (on the device)

Stop the onroad stack first so `modelmanagerd`/params aren't contended, then:

```bash
cd /data/openpilot   # your on-device checkout of this branch
python3 tools/ioniq6n_model_replay/replay_models.py \
    --seg-dir "/data/media/0/realdata/99b215d21bbf8735|00000003--021caa3877--12" \
    --indices 67,63,58,53 \
    --start 0 --end 200 \
    --out-dir /data/model_replay_out
```

- `--seg-dir` — a **local segment** dir containing `rlog.zst`, `fcamera.hevc`,
  `ecamera.hevc`. Pick a **representative driving** segment (steady city or highway,
  lane lines visible) — not the boot segment (seg 0) and not a standstill/offroad
  tail. Segment 12 of route `00000003` is a reasonable default; browse
  `/data/media/0/realdata/` for good ones.
- `--indices` — bundle indices to compare (see the catalog list below). The harness
  switches models via sunnypilot's own param path (`ModelManager_DownloadIndex`),
  waits for each to download+activate, replays, then restores your original model.
- `--start/--end` — frame window. `200` frames ≈ 10 s @ 20 Hz. Each model re-runs the
  whole window, so keep it modest; widen once you know it works.

The daemon must be able to **download** any index not already cached, so keep the
device online for the first run.

## 2. Score the results (anywhere)

Pull `/data/model_replay_out/*.zst` off the device (or run in place) and:

```bash
python3 tools/ioniq6n_model_replay/compare_models.py /data/model_replay_out
```

Output columns (all on op-relevant `modelV2` fields, first 20 frames dropped as warmup):

| column | meaning | direction |
|--------|---------|-----------|
| `jitter_dc`   | `action.desiredCurvature` 2–8 Hz band RMS (1e-4 1/m) — planner hi-freq noise, the felt low-speed wobble driver | lower = smoother |
| `jitter_path` | mid-horizon `position.y` 2–8 Hz band RMS (m) — path shimmer | lower = smoother |
| `center_p95`  | \|lane-center offset\| p95 (m), both lane lines confident | lower = better centered |
| `clr_near%`   | % frames nearest lane line < 0.9 m — lane-hug risk | lower = safer |
| `lp_conf`     | mean min(left,right) lane-line prob — model confidence | higher = better |
| `dc_range`    | p95−p5 of desiredCurvature (1e-4 1/m) — steering travel used | context, not good/bad |

Rows are sorted by `jitter_dc` (primary smoothness proxy).

## Candidate indices (from the on-device ModelsCache, 2026-07-02)

Decision-relevant, replayable (off-policy `lat=.0` family + today's two + release refs):

| idx | name | channel | notes |
|----:|------|---------|-------|
| 67 | OP Model 10 V3 | dev | ran on route 0x01 today; best in on-road A/B |
| 66 | OP Model 10 v2 | dev | |
| 65 | OP Model v10 | dev | |
| 64 | OP Model 8 | dev | |
| 63 | OP Model 7 | **master** | most-validated off-policy `lat=.0` |
| 62 | OP Model | master | |
| 59 | Off-Policy v5 | dev | |
| 58 | Off-Policy V4 | dev | |
| 57 | Off Policy v3 | dev | |
| 53 | WMI V12 | master | ran on routes 0x02/0x03 today (world-model lineage) |
| 41 | The Cool Peoples v3 | release | `lat=.1` ref — may `replay_failed` if arch differs |
| 38 | North Nevada V2 | release | `lat=.1` ref — may `replay_failed` |

Suggested first run: `--indices 67,63,58,53` (today's best vs a master off-policy vs
an older off-policy vs today's world-model). Add more once the pipeline is proven.

## Caveats

- Replay uses recorded `liveCalibration`; if calibration was mid-convergence in the
  chosen segment, absolute centering is noisier — trust the **relative** ranking.
- `modelExecutionTime` is not a fair metric under replay; ignore timing here.
- One segment ≈ one road context. Repeat on 2–3 varied segments (a curvy one, a
  highway one) before committing to a model.
