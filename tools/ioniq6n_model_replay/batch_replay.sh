#!/bin/bash
# Batch replay of a model set over segments of the N most-recent routes, then aggregate.
# Resumable (done outputs skipped) and time-capped (stops before MAX_SECONDS, then
# aggregates whatever finished). Run detached so it survives an SSH disconnect:
#
#   nohup bash tools/ioniq6n_model_replay/batch_replay.sh > /data/model_replay_batch/nohup.out 2>&1 &
#   tail -f /data/model_replay_batch/batch.log
#
# Segments are processed model-inner / segment-outer and INTERLEAVED across routes, so a
# capped run yields fully-covered segments spread evenly over all N routes. Re-launch to
# resume the rest. Tune MODELS / RECENT_ROUTES / END / MAX_SECONDS below.
set -u

ROOT=/data/media/0/realdata
OUT=/data/model_replay_batch
HARNESS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Replayable (2026 split-vision) models to compare. Broad off-policy lineage since a
# single route is cheap: OP Model 10 lineage (67/66/65/64), OP MODEL (62), OPM7 (63),
# Off-Policy V5/V4/V3 (59/58/57) + WMI V12 (53, known-worst reference).
MODELS="67,66,65,64,63,62,59,58,57,53"
RECENT_ROUTES=1       # only the most-recent route (all its segments)
END=200               # frames (~10 s @ 20 Hz); enough for the 2-8 Hz metric
MAX_SECONDS=14000     # hard cap ~3h53m, leaving buffer under 4 h for aggregation

mkdir -p "$OUT"
LOG="$OUT/batch.log"
echo "=== batch start $(date) | models=$MODELS routes=$RECENT_ROUTES end=$END cap=${MAX_SECONDS}s ===" | tee -a "$LOG"

# Select the N most-recent routes (by segment mtime), all their segments except boot --0,
# interleaved across routes so a time-capped run spreads evenly.
mapfile -t segs < <(python3 - "$ROOT" "$RECENT_ROUTES" <<'PY'
import sys, os, glob, re
root, n = sys.argv[1], int(sys.argv[2])
segdirs = [os.path.dirname(p) for p in glob.glob(os.path.join(root, "*", "fcamera.hevc"))]
def route(d): return re.sub(r'--\d+$', '', os.path.basename(d))
rt = {}
for d in segdirs:
    rt.setdefault(route(d), 0.0)
    rt[route(d)] = max(rt[route(d)], os.path.getmtime(d))
recent = set(sorted(rt, key=lambda r: rt[r], reverse=True)[:n])
by_route = {}
for d in sorted(segdirs):
    r = route(d)
    if r in recent and not d.endswith('--0'):
        by_route.setdefault(r, []).append(d)
# round-robin interleave across routes
order = sorted(by_route, key=lambda r: rt[r], reverse=True)
i = 0
while any(by_route[r] for r in order):
    for r in order:
        if i < len(by_route[r]):
            print(by_route[r][i])
    i += 1
PY
)
total=${#segs[@]}
echo "segments selected: $total (from $RECENT_ROUTES most-recent routes)" | tee -a "$LOG"

start=$(date +%s)
i=0
for seg in "${segs[@]}"; do
  now=$(date +%s); el=$((now - start))
  if [ "$el" -gt "$MAX_SECONDS" ]; then
    echo "=== time cap ${MAX_SECONDS}s reached at segment $i/$total — stopping, will aggregate ===" | tee -a "$LOG"
    break
  fi
  i=$((i+1))
  name="$(basename "$seg")"
  echo "[$i/$total  ${el}s elapsed] $name  $(date +%H:%M:%S)" | tee -a "$LOG"
  python3 "$HARNESS/replay_models.py" \
      --seg-dir "$seg" --indices "$MODELS" --end "$END" \
      --out-dir "$OUT/$name" >> "$LOG" 2>&1 \
    || echo "  ERROR on segment $name (continuing)" | tee -a "$LOG"
done

echo "=== replay phase done $(date), aggregating ===" | tee -a "$LOG"
python3 "$HARNESS/aggregate.py" "$OUT" 2>&1 | tee "$OUT/summary.txt" | tee -a "$LOG"
echo "=== ALL DONE $(date)  ->  $OUT/summary.txt ===" | tee -a "$LOG"
