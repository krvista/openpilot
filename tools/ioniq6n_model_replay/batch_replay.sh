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

# Pick the most-recent route (by newest segment mtime), then take ALL its segments except
# boot --0. Pure find/bash — no python — so it can't stall on glob. RECENT_ROUTES is fixed
# at 1 here for simplicity; widen by looping this block if needed.
echo "selecting most-recent route ..." | tee -a "$LOG"
newest=$(find "$ROOT" -maxdepth 2 -name fcamera.hevc -printf '%T@ %h\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)
if [ -z "$newest" ]; then
  echo "ERROR: no segments with fcamera.hevc under $ROOT" | tee -a "$LOG"; exit 1
fi
route=$(basename "$newest" | sed -E 's/--[0-9]+$//')
echo "most-recent route: $route" | tee -a "$LOG"
mapfile -t segs < <(find "$ROOT" -maxdepth 1 -type d -name "${route}--*" ! -name "${route}--0" 2>/dev/null | sort -V)
total=${#segs[@]}
echo "segments selected: $total (route $route)" | tee -a "$LOG"
if [ "$total" -eq 0 ]; then
  echo "ERROR: 0 segments selected for route $route" | tee -a "$LOG"; exit 1
fi

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
