#!/bin/bash
# Diverse-sample batch: for every local route, pick a few evenly-spaced segments
# (start / middle / end), replay them through all FULLY-CACHED models, then aggregate.
# Only cached models are used (auto-detected), so it never stalls on a download.
# Resumable + time-capped. Run detached:
#
#   nohup bash tools/ioniq6n_model_replay/batch_diverse.sh > /data/model_replay_diverse/nohup.out 2>&1 &
#   tail -f /data/model_replay_diverse/batch.log
set -u

ROOT=/data/media/0/realdata
OUT=/data/model_replay_diverse
HARNESS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
END=200
MAX_SECONDS=14000        # ~3h53m hard cap
SAMPLES_PER_ROUTE=3      # segments per route (spread across start/mid/end)

mkdir -p "$OUT"; LOG="$OUT/batch.log"
echo "=== diverse batch start $(date) ===" | tee -a "$LOG"

# Auto-detect fully-cached model indices (never request an uncached one -> no stall).
MODELS=$(python3 "$HARNESS/replay_models.py" --check 2>/dev/null \
         | sed -n 's/^fully cached indices: //p' | tr -d ' ')
if [ -z "$MODELS" ]; then echo "ERROR: no cached models found" | tee -a "$LOG"; exit 1; fi
echo "cached models: $MODELS" | tee -a "$LOG"

# For each route, pick SAMPLES_PER_ROUTE evenly-spaced segments (skip boot --0).
segs=()
mapfile -t routes < <(find "$ROOT" -maxdepth 1 -type d -name '*--*--*' -printf '%f\n' \
                      | sed -E 's/--[0-9]+$//' | sort -u)
for route in "${routes[@]}"; do
  mapfile -t rsegs < <(find "$ROOT" -maxdepth 1 -type d -name "${route}--*" ! -name "${route}--0" \
                       -printf '%f\n' | sort -V)
  n=${#rsegs[@]}; [ "$n" -eq 0 ] && continue
  picks=""
  for num in 1 3 5; do                       # ~17% / 50% / 83% positions
    i=$(( num * n / 6 )); [ "$i" -ge "$n" ] && i=$((n-1))
    picks="$picks $i"
  done
  # unique pick indices, then collect
  for i in $(echo "$picks" | tr ' ' '\n' | sort -un); do
    segs+=("$ROOT/${rsegs[$i]}")
  done
done
total=${#segs[@]}
echo "routes: ${#routes[@]}   sampled segments: $total" | tee -a "$LOG"
printf '  %s\n' "${segs[@]}" | sed "s#$ROOT/##" | tee -a "$LOG"

start=$(date +%s); i=0
for seg in "${segs[@]}"; do
  el=$(( $(date +%s) - start ))
  if [ "$el" -gt "$MAX_SECONDS" ]; then
    echo "=== time cap reached at $i/$total — aggregating ===" | tee -a "$LOG"; break
  fi
  i=$((i+1)); name="$(basename "$seg")"
  echo "[$i/$total  ${el}s] $name  $(date +%H:%M:%S)" | tee -a "$LOG"
  python3 "$HARNESS/replay_models.py" --seg-dir "$seg" --indices "$MODELS" \
      --end "$END" --out-dir "$OUT/$name" >> "$LOG" 2>&1 \
    || echo "  ERROR on $name (continuing)" | tee -a "$LOG"
done

echo "=== replay done $(date), aggregating ===" | tee -a "$LOG"
python3 "$HARNESS/aggregate.py" "$OUT" 2>&1 | tee "$OUT/summary.txt" | tee -a "$LOG"
echo "=== ALL DONE $(date)  ->  $OUT/summary.txt ===" | tee -a "$LOG"
