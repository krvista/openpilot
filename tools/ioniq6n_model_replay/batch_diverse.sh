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
MAX_SECONDS="${MAX_SECONDS:-28000}"    # ~7h47m hard cap (override: MAX_SECONDS=NNN)
# Which routes: empty = all local routes; or a space-separated list of route ids
# (from list_routes.py) to compare specific trips, e.g. ROUTES="00000003--021caa3877 ..."
ROUTES="${ROUTES:-}"
# Segments per route: a number (evenly spread start/mid/end) OR "all" for every segment.
# "all" is more thorough but within-route segments are correlated, so for a fixed time
# budget spreading a few across many routes usually ranks models more reliably.
SAMPLES_PER_ROUTE="${SAMPLES_PER_ROUTE:-3}"

mkdir -p "$OUT"; LOG="$OUT/batch.log"
echo "=== diverse batch start $(date) ===" | tee -a "$LOG"

# Auto-detect fully-cached model indices (never request an uncached one -> no stall).
MODELS=$(python3 "$HARNESS/replay_models.py" --check 2>/dev/null \
         | sed -n 's/^fully cached indices: //p' | tr -d ' ')
if [ -z "$MODELS" ]; then echo "ERROR: no cached models found" | tee -a "$LOG"; exit 1; fi
echo "cached models: $MODELS" | tee -a "$LOG"

# Route list: explicit ROUTES if given, else every local route.
if [ -n "$ROUTES" ]; then
  read -r -a routes <<< "$ROUTES"
else
  mapfile -t routes < <(find "$ROOT" -maxdepth 1 -type d -name '*--*--*' -printf '%f\n' \
                        | sed -E 's/--[0-9]+$//' | sort -u)
fi

# For each route, collect segments: all of them, or SAMPLES_PER_ROUTE evenly-spaced.
segs=()
for route in "${routes[@]}"; do
  mapfile -t rsegs < <(find "$ROOT" -maxdepth 1 -type d -name "${route}--*" ! -name "${route}--0" \
                       -printf '%f\n' | sort -V)
  n=${#rsegs[@]}; [ "$n" -eq 0 ] && continue
  if [ "$SAMPLES_PER_ROUTE" = "all" ]; then
    for s in "${rsegs[@]}"; do segs+=("$ROOT/$s"); done
  else
    picks=""
    for k in $(seq 1 2 $(( 2 * SAMPLES_PER_ROUTE ))); do   # evenly spread positions
      i=$(( k * n / (2 * SAMPLES_PER_ROUTE) )); [ "$i" -ge "$n" ] && i=$((n-1))
      picks="$picks $i"
    done
    for i in $(echo "$picks" | tr ' ' '\n' | sort -un); do segs+=("$ROOT/${rsegs[$i]}"); done
  fi
done
total=${#segs[@]}
echo "routes: ${#routes[@]}   segments to run: $total (SAMPLES_PER_ROUTE=$SAMPLES_PER_ROUTE)" | tee -a "$LOG"
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
