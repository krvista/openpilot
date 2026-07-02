#!/bin/bash
# Overnight batch: replay a set of models over ALL local drive segments, then aggregate.
# Resumable — already-produced outputs are skipped, so re-launching continues where it
# stopped. Run detached (nohup/tmux) so it survives an SSH disconnect.
#
#   nohup bash tools/ioniq6n_model_replay/batch_replay.sh > /data/model_replay_batch/nohup.out 2>&1 &
#   tail -f /data/model_replay_batch/batch.log
#
# Widen/narrow by editing MODELS. Adding indices and re-launching only runs the new ones.
set -u

ROOT=/data/media/0/realdata
OUT=/data/model_replay_batch
HARNESS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Replayable (2026 split-vision) decision-relevant set: OP Model 10 lineage (67/66/65/64),
# OP Model 7 (63), Off-Policy V4/V5 (58/59), + WMI V12 (53) as the known-worst reference.
MODELS="67,63,58,66,65,64,59,53"
END=300

mkdir -p "$OUT"
LOG="$OUT/batch.log"
echo "=== batch start $(date) | models=$MODELS end=$END ===" | tee -a "$LOG"

# All segments that have a full-res road camera, excluding boot segment (--0).
mapfile -t segs < <(find "$ROOT" -maxdepth 2 -name fcamera.hevc -printf '%h\n' | grep -vE -- '--0$' | sort)
total=${#segs[@]}
echo "segments to process: $total" | tee -a "$LOG"

i=0
for seg in "${segs[@]}"; do
  i=$((i+1))
  name="$(basename "$seg")"
  echo "[$i/$total] $name  $(date +%H:%M:%S)" | tee -a "$LOG"
  python3 "$HARNESS/replay_models.py" \
      --seg-dir "$seg" --indices "$MODELS" --end "$END" \
      --out-dir "$OUT/$name" >> "$LOG" 2>&1 \
    || echo "  ERROR on segment $name (continuing)" | tee -a "$LOG"
done

echo "=== replay done $(date), aggregating ===" | tee -a "$LOG"
python3 "$HARNESS/aggregate.py" "$OUT" 2>&1 | tee "$OUT/summary.txt" | tee -a "$LOG"
echo "=== ALL DONE $(date)  ->  $OUT/summary.txt ===" | tee -a "$LOG"
