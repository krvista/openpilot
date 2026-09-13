#!/usr/bin/env bash
# PC-side (WSL) NNLC data pipeline: pull rlogs from the device over LAN into a
# scratch dir, build the training CSV locally, keep only train/ (tens of MB),
# and optionally delete the rlogs. Nothing large ever goes to GitHub.
#
#   bash pc_pipeline.sh -d comma@192.168.1.155 -o ~/wk2-nnlc [-r "0000002b--ec8d875f19 0000002a--9e72ae868f"]
#                       [--expect-commit d67c5fe] [--delete-rlogs] [--min-score 60] [--max-grip 40]
# Requires: patched nnlc-tools installed (uv pip install -e .), NNLC_CEREAL_DIR set,
#           and this script's siblings (route_audit.py, prune_grip.py) next to it.
set -euo pipefail
DEV=""; OUT="$HOME/wk2-nnlc"; ROUTES=""; EXPECT=""; DELETE=0; MIN_SCORE=60; MAX_GRIP=40
CAR="${CAR:-JEEP_GRAND_CHEROKEE_2019}"; REMOTE_PATH="/data/media/0/realdata"
while [[ $# -gt 0 ]]; do case "$1" in
  -d) DEV="$2"; shift 2;; -o) OUT="$2"; shift 2;; -r) ROUTES="$2"; shift 2;;
  --expect-commit) EXPECT="$2"; shift 2;; --delete-rlogs) DELETE=1; shift;;
  --min-score) MIN_SCORE="$2"; shift 2;; --max-grip) MAX_GRIP="$2"; shift 2;;
  *) echo "unknown arg $1"; exit 1;; esac; done
[[ -n "$DEV" ]] || { echo "need -d user@device"; exit 1; }
: "${NNLC_CEREAL_DIR:?set NNLC_CEREAL_DIR=<sunnypilot checkout>/cereal}"
HERE="$(cd "$(dirname "$0")" && pwd)"; mkdir -p "$OUT/rlogs" "$OUT/train"

# 1) rsync only rlog.zst (segment dirs), optionally restricted to given routes
INC=(); if [[ -n "$ROUTES" ]]; then for r in $ROUTES; do INC+=(--include="${r}--*/" --include="${r}--*/rlog.zst"); done
        INC+=(--exclude="*"); else INC=(--include="*/" --include="rlog.zst" --exclude="*"); fi
rsync -avz --progress --partial "${INC[@]}" "$DEV:$REMOTE_PATH/" "$OUT/rlogs/"
echo "segments on disk: $(find "$OUT/rlogs" -name rlog.zst | wc -l)"

# 2) extract -> audit (build/NNLC state/grip) -> score -> prune -> coverage -> interventions -> grip prune
python3 -m nnlc_tools.extract_lateral_data "$OUT/rlogs" -o "$OUT/lat.csv" --temporal
python3 "$HERE/route_audit.py" "$OUT/lat.csv" --rlogs "$OUT/rlogs" -o "$OUT/train/keep_routes.txt" ${EXPECT:+--expect-commit "$EXPECT"} | tee "$OUT/train/audit.txt"
python3 -m nnlc_tools.score_routes "$OUT/lat.csv" | tee "$OUT/train/score.txt"
python3 -m nnlc_tools.prune_routes "$OUT/lat.csv" --min-score "$MIN_SCORE" -o "$OUT/lat_routes.csv"
python3 -m nnlc_tools.visualize_coverage "$OUT/lat_routes.csv" -o "$OUT/train/coverage.png"
python3 -m nnlc_tools.analyze_interventions "$OUT/lat_routes.csv" --prune both --prune-output "$OUT/lat_pruned.csv"
python3 "$HERE/prune_grip.py" "$OUT/lat_pruned.csv" -o "$OUT/train/$CAR.csv" --max-torque "$MAX_GRIP" --keep-routes "$OUT/train/keep_routes.txt"

# 3) train/ is the only thing that leaves this machine: CSV for Julia (local), gz copy for the branch
gzip -9 -k -f "$OUT/train/$CAR.csv"
rm -f "$OUT"/lat*.csv
[[ $DELETE -eq 1 ]] && { rm -rf "$OUT/rlogs"; echo "rlogs deleted"; } || echo "rlogs kept in $OUT/rlogs (delete after the model is validated)"
ls -la "$OUT/train"
cat <<MSG

NEXT
  train locally : bash training/run.sh "$OUT/train/"        (add --cpu if no NVIDIA GPU)
  share results : push $OUT/train/{$CAR.csv.gz,coverage.png,score.txt,audit.txt} to branch wk2-nnlc-train (no rlogs)
MSG
