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
DEV=""; OUT="$HOME/wk2-nnlc"; ROUTES=""; EXPECT=""; DELETE=0; MIN_SCORE=60; MAX_GRIP=40; SKIP_SYNC=0
CAR="${CAR:-JEEP_GRAND_CHEROKEE_2019}"; REMOTE_PATH="/data/media/0/realdata"
while [[ $# -gt 0 ]]; do case "$1" in
  -d) DEV="$2"; shift 2;; -o) OUT="$2"; shift 2;; -r) ROUTES="$2"; shift 2;;
  --expect-commit) EXPECT="$2"; shift 2;; --delete-rlogs) DELETE=1; shift;;
  --min-score) MIN_SCORE="$2"; shift 2;; --max-grip) MAX_GRIP="$2"; shift 2;;
  --skip-sync) SKIP_SYNC=1; shift;;
  *) echo "unknown arg $1"; exit 1;; esac; done
[[ -n "$DEV" || $SKIP_SYNC -eq 1 ]] || { echo "need -d user@device (or --skip-sync)"; exit 1; }
: "${NNLC_CEREAL_DIR:?set NNLC_CEREAL_DIR=<sunnypilot checkout>/cereal}"
HERE="$(cd "$(dirname "$0")" && pwd)"; mkdir -p "$OUT/rlogs" "$OUT/train"

# Python: prefer an activated venv; otherwise the nnlc-tools .venv (NNLC_TOOLS_DIR or cwd); fail early if deps are missing.
PY="${PYTHON:-python3}"
if ! "$PY" -c "import nnlc_tools, numpy, pandas" 2>/dev/null; then
  for cand in "${NNLC_TOOLS_DIR:-}/.venv/bin/python" "./.venv/bin/python"; do
    [[ -x "$cand" ]] && "$cand" -c "import nnlc_tools, numpy, pandas" 2>/dev/null && { PY="$cand"; break; }
  done
fi
"$PY" -c "import nnlc_tools, numpy, pandas" 2>/dev/null || { echo "nnlc_tools/numpy not importable: run 'source <nnlc-tools>/.venv/bin/activate' or set NNLC_TOOLS_DIR"; exit 1; }
echo "python: $PY"

# 1) rsync only rlog.zst (segment dirs), optionally restricted to given routes
if [[ $SKIP_SYNC -eq 0 ]]; then
  INC=(); if [[ -n "$ROUTES" ]]; then for r in $ROUTES; do INC+=(--include="${r}--*/" --include="${r}--*/rlog.zst"); done
          INC+=(--exclude="*"); else INC=(--include="*/" --include="rlog.zst" --exclude="*"); fi
  rsync -avz --progress --partial "${INC[@]}" "$DEV:$REMOTE_PATH/" "$OUT/rlogs/"
fi
echo "segments on disk: $(find "$OUT/rlogs" -name rlog.zst | wc -l)"

# 2) extract -> audit (build/NNLC state/grip) -> score -> prune -> coverage -> interventions -> grip prune
"$PY" -m nnlc_tools.extract_lateral_data "$OUT/rlogs" -o "$OUT/lat.csv" --temporal
"$PY" "$HERE/route_audit.py" "$OUT/lat.csv" --rlogs "$OUT/rlogs" -o "$OUT/train/keep_routes.txt" ${EXPECT:+--expect-commit "$EXPECT"} | tee "$OUT/train/audit.txt"
"$PY" -m nnlc_tools.score_routes "$OUT/lat.csv" | tee "$OUT/train/score.txt"
"$PY" -m nnlc_tools.prune_routes "$OUT/lat.csv" --min-score "$MIN_SCORE" -o "$OUT/lat_routes.csv"
"$PY" -m nnlc_tools.visualize_coverage "$OUT/lat_routes.csv" -o "$OUT/train/coverage.png"
"$PY" -m nnlc_tools.analyze_interventions "$OUT/lat_routes.csv" --prune both --prune-output "$OUT/lat_pruned.csv"
"$PY" "$HERE/prune_grip.py" "$OUT/lat_pruned.csv" -o "$OUT/train/$CAR.csv" --max-torque "$MAX_GRIP" --keep-routes "$OUT/train/keep_routes.txt"

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
