#!/usr/bin/env bash
# Build the NNLC training set from rlogs stored on a drivelog git branch, without
# keeping rlogs anywhere permanently. Run where the openpilot fork (for cereal) and
# the patched nnlc-tools are available (e.g. the Claude sandbox). Output: train/
# containing JEEP_GRAND_CHEROKEE_2019.csv.gz + coverage.png + score.txt, ready to be
# pushed to the lightweight orphan branch `wk2-nnlc-train`.
#
#   bash make_trainset.sh <drivelog-ref> <dongle_id> <out_dir> [route-id ...]
#   e.g. bash make_trainset.sh origin/wk2-drivelog 99b215d21bbf8735 ./out 0000000c--aaaa 0000000d--bbbb
set -euo pipefail
REF="$1"; DONGLE="$2"; OUT="$3"; shift 3
OPENPILOT_DIR="${OPENPILOT_DIR:-/home/user/openpilot}"
export NNLC_CEREAL_DIR="${NNLC_CEREAL_DIR:-$OPENPILOT_DIR/cereal}"
CAR="${CAR:-JEEP_GRAND_CHEROKEE_2019}"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$OUT/rlogs" "$OUT/train"

# 1) pull only the requested routes' rlogs, blob by blob (partial clone friendly)
git -C "$OPENPILOT_DIR" ls-tree -r --name-only "$REF" | grep "${DONGLE}_" | grep -- "--rlog.zst$" | while read -r path; do
  name="$(basename "$path")"
  for r in "$@"; do
    if [[ "$name" == "${DONGLE}_${r}--"*"--rlog.zst" ]]; then
      seg="${name#${DONGLE}_${r}--}"; seg="${seg%--rlog.zst}"
      d="$OUT/rlogs/${r}--${seg}"; mkdir -p "$d"
      [[ -s "$d/rlog.zst" ]] || git -C "$OPENPILOT_DIR" show "$REF:$path" > "$d/rlog.zst"
    fi
  done
done
echo "segments staged: $(find "$OUT/rlogs" -name rlog.zst | wc -l)"

# 2) extract -> score -> route prune -> coverage -> intervention prune -> grip prune
python3 -m nnlc_tools.extract_lateral_data "$OUT/rlogs" -o "$OUT/lat.csv" --temporal
python3 -m nnlc_tools.score_routes "$OUT/lat.csv" | tee "$OUT/train/score.txt"
python3 -m nnlc_tools.prune_routes "$OUT/lat.csv" --min-score "${MIN_SCORE:-60}" -o "$OUT/lat_routes.csv"
python3 -m nnlc_tools.visualize_coverage "$OUT/lat_routes.csv" -o "$OUT/train/coverage.png"
python3 -m nnlc_tools.analyze_interventions "$OUT/lat_routes.csv" --prune both --prune-output "$OUT/lat_pruned.csv"
python3 "$HERE/prune_grip.py" "$OUT/lat_pruned.csv" -o "$OUT/train/$CAR.csv" --max-torque "${MAX_GRIP:-40}"

# 3) pack for the PC: GitHub rejects files > 100 MB, so ship gzip (Julia reads the .csv after gunzip)
gzip -9 -f "$OUT/train/$CAR.csv"
rm -rf "$OUT/rlogs" "$OUT"/lat*.csv          # no rlogs are kept
ls -la "$OUT/train"
echo "next: push $OUT/train/* to orphan branch wk2-nnlc-train; on the PC: gunzip && bash training/run.sh <dir>"
