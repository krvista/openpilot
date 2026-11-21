#!/bin/bash
# E155 (report §95): chain sim (E64) on the b62f36a0 drives (87, 89, 91, 93). Reference = action re-timed to 0.375 s
# (E64_REF=a0), so E133 / E154 keep their 375 ms yardstick on drives whose action was computed for 0.65 / 0.76 s.
# Tags: drv = as driven (told delay from the log) | val = told 0.30 (the configuration the Phase 60 / 69a sims validated)
#       lrn = told 0.759 (the learned value the next drives start with) | hi = lrn + 0.1 s more advance from 80 km/h
# usage (repo root): e155_run_sim_b62.sh <scratch> <plant: rf|rfw> <tag> [tag ...]   rows -> <scratch>/qa/e64_rows_63<plant>_n_<tag>.pkl
S=$1; PL=$2; shift 2
export PYTHONPATH=$PWD:$PWD/opendbc_repo
declare -A TOLD=( [drv]="" [val]=0.30 [lrn]=0.759 [hi]=0.759 )
declare -A E=( [drv]=0.5,0.5,0.5,0.3,0.3 [val]=0.5,0.5,0.5,0.3,0.3 [lrn]=0.5,0.5,0.5,0.3,0.3 [hi]=0.5,0.5,0.5,0.56,0.56 )
for tag in "$@"; do
  if [ -n "${TOLD[$tag]}" ]; then export REPLAY_FORCE_TOLD=${TOLD[$tag]}; else unset REPLAY_FORCE_TOLD; fi
  env E64_PLANT=$PL E64_COMMITS=b62f36a07 E64_CCP="MDPS_COMP_LAMBDA=0" E64_ETAB=${E[$tag]} REPLAY_MODEL_DELAY=log E64_REF=a0 \
    E64_VARS=Ce+0.0 E64_TAG=_63${PL}_n_$tag python3 tools/i6nv3_bench/probes/qa/e64_chain_sim.py $S 4 2>&1 | grep -v Warn | tail -1 | sed "s/^Ce+0.0 /$PL $tag /"
done
