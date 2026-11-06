#!/bin/bash
# Phase 63 sim matrix: plant (m1 | rf) x variant; all on the same windows (op hands-off >= 30 km/h, 4 builds)
S=${1:?scratch dir}; PL=${2:?plant}; shift 2
cd /home/user/wt3
export PYTHONPATH=$PWD:$PWD/opendbc_repo
OFF63A="CURVE_TRIM_GATE_V=4,4;CURVE_TRIM_HOLD_V=3,3;CURVE_TRIM_DEADBAND_V=0.7,0.7"
declare -A V=( [base]="MDPS_COMP_LAMBDA=0;ENTRY_BOOST_GAIN=0;$OFF63A" [a]="MDPS_COMP_LAMBDA=0;ENTRY_BOOST_GAIN=0"
               [b4]="MDPS_COMP_LAMBDA=0;$OFF63A" [ab4]="MDPS_COMP_LAMBDA=0" [ab6]="MDPS_COMP_LAMBDA=0;ENTRY_BOOST_GAIN=0.6"
               [ab3]="MDPS_COMP_LAMBDA=0;ENTRY_BOOST_GAIN=0.3"
               [fin]="MDPS_COMP_LAMBDA=0;CURVE_TRIM_CAP_SPEEDS_MS=8.3,19.4,25,30;CURVE_TRIM_CAP_DEG=6,4.86,2.5,1.5"
               [ab4c]="MDPS_COMP_LAMBDA=0;CURVE_TRIM_CAP_SPEEDS_MS=8.3,19.4,27.8;CURVE_TRIM_CAP_DEG=6,3,1.5"
               [ab4d]="MDPS_COMP_LAMBDA=0;CURVE_TRIM_CAP_SPEEDS_MS=8.3,19.4,27.8;CURVE_TRIM_CAP_DEG=6,2,1"
               [ab4e]="MDPS_COMP_LAMBDA=0;CURVE_TRIM_CAP_SPEEDS_MS=8.3,19.4,27.8;CURVE_TRIM_CAP_DEG=6,2,1;CURVE_TRIM_RATE_DPS=1.0" )
for tag in "$@"; do
  SECONDS=0; env E64_PLANT=$PL E64_COMMITS=a0108a47b,98e912d3d,8de98262a,bf36aa2d6 E64_CCP="${V[$tag]}" E64_ETAB=0.3,0.3,0.3,0.3,0.3 \
    E64_VARS=Ce+0.0 E64_TAG=_63${PL}_$tag python3 $S/qa/e64_chain_sim.py $S 4 2>&1 | grep -v Warn | tail -2 | sed "s/^Ce+0.0 /$PL $tag /"; echo "  ($SECONDS s)"
done
echo ALL_DONE
