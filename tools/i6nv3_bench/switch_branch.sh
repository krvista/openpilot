#!/usr/bin/env bash
# i6nv3 bench: switch the device between i6nv2 (road build) and i6nv3 (rebase).
# Run ON THE DEVICE over SSH:  bash tools/i6nv3_bench/switch_branch.sh i6nv3
# Requirements: ignition OFF, device offroad, WiFi connected.
set -euo pipefail
TARGET="${1:-}"
[[ "$TARGET" == "i6nv2" || "$TARGET" == "i6nv3" ]] || { echo "usage: $0 i6nv2|i6nv3"; exit 2; }
cd /data/openpilot
CUR=$(git branch --show-current)
echo "▶ current: $CUR  ->  target: $TARGET"
if [[ -n "$(git status --porcelain --untracked-files=no)" ]]; then
  echo "✖ working tree has tracked modifications — refusing (git status):"; git status --short --untracked-files=no | head; exit 1
fi
if [[ -f /data/params/d/IsOnroad ]] && [[ "$(cat /data/params/d/IsOnroad)" == "1" ]]; then
  echo "✖ device is onroad — switch only with ignition off"; exit 1
fi
echo "▶ fetch origin/$TARGET"
git fetch origin "$TARGET"
# panda firmware build outputs are ignored files; drop them so the target
# branch's expectation (i6nv2: tracked .bin.signed / i6nv3: fresh build) is unambiguous
echo "▶ clearing panda/board/obj build outputs"
rm -rf panda/board/obj/*.bin.signed panda/board/obj/panda_h7 panda/board/obj/body_h7 2>/dev/null || true
git checkout -f "$TARGET" 2>&1 | tail -1
git reset -q --hard "origin/$TARGET"
echo "▶ submodules"
git submodule sync -q
git submodule update --init --recursive 2>&1 | tail -3 || true
echo "▶ state"
echo "   branch:   $(git branch --show-current) @ $(git rev-parse --short HEAD)"
echo "   prebuilt: $([[ -f prebuilt ]] && echo 'yes (no scons on boot)' || echo 'NO -> full scons build on next boot (long)')"
if [[ -f panda/board/obj/panda_h7.bin.signed ]]; then
  echo "   panda fw: tracked binary present -> pandad flashes it if signature differs"
else
  echo "   panda fw: none yet -> built during boot, then flashed by pandad"
fi
echo "   submods:  $(git submodule status --recursive | awk '{print substr($1,1,1)}' | sort | uniq -c | tr '\n' ' ')  (' ' = ok, '-' = uninitialized)"
echo
echo "✔ switched. Reboot to apply:  sudo reboot"
echo "   after boot, run:  python3 tools/i6nv3_bench/panda_check.py"
