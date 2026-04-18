#!/usr/bin/env python3
"""Phase 4 validation — run all 5 steps sequentially + regression sims.

Usage:  python3 tools/ioniq6n_phase4_validate_all.py

Returns 0 on full pass, non-zero on any failure.
"""
import subprocess
import sys
import time

STEPS = [
  ("Step 1 — VM init smoke test",            "tools/ioniq6n_phase4_validate_01_vm_init.py"),
  ("Step 2 — normal op + MADS+ACC",          "tools/ioniq6n_phase4_validate_02_normal.py"),
  ("Step 3 — MADS/ACC/passthrough branches", "tools/ioniq6n_phase4_validate_03_branches.py"),
  ("Step 4 — driver override + NaN/inf + cam_stale", "tools/ioniq6n_phase4_validate_04_adversarial.py"),
  ("Step 5 — engage/jitter/hysteresis",      "tools/ioniq6n_phase4_validate_05_engage_jitter.py"),
  ("Step 6a — regression: edge-case sim",    "tools/ioniq6n_edge_case_sim.py"),
  ("Step 6b — regression: parking-mode sim", "tools/ioniq6n_parking_mode_toggle_sim.py"),
]


def run_one(title, script):
  t0 = time.time()
  result = subprocess.run(["python3", script], capture_output=True, text=True,
                          cwd="/home/user/openpilot")
  dt = time.time() - t0
  ok = result.returncode == 0
  tail = result.stdout.strip().splitlines()[-3:] if result.stdout else []
  status = "✅" if ok else "❌"
  print(f"{status} {title:55s}  ({dt:.1f}s)")
  for line in tail:
    print(f"    {line}")
  if not ok:
    print("---- stderr ----")
    print(result.stderr)
  return ok


if __name__ == "__main__":
  print("=" * 78)
  print("  Phase 4 validation — Ioniq 6N steering pipeline")
  print("=" * 78)
  results = [(title, run_one(title, script)) for title, script in STEPS]
  passed = sum(1 for _, ok in results if ok)
  total = len(results)
  print("=" * 78)
  if passed == total:
    print(f"  ✅ ALL {total} VALIDATION STEPS PASSED")
  else:
    print(f"  ❌ {total - passed} / {total} FAILED")
    for title, ok in results:
      if not ok:
        print(f"     - {title}")
  print("=" * 78)
  sys.exit(0 if passed == total else 1)
