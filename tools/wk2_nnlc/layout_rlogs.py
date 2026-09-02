#!/usr/bin/env python3
"""Re-layout flat drivelog uploads into the per-segment directory layout that
nnlc-tools expects (<route>--<seg>/rlog.zst), via symlinks (no copy).

  python3 layout_rlogs.py <flat_dir_with_<dongle>_<route>--<seg>--rlog.zst> <out_dir> [--dongle ID]
"""
import argparse, glob, os, re, sys

ap = argparse.ArgumentParser()
ap.add_argument("flat_dir")
ap.add_argument("out_dir")
ap.add_argument("--dongle", default=None, help="only this dongle id (default: all)")
a = ap.parse_args()

pat = re.compile(r"(?:(?P<dongle>[0-9a-f]{16})_)?(?P<route>[0-9a-f]+--[0-9a-f]+)--(?P<seg>\d+)--rlog\.zst$")
n = 0
for f in sorted(glob.glob(os.path.join(a.flat_dir, "**", "*rlog.zst"), recursive=True)):
    m = pat.search(os.path.basename(f))
    if not m or (a.dongle and m.group("dongle") != a.dongle):
        continue
    d = os.path.join(a.out_dir, f"{m.group('route')}--{m.group('seg')}")
    os.makedirs(d, exist_ok=True)
    dst = os.path.join(d, "rlog.zst")
    if not os.path.exists(dst):
        os.symlink(os.path.abspath(f), dst)
    n += 1
print(f"linked {n} segments into {a.out_dir}")
