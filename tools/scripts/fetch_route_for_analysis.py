#!/usr/bin/env python3
"""
Download rlog/qlog for a route from comma connect, save them under
debug_logs/<route>/, and (optionally) commit + push so Claude can analyze.

Auth: requires `python tools/lib/auth.py` to have been run once (or pass
      `--jwt <JWT>`; get one at https://jwt.comma.ai).

Usage examples:
  # Default: qlog only, all segments, commit + push to current branch
  python tools/scripts/fetch_route_for_analysis.py 0fb02cc3a5abcc2f/0000000b--99d3e5258c

  # Add rlog (large; use with --segments to keep the diff manageable)
  python tools/scripts/fetch_route_for_analysis.py 0fb02cc3a5abcc2f/0000000b--99d3e5258c --rlog --segments 0:3

  # Only download, do not commit
  python tools/scripts/fetch_route_for_analysis.py 0fb02cc3a5abcc2f/0000000b--99d3e5258c --no-commit

  # Tag the commit with a note (e.g. crash time within route)
  python tools/scripts/fetch_route_for_analysis.py <route> --note "cruise dropped at ~04:32"
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

import requests

try:
  from openpilot.tools.lib.auth_config import get_token
except ModuleNotFoundError:
  # Fallback: read the same auth.json the openpilot tool writes.
  import os
  def get_token():
    path = os.path.expanduser(os.environ.get("COMMA_AUTH", "~/.comma/auth.json"))
    try:
      with open(path) as f:
        return json.load(f).get("access_token")
    except FileNotFoundError:
      return None

API_HOST = "https://api.commadotai.com"
DEFAULT_OUT_DIR = "debug_logs"
GH_FILE_LIMIT_BYTES = 95 * 1024 * 1024   # GitHub hard-rejects > 100 MB; warn at 95
GH_PUSH_LIMIT_BYTES = 1900 * 1024 * 1024  # GitHub recommends < 2 GB push


def parse_args():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("route_id", help="Route, e.g. 0fb02cc3a5abcc2f/0000000b--99d3e5258c (or with '|')")
  p.add_argument("--rlog", action="store_true", help="Also download rlog.zst (large)")
  p.add_argument("--segments", default=None, help="Slice of segments, e.g. '0:5', '3', or ':10'")
  p.add_argument("--out-dir", default=DEFAULT_OUT_DIR, help=f"Output root (default: {DEFAULT_OUT_DIR}/)")
  p.add_argument("--branch", default=None, help="Refuse to run unless on this branch")
  p.add_argument("--jwt", default=None, help="Override JWT (else read from auth.json)")
  p.add_argument("--no-commit", action="store_true", help="Skip git add/commit/push")
  p.add_argument("--no-push", action="store_true", help="Commit but do not push")
  p.add_argument("--note", default="", help="Free-form note included in metadata + commit msg")
  p.add_argument("--force", action="store_true", help="Proceed even on size warnings")
  return p.parse_args()


def parse_slice(spec, n):
  if spec is None:
    return list(range(n))
  if ":" not in spec:
    i = int(spec)
    if i < 0:
      i += n
    return [i]
  start, end = spec.split(":", 1)
  s = int(start) if start else 0
  e = int(end) if end else n
  if s < 0:
    s += n
  if e < 0:
    e += n
  return [i for i in range(s, e) if 0 <= i < n]


def get_route_files(route_canonical, jwt):
  url = f"{API_HOST}/v1/route/{route_canonical}/files"
  r = requests.get(url, headers={"Authorization": f"JWT {jwt}"}, timeout=20)
  if r.status_code in (401, 403):
    sys.exit("ERROR: unauthorized. Re-run `python tools/lib/auth.py` or pass a fresh --jwt.")
  if r.status_code == 404:
    sys.exit(f"ERROR: route not found or not accessible: {route_canonical}")
  r.raise_for_status()
  return r.json()


def segment_index_from_url(url):
  # .../<dongle>/<time>/<seg_num>/<file>
  parts = url.split("?", 1)[0].rstrip("/").split("/")
  try:
    return int(parts[-2])
  except ValueError:
    return None


def download(url, dst):
  dst.parent.mkdir(parents=True, exist_ok=True)
  if dst.exists() and dst.stat().st_size > 0:
    print(f"  skip (exists): {dst} ({dst.stat().st_size / 1e6:.1f} MB)")
    return dst.stat().st_size
  tmp = dst.with_suffix(dst.suffix + ".part")
  size = 0
  with requests.get(url, stream=True, timeout=120) as r:
    r.raise_for_status()
    with open(tmp, "wb") as f:
      for chunk in r.iter_content(chunk_size=1 << 20):
        if chunk:
          f.write(chunk)
          size += len(chunk)
  tmp.rename(dst)
  print(f"  saved: {dst} ({size / 1e6:.1f} MB)")
  return size


def git(*args, check=True):
  return subprocess.run(["git", *args], check=check, capture_output=True, text=True)


def main():
  args = parse_args()
  jwt = args.jwt or get_token()
  if not jwt:
    sys.exit("ERROR: no auth token. Run `python tools/lib/auth.py` first or pass --jwt.")

  route_canonical = args.route_id.replace("/", "|")
  if route_canonical.count("|") != 1:
    sys.exit(f"ERROR: route ID must look like '<dongle>/<time>', got: {args.route_id}")

  if args.branch:
    cur = git("rev-parse", "--abbrev-ref", "HEAD").stdout.strip()
    if cur != args.branch:
      sys.exit(f"ERROR: on branch '{cur}', expected '{args.branch}'.")

  print(f"[1/4] Resolving {route_canonical} ...")
  files = get_route_files(route_canonical, jwt)
  qlogs = files.get("qlogs") or []
  rlogs = files.get("logs") or []

  qmap = {segment_index_from_url(u): u for u in qlogs if segment_index_from_url(u) is not None}
  rmap = {segment_index_from_url(u): u for u in rlogs if segment_index_from_url(u) is not None}
  if not qmap:
    sys.exit("ERROR: route has no qlogs (not yet uploaded?).")

  n_segments = max(qmap) + 1
  selected = parse_slice(args.segments, n_segments)
  print(f"     route has {n_segments} segment(s); selected {len(selected)}")

  safe_id = args.route_id.replace("/", "_").replace("|", "_")
  out_root = Path(args.out_dir) / safe_id

  print(f"[2/4] Downloading to {out_root}/ ...")
  total = 0
  big_files = []
  for i in selected:
    seg_dir = out_root / f"{i:04d}"
    if i in qmap:
      sz = download(qmap[i], seg_dir / "qlog.zst")
      total += sz
      if sz > GH_FILE_LIMIT_BYTES:
        big_files.append(seg_dir / "qlog.zst")
    else:
      print(f"  seg {i}: qlog missing")
    if args.rlog:
      if i in rmap:
        sz = download(rmap[i], seg_dir / "rlog.zst")
        total += sz
        if sz > GH_FILE_LIMIT_BYTES:
          big_files.append(seg_dir / "rlog.zst")
      else:
        print(f"  seg {i}: rlog missing")

  metadata = {
    "route_id": args.route_id,
    "canonical": route_canonical,
    "segments_total": n_segments,
    "segments_fetched": selected,
    "with_rlog": args.rlog,
    "note": args.note,
    "total_bytes": total,
  }
  (out_root / "metadata.json").write_text(json.dumps(metadata, indent=2))
  print(f"     {total / 1e6:.1f} MB total")

  if big_files:
    print("\n  WARNING: files exceed GitHub's 100 MB hard limit:")
    for p in big_files:
      print(f"    {p}  ({p.stat().st_size / 1e6:.1f} MB)")
    if not args.force:
      sys.exit("Aborting commit. Re-run without --rlog, narrow --segments, or pass --force after setting up Git LFS.")

  if total > GH_PUSH_LIMIT_BYTES and not args.force:
    sys.exit(f"Total {total / 1e9:.1f} GB exceeds safe push size. Narrow --segments, drop --rlog, or use --force.")

  if args.no_commit:
    print("\n[3/4] --no-commit: stopping after download.")
    print(f"      logs at: {out_root}/")
    return

  print("\n[3/4] git add + commit ...")
  git("add", "--", str(out_root))
  staged = git("diff", "--cached", "--name-only").stdout.strip()
  if not staged:
    print("      nothing new to commit (already up to date).")
    return
  msg = f"debug: logs for {args.route_id}"
  if args.note:
    msg += f"\n\n{args.note}"
  git("commit", "-m", msg)

  if args.no_push:
    print("[4/4] --no-push: commit stays local.")
    return

  branch = git("rev-parse", "--abbrev-ref", "HEAD").stdout.strip()
  print(f"[4/4] git push -u origin {branch} ...")
  res = git("push", "-u", "origin", branch, check=False)
  sys.stdout.write(res.stdout)
  sys.stderr.write(res.stderr)
  if res.returncode != 0:
    sys.exit("git push failed")
  print(f"\nDone. Logs committed under {out_root}/")
  print("Tell Claude: \"analyze logs in " + str(out_root) + "\" along with the approximate crash time.")


if __name__ == "__main__":
  main()
