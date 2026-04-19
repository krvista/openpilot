#!/usr/bin/env python3
"""Determine which build routes 42/43 were captured on, and look for
a `card` crash logMessage at the exact timestamp TX goes silent."""
import glob, sys, json

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

for route_id in ['0000002a', '0000002b']:
  paths = sorted(glob.glob(f'/home/user/openpilot/drivelog/99b215d21bbf8735_{route_id}--*--rlog.zst'),
                 key=lambda p: int(p.rsplit('/',1)[-1].split('--')[2]))
  if not paths: continue
  print(f'\n=== Route {route_id}: first-segment build + crash probe ===')

  version = None
  git_commit = None
  crash_msgs = []
  t0 = None
  for m in LogReader(paths[0]):
    try: w = m.which()
    except Exception: continue
    if t0 is None: t0 = m.logMonoTime
    t_ms = (m.logMonoTime - t0) / 1e6
    if w == 'initData':
      version = str(m.initData.version)
      git_commit = str(m.initData.gitCommit)
    elif w == 'logMessage':
      txt = str(m.logMessage)
      if 'error' in txt.lower() or 'crash' in txt.lower() or 'traceback' in txt.lower() \
         or 'capnp' in txt.lower() or 'kjexception' in txt.lower():
        crash_msgs.append((t_ms, txt[:300]))
    elif w == 'procLog':
      pass
  print(f'  version:    {version}')
  print(f'  gitCommit:  {git_commit}')
  print(f'  logMessage ERROR/CRASH count (seg 0): {len(crash_msgs)}')
  for t, msg in crash_msgs[:15]:
    print(f'    t={t:8.1f}ms  {msg}')

  # Also scan seg 1 to see if processes are even alive
  if len(paths) > 1:
    print(f'  -- seg 1 quick probe --')
    cnt = 0
    crashes1 = 0
    for m in LogReader(paths[1]):
      try: w = m.which()
      except Exception: continue
      cnt += 1
      if w == 'logMessage':
        txt = str(m.logMessage)
        if 'error' in txt.lower() or 'crash' in txt.lower() or 'traceback' in txt.lower():
          crashes1 += 1
    print(f'  seg 1: total msgs={cnt}, error/crash logMessages={crashes1}')
