"""proclogd: the PSS (smaps) refresh must be staggered across cycles, not burst every 40 s."""
import importlib
import sys
import types
from collections import Counter

import pytest


@pytest.fixture
def proclogd(monkeypatch):
  # openpilot.system.proclogd imports messaging/realtime/swaglog at module level; stub them so the
  # scheduling helpers can be exercised without the device runtime.
  for name in ("openpilot.cereal.messaging", "openpilot.common.realtime", "openpilot.common.swaglog"):
    mod = types.ModuleType(name)
    if name.endswith("realtime"):
      mod.Ratekeeper = object
    if name.endswith("swaglog"):
      mod.cloudlog = types.SimpleNamespace(exception=lambda *a, **k: None)
    monkeypatch.setitem(sys.modules, name, mod)
  monkeypatch.delitem(sys.modules, "openpilot.system.proclogd", raising=False)   # restored at teardown
  return importlib.import_module("openpilot.system.proclogd")


def _reset(proclogd):
  proclogd._proc_cache.clear()
  proclogd._smaps_cache.clear()
  proclogd._smaps_slot.clear()
  proclogd._smaps_next_slot = 0
  proclogd._smaps_cycle = 0


def test_each_pid_refreshed_once_per_period_in_own_slot(proclogd, monkeypatch):
  reads = []
  tick = [0]  # absolute cycle index (the module counter wraps every _SMAPS_EVERY cycles)
  monkeypatch.setattr(proclogd, "_read_smaps", lambda pid: (reads.append((tick[0], pid)) or {'pss': pid, 'pss_anon': 0, 'pss_shmem': 0}))
  pids = list(range(1000, 1092, 2))  # 46 processes with an even pid stride (the case pid % 20 would pile up)
  _reset(proclogd)
  for _cycle in range(3 * proclogd._SMAPS_EVERY):
    for pid in pids:
      assert proclogd._get_smaps_cached(pid)['pss'] == pid
    proclogd._end_sweep(set(pids))   # the production increment, wrap included
    assert 0 <= proclogd._smaps_cycle < proclogd._SMAPS_EVERY
    tick[0] += 1
  per_cycle = Counter(c for c, _ in reads[len(pids):])  # skip the initial population reads
  # 46 pids over 20 round-robin slots: at most ceil(46/20) = 3 reads in any later cycle (was 46 in one cycle)
  assert max(per_cycle.values()) <= 3
  per_pid = Counter(p for _, p in reads)
  # first read + one refresh per period over 3 periods (the first read doubles as the refresh for slot-0 pids)
  assert all(3 <= n <= 4 for n in per_pid.values()), per_pid
  assert sum(per_pid.values()) == len(pids) + 3 * len(pids) - sum(1 for p in pids if proclogd._smaps_slot[p] == 0)


def test_first_sight_reads_immediately(proclogd, monkeypatch):
  monkeypatch.setattr(proclogd, "_read_smaps", lambda pid: {'pss': 7, 'pss_anon': 1, 'pss_shmem': 2})
  _reset(proclogd)
  proclogd._smaps_cycle = 5
  assert proclogd._get_smaps_cached(4242)['pss'] == 7   # slot 0 != cycle 5: still read on first sight


def test_dead_pids_are_evicted_and_reused_pid_gets_fresh_slot(proclogd, monkeypatch):
  monkeypatch.setattr(proclogd, "_read_smaps", lambda pid: {'pss': 1, 'pss_anon': 0, 'pss_shmem': 0})
  _reset(proclogd)
  for pid in (10, 11, 12):
    proclogd._get_smaps_cached(pid)
  proclogd._end_sweep({10, 12})
  assert 11 not in proclogd._smaps_cache and 11 not in proclogd._smaps_slot
  proclogd._get_smaps_cached(11)               # pid reused by a new process: re-read and re-slotted
  assert proclogd._smaps_slot[11] == 3


def test_exe_cmdline_cache_evicted_with_dead_pids(proclogd):
  _reset(proclogd)
  proclogd._proc_cache.clear()
  for pid in (20, 21):
    proclogd._proc_cache[pid] = {'pid': pid, 'name': 'x', 'exe': '', 'cmdline': []}
  proclogd._end_sweep({20})
  assert 21 not in proclogd._proc_cache and 20 in proclogd._proc_cache
