"""E77: which CAN bits differ between a boot WITH the ADAS warnings and normal boots, in the stock window before the
relay engages (t 2-9 s). Per (bus, address): bits constant in each boot; report bits whose constant value in the bad boot
differs from the value in ALL good boots, plus addresses present in only one side. Then decode those messages with the
hyundai_canfd DBC signal names covering the differing bits."""
import glob
import sys
from collections import defaultdict
import numpy as np
from openpilot.tools.lib.logreader import LogReader
BAD = sys.argv[1]; GOOD = sys.argv[2].split(",")
T0, T1 = 2.0, 9.0


def collect(r):
  f = glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--0--rlog.zst")[0]
  t0 = None; d = defaultdict(list)
  for m in LogReader(f):
    t = m.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    if t - t0 > T1:
      break
    if m.which() == "can" and t - t0 >= T0:
      for c in m.can:
        if c.src < 3:
          d[(c.src, c.address)].append(bytes(c.dat))
  return d


def const_bits(msgs):
  L = max(len(x) for x in msgs)
  A = np.zeros((len(msgs), L * 8), np.uint8)
  for i, x in enumerate(msgs):
    b = np.unpackbits(np.frombuffer(x.ljust(L, b"\0"), np.uint8))
    A[i] = b
  allone = A.all(0); allzero = (~A.astype(bool)).all(0)
  return {i: 1 for i in np.flatnonzero(allone)} | {i: 0 for i in np.flatnonzero(allzero)}, len(msgs)


bad = collect(BAD); goods = [collect(g) for g in GOOD]
only_bad = sorted(set(bad) - set().union(*[set(g) for g in goods]))
only_good = sorted(set().union(*[set(g) for g in goods]) & set.intersection(*[set(g) for g in goods]) - set(bad))
print(f"addresses only in the bad boot: {[(b, hex(a), len(bad[(b, a)])) for b, a in only_bad][:30]}")
print(f"addresses in every good boot but not the bad one: {[(b, hex(a)) for b, a in only_good][:30]}")
diffs = []
for key in sorted(set(bad) & set.intersection(*[set(g) for g in goods])):
  cb, nb = const_bits(bad[key])
  cg = [const_bits(g[key])[0] for g in goods]
  for bit, val in cb.items():
    if all(bit in c and c[bit] != val for c in cg):
      diffs.append((key, bit, val))
by = defaultdict(list)
for key, bit, val in diffs:
  by[key].append((bit, val))
print(f"(bus, addr) with bits that differ consistently (bad vs every good boot): {len(by)}")
for key, L in sorted(by.items()):
  print(f"  bus {key[0]} 0x{key[1]:03x} n {len(bad[key])}: " + " ".join(f"byte{b // 8}.bit{7 - b % 8}={v}" for b, v in L[:16]))
