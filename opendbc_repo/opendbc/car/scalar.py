"""Scalar stand-ins for np.interp / np.clip / np.sign on the 100 Hz control path.

numpy's per-call overhead (~3-5 us each) dominated the car controller: route 00000005 profile showed
~19 np.clip + ~23 np.interp per frame (~36 % of CarController.update) with every call site scalar.
These reproduce numpy's arithmetic exactly, so outputs are bit-identical:
  interp: slope * (x - xp[j]) + fp[j] with slope = (fp[j+1] - fp[j]) / (xp[j+1] - xp[j]), ends clamped,
          NaN in -> NaN out (ascending xp, as every table here is)
  clip:   NaN passes through (as np.clip)
  sign:   NaN -> NaN, 0 -> 0.0
"""


def interp(x, xp, fp):
  if x != x:
    return float("nan")
  # right edge first: numpy tests x == xp[-1] before the left edge, which matters for a degenerate
  # [X, X] table (a flat "kill switch" pair) at x == X -> fp[-1]
  if x >= xp[-1]:
    return float(fp[-1])
  if x <= xp[0]:
    return float(fp[0])
  for j in range(len(xp) - 1):
    if x < xp[j + 1]:
      return (fp[j + 1] - fp[j]) / (xp[j + 1] - xp[j]) * (x - xp[j]) + fp[j]
  return float(fp[-1])


def clip(v, lo, hi):
  if lo != lo or hi != hi:      # NaN bound -> NaN, as np.clip
    return float("nan")
  return lo if v < lo else (hi if v > hi else v)


def sign(x):
  if x != x:
    return float("nan")
  return 1.0 if x > 0 else (-1.0 if x < 0 else 0.0)
