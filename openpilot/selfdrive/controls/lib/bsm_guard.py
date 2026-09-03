"""Phase 37b: BSM lane guard for the op-active path.

When the BSM radar reports a side occupied, the lane line on that side is
close, and the driver is NOT signalling toward it, op must not steer further
toward that side. Same mechanism as the existing lane-departure gate in
controlsd (hold the previous curvature), but keyed on radar + lane position
instead of the LDW predictor, which needs a sustained monotonic approach and
did not fire once in ~2 h of corpus (0x58/0x5c/0x5d: 6 relabel jumps rejected
correctly, 1 fast crossing over the jump gate).

Sign conventions (this tree): desired curvature NEGATIVE = left (controlsd
negates VM.calc_curvature; corpus check sign(wheel*k) = -0.89 over |angle| > 3 deg,
n=19008, with wheel positive = left); lane line y NEGATIVE = left (laneLines[1].y[0] ~ -1.7).
Kill: BSM_LANE_GUARD_M = 0.0.
"""

BSM_LANE_GUARD_M = 0.9        # line closer than this (m) on the occupied side
BSM_LANE_GUARD_MIN_PROB = 0.5
BSM_LANE_GUARD_MAX_S = 2.0    # hold no longer than this per episode (tightening bend + parked truck)


def bsm_lane_guard(new_k: float, prev_k: float, left_bs: bool, right_bs: bool,
                   left_blinker: bool, right_blinker: bool,
                   left_y: float, right_y: float, left_p: float, right_p: float,
                   guard_m: float = BSM_LANE_GUARD_M, min_prob: float = BSM_LANE_GUARD_MIN_PROB):
  """Return (curvature, blocked). Blocks only the component that moves the car
  further toward the occupied side; the driver's own blinker toward that side
  lifts the guard (their decision, and the blinker concession logic in the
  car controller handles it)."""
  if guard_m <= 0.0:
    return new_k, False
  if (left_bs and not left_blinker and left_p > min_prob and abs(left_y) < guard_m
      and new_k < prev_k):
    return prev_k, True
  if (right_bs and not right_blinker and right_p > min_prob and abs(right_y) < guard_m
      and new_k > prev_k):
    return prev_k, True
  return new_k, False


class BsmLaneGuard:
  """Stateful wrapper: the hold is released after BSM_LANE_GUARD_MAX_S of
  continuous blocking so a tightening bend toward an occupied side cannot run
  the car wide indefinitely; the counter resets on the first unblocked frame."""

  def __init__(self, dt: float, max_s: float = BSM_LANE_GUARD_MAX_S):
    self.max_frames = int(round(max_s / dt))
    self.frames = 0

  def reset(self):
    self.frames = 0

  def update(self, new_k: float, prev_k: float, *args, **kwargs):
    k, blocked = bsm_lane_guard(new_k, prev_k, *args, **kwargs)
    if not blocked:
      self.frames = 0
      return new_k, False
    self.frames += 1
    if self.frames > self.max_frames:
      return new_k, False
    return k, True
