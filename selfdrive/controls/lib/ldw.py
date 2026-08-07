import math
from collections import deque

from cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.common.constants import CV


CAMERA_OFFSET = 0.04
LDW_MIN_SPEED = 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1

# Phase 6g-4: op-active LDW. Upstream LDW is hard-gated off while openpilot
# steers (`not CC.latActive`), which on a MADS fork means it is effectively
# always off — and the model's desirePrediction stays ~0 while op lane-keeps
# (it predicts intent, not tracking error), so the manual-driving trigger
# cannot be reused. Instead, fire on a CONTINUITY-GATED clearance approach:
# the inner lane line is well-tracked (prob > 0.5 for the whole window),
# closer than OP_LDW_CLR_M, monotonically approaching (no frame regresses
# more than OP_LDW_STEP_TOL_M), with at least OP_LDW_MIN_APPROACH_M total
# approach over the window, and no relabel jump (frame-to-frame change
# bounded by OP_LDW_JUMP_M — lane re-identification at merges/forks produces
# step discontinuities that must not fire). Replayed on ccnc-drivelog
# 0x40-0x44/0x48: 0 false fires, 1 genuine sustained approach caught.
# Kill switch: OP_LDW_CLR_M = 0.0 (op-active branch never fires).
OP_LDW_CLR_M = 0.7            # fire only when the line is this close (m)
OP_LDW_WINDOW = 7             # model frames (~0.3 s @ 20 Hz)
OP_LDW_MIN_PROB = 0.5
OP_LDW_STEP_TOL_M = 0.02      # per-frame regression tolerance (still "approaching")
OP_LDW_JUMP_M = 0.3           # larger per-frame change = lane relabel, ignore
OP_LDW_MIN_APPROACH_M = 0.10  # total approach over the window


class LaneDepartureWarning:
  def __init__(self):
    self.left = False
    self.right = False
    self.last_blinker_frame = 0
    self._hist_l: deque = deque(maxlen=OP_LDW_WINDOW)
    self._hist_r: deque = deque(maxlen=OP_LDW_WINDOW)

  @staticmethod
  def _op_active_side_departure(hist) -> bool:
    if len(hist) < OP_LDW_WINDOW:
      return False
    probs = [p for p, _ in hist]
    clrs = [c for _, c in hist]
    # every gate below is a comparison, and NaN compares False against
    # everything — a single NaN frame would therefore pass ALL the
    # continuity gates instead of none. The detector's contract is
    # "every frame confirms", so a corrupted window must not fire.
    if not all(map(math.isfinite, probs)) or not all(map(math.isfinite, clrs)):
      return False
    if min(probs) <= OP_LDW_MIN_PROB or clrs[-1] >= OP_LDW_CLR_M:
      return False
    steps = [b - a for a, b in zip(clrs, clrs[1:])]
    if any(abs(s) >= OP_LDW_JUMP_M for s in steps):    # relabel jump
      return False
    if any(s >= OP_LDW_STEP_TOL_M for s in steps):     # not approaching
      return False
    return (clrs[0] - clrs[-1]) > OP_LDW_MIN_APPROACH_M

  def update(self, frame, modelV2, CS, CC):
    if CS.leftBlinker or CS.rightBlinker:
      self.last_blinker_frame = frame

    recent_blinker = (frame - self.last_blinker_frame) * DT_CTRL < 5.0  # 5s blinker cooldown
    lane_lines = modelV2.laneLines
    probs = modelV2.laneLineProbs

    if CC.latActive:
      # Phase 6g-4: op-active path (see constants above).
      base_allowed = (CS.vEgo > LDW_MIN_SPEED and not recent_blinker and OP_LDW_CLR_M > 0.0
                      and len(probs) >= 3 and len(lane_lines) >= 3
                      # model glitch frames can carry empty y arrays even when
                      # laneLines itself has entries — y[0] below would IndexError
                      # and take plannerd down with it
                      and len(lane_lines[1].y) > 0 and len(lane_lines[2].y) > 0
                      and modelV2.meta.laneChangeState == log.LaneChangeState.off)
      if base_allowed:
        self._hist_l.append((float(probs[1]), abs(float(lane_lines[1].y[0]))))
        self._hist_r.append((float(probs[2]), abs(float(lane_lines[2].y[0]))))
        self.left = self._op_active_side_departure(self._hist_l)
        self.right = self._op_active_side_departure(self._hist_r)
      else:
        self._hist_l.clear()
        self._hist_r.clear()
        self.left, self.right = False, False
      return

    # Manual-driving path (upstream behaviour, unchanged).
    self._hist_l.clear()
    self._hist_r.clear()
    ldw_allowed = CS.vEgo > LDW_MIN_SPEED and not recent_blinker

    desire_prediction = modelV2.meta.desirePrediction
    if len(desire_prediction) and ldw_allowed:
      right_lane_visible = probs[2] > 0.5
      left_lane_visible = probs[1] > 0.5
      l_lane_change_prob = desire_prediction[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_prediction[log.Desire.laneChangeRight]

      l_lane_close = left_lane_visible and (lane_lines[1].y[0] > -(1.08 + CAMERA_OFFSET))
      r_lane_close = right_lane_visible and (lane_lines[2].y[0] < (1.08 - CAMERA_OFFSET))

      self.left = bool(l_lane_change_prob > LANE_DEPARTURE_THRESHOLD and l_lane_close)
      self.right = bool(r_lane_change_prob > LANE_DEPARTURE_THRESHOLD and r_lane_close)
    else:
      self.left, self.right = False, False

  @property
  def warning(self) -> bool:
    return bool(self.left or self.right)
