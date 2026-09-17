"""Phase 41: low-lane-confidence steering-RATE cap for the op-active angle path.

Finding (i6nv3 routes 0x17-0x1f, 09-15..09-17, report §23): of 162 op swings >= 4 deg in 0.5 s (lat active, no
blinker, no lane change, >= 30 km/h, driver not pressing at onset) the driver fought 21 with opposite torque; 14 of
those sat where the weaker inner lane line was below 0.3 (33 % of that band's swings vs 2 % above 0.8). All 14 were
the MODEL plan re-centring on a merged/split lane (half had a > 0.3 m lane-width change in 2 s) and op followed it.
Nothing in the logs separates them from the 23 legitimate swings in the same band (lane-width change, plan shift,
speed, request rate, swing toward the weaker line — all overlap), and the two existing guards do not reach them:
the 6g confidence blend mixes per 100 Hz frame so a 0.3 confidence passes the plan within a few frames, and the
39-2 dropout latch needs < 0.15 for 0.3 s at >= 40 km/h (0 of the 162 swings qualified).

Policy, not a discriminator: while the weaker inner line is below LOWCONF_LANE_MIN and the driver is not holding
the wheel, the commanded curvature may change no faster than LOWCONF_CAP_DPS of steering angle per second. A wrong
re-centring becomes a slow drift the driver can veto instead of a snatch; a legitimate corner in the same band
arrives ~0.5-1 s later (the driver was already adding torque in 13 of those 23). Exposure on the 7 routes: 6.8 %
of active >= 30 km/h time is in the band hands-off; a 6 deg/s cap withholds > 2 deg for 0.24 min in 68 min.
Release: when the gate drops the cap ramps from LOWCONF_CAP_DPS to LOWCONF_RELEASE_DPS over LOWCONF_RELEASE_S so
the plan the cap held back cannot land as one step (the snatch, delayed). A driver press, blinker or lane change
lifts the cap at once (the driver has the wheel; the anchor logic in the car controller takes over).
Adoption criteria (§23): opposite-torque grabs after low-confidence swings down >= 50 % per hour; same-direction
assists in the band up no more than 30 %. Kill: LOWCONF_CAP_DPS = 0.0.
"""
import math

LOWCONF_CAP_DPS = 6.0          # steering-angle rate cap while gated (deg/s); 0.0 = off
LOWCONF_RELEASE_DPS = 30.0     # cap at the end of the release ramp (above any corpus corner-entry rate: p50 ~20 deg/s)
LOWCONF_RELEASE_S = 1.0        # ramp length after the gate drops
LOWCONF_LANE_MIN = 0.3         # gate: min(inner-left, inner-right) lane-line prob below this
LOWCONF_MIN_SPEED = 30.0 / 3.6 # m/s; below this the plan is right where the lines leave the camera (39-2 finding)
NOMINAL_K_PER_DEG = math.radians(1.0) / (2.95 * 15.0)   # fallback curvature per deg of steer (Ioniq 6 wheelbase x ~steer ratio)


class LowConfRateCap:
  def __init__(self, dt: float):
    self.dt = dt
    self.release_frames = int(round(LOWCONF_RELEASE_S / dt))
    self.release_left = 0
    self.active = False      # gate on this frame (telemetry / tests)
    self.capped = False      # the cap actually held the command back this frame

  def reset(self):
    self.release_left = 0
    self.active = False
    self.capped = False

  def update(self, new_k: float, prev_k: float, lane_min: float, pressed: bool, blinker_or_lc: bool,
             v_ego: float, k_per_deg: float, cap_dps: float = LOWCONF_CAP_DPS) -> float:
    """Return the curvature to command. prev_k is the curvature commanded on the previous frame."""
    self.capped = False
    if cap_dps <= 0.0:
      self.active = False
      return new_k
    if pressed or blinker_or_lc:
      # the driver has the wheel or has declared a manoeuvre: no cap, and no release ramp to trip over afterwards
      self.release_left = 0
      self.active = False
      return new_k
    gate = (lane_min < LOWCONF_LANE_MIN) and (v_ego >= LOWCONF_MIN_SPEED)
    self.active = gate
    if gate:
      self.release_left = self.release_frames
      dps = cap_dps
    elif self.release_left > 0:
      # linear ramp cap_dps -> LOWCONF_RELEASE_DPS as release_left runs down
      frac = 1.0 - self.release_left / float(self.release_frames)
      dps = cap_dps + (LOWCONF_RELEASE_DPS - cap_dps) * frac
      self.release_left -= 1
    else:
      return new_k
    kpd = k_per_deg if (math.isfinite(k_per_deg) and k_per_deg > 1e-7) else NOMINAL_K_PER_DEG
    step = dps * kpd * self.dt
    delta = new_k - prev_k
    if abs(delta) > step:
      self.capped = True
      return prev_k + math.copysign(step, delta)
    return new_k
