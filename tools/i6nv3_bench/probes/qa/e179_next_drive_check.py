"""E179 (report 117): first-pass check of a new drive against the a8 / a9 numbers - one line per item of the next-drive list
(Phases 80-84 have not been driven yet): id, verdict (PASS / FAIL / CHECK / NO DATA), value, the same value on the baseline
routes, and the time stamps of the worst events so that someone can look at the video.

  python e179_next_drive_check.py <route.npz> [--baseline a8.npz a9.npz]

Input is the route_extract.py npz (v7 runs; the v8 fields - raw 0x400 blinker state, 0x3E3 lamps, alert texts, free space,
GPS fix, canValid, frame drops, build identity - give NO DATA on a v7 file). Items that need the new record bits (steerFlags
14 lane fit / 15 avoidance release) give NO DATA on older builds. CHECK = too few samples, between the pass and fail lines,
or a person has to look. A series an older npz lacks gets a neutral default (listed on the BUILD-1 line), never a KeyError. What is NOT computed here (controlsd replay, video, the owner's impression) is listed at the end.

Signs: steering angle / driver torque + = left; curvature + = right; pos (factory camera) + = car right of the lane centre.
Masks: rel = op's frame reaches the MDPS; ho = rel, no press +-1 s, no blinker / lane change +-3 s; st = straight,
cv = steady curve (both ho, both factory lines good); op = car controller active on the wire; hoc = no press, |torque| < 150."""
import sys
from collections import Counter

import numpy as np

np.seterr(all="ignore")
HEAD = "2fcd4ed9"                                  # Phase 84
OLD_BUILDS = ("19454511", "dcfcdb1f")              # a8 / a9, a7
HALF_W = 0.94                                      # m, half body width: clearance of the body edge to a line
EV_OK = {"steerOverride", "gasPressedOverride", "preLaneChangeLeft", "preLaneChangeRight", "laneChange", "laneChangeBlocked",
         "laneDropout", "selfdriveInitializing", "preEnableStandstill", "steerSaturated"}
EV_BAD = {"controlsMismatch", "canError", "canBusMissing", "steerUnavailable", "steerTempUnavailable", "commIssue",
          "posenetInvalid", "locationdTemporaryError", "paramsdTemporaryError", "modeldLagging", "controlsdLagging",
          "processNotRunning", "outOfSpace", "overheat", "cameraMalfunction", "usbError"}
# series an older npz may lack -> neutral default, so the run never stops on a KeyError (the items then read NO DATA / 0)
DEFAULTS = dict(cb_lvl2=2, mdps_active=2, tx_active=2, lat_reason=0, lat_paused=0, lat_granted=1, lfa_sym=-1, gear=0, cruise=0,
                mdps_fault=0, tx_reject=0, bsl=0, bsr=0, sf_ctl=0, sf_cc=0, gain=1.0, lcs=0, lane_min=1.0, rate=0.0, tq_s=0.0,
                k_model=0.0, k_ctrl=0.0, k_meas=0.0, apply=0.0, wire=0.0, lb=0, rb=0, pressed=0, lat=0)
BOOL = ("cruise", "lat_paused", "lat_granted", "bsl", "bsr", "lb", "rb", "pressed", "lat")
ROWS = []


# ---------------------------------------------------------------- helpers
def box(x, n):
  x = np.asarray(x, float)
  return np.convolve(np.r_[np.full(n // 2, x[0]), x, np.full(n - n // 2 - 1, x[-1])], np.ones(n) / n, "valid")


def runs(m):
  e = np.flatnonzero(np.diff(np.r_[0, np.asarray(m).astype(np.int8), 0]))
  return list(zip(e[::2], e[1::2], strict=True))


def held(m, nfr):
  """True where m has been True for at least nfr frames."""
  c = np.zeros(len(m), int); k = 0
  for i, x in enumerate(m):
    k = k + 1 if x else 0; c[i] = k
  return c >= nfr


def first_run(m, nfr):
  h = held(m, nfr)
  return int(np.argmax(h)) - nfr + 1 if h.any() else None


def boot_se(x, blk):
  ids = np.unique(blk)
  if len(ids) < 3:
    return float("nan")
  m = np.array([x[blk == i].mean() for i in ids]); w = np.array([(blk == i).sum() for i in ids])
  j = np.random.default_rng(0).integers(0, len(ids), (400, len(ids)))
  return float(np.std((m[j] * w[j]).sum(1) / w[j].sum(1)))


def band(x, lo_s, hi_s):
  return box(x, int(lo_s * 50)) - box(x, int(hi_s * 100))


def med(x):
  x = np.asarray(x, float); x = x[np.isfinite(x)]
  return float(np.median(x)) if len(x) else float("nan")


def pct(x, q):
  x = np.asarray(x, float); x = x[np.isfinite(x)]
  return float(np.percentile(x, q)) if len(x) else float("nan")


def bit(a, k):
  return ((a >> k) & 1).astype(bool)


def ts(times, k=4):
  t = [f"{x:.1f}" for x in list(times)[:k]]
  return ("t=" + ",".join(t) + (" .." if len(list(times)) > k else "")) if t else ""


def vd(ok, fail=None, enough=True):
  if fail if fail is not None else not ok:
    return "FAIL"
  return "PASS" if ok and enough else "CHECK"


def row(i, what, val, verdict, when=""):
  ROWS.append((i, what, val, verdict, when))


# ---------------------------------------------------------------- load
def load(p):
  d = dict(np.load(p, allow_pickle=True)); T = d["T"]; n = len(T); v = d["v"]
  miss = [k for k in DEFAULTS if k not in d]
  for k in miss:
    d[k] = np.full(n, DEFAULTS[k], dtype=bool if k in BOOL else type(DEFAULTS[k]))
  for k in BOOL:
    d[k] = d[k].astype(bool)
  d.setdefault("commit", np.array(""))
  o = dict(d=d, T=T, n=n, v=v, path=p, ver=int(d["version"]) if "version" in d else 0, miss=miss)
  pr = d["pressed"].astype(bool); lat = d["lat"].astype(bool)
  bl = (d["lb"] | d["rb"] | (d["lcs"] > 0)).astype(bool)
  rel = (d["tx_active"] == 2) & (d["cb_lvl2"] == 2) & (d["mdps_active"] == 2) & lat
  o.update(pr=pr, lat=lat, bl=bl, rel=rel, ho=rel & ~(box(pr, 200) > 0) & ~(box(bl, 600) > 0))
  o["has_fcam"] = "fcam_l" in d and (d["fcam_ql"] >= 2).any()
  fl, fr = (d["fcam_l"], d["fcam_r"]) if "fcam_l" in d else (np.zeros(n), np.zeros(n))
  ql, qr = (d["fcam_ql"], d["fcam_qr"]) if "fcam_l" in d else (np.full(n, -1), np.full(n, -1))
  w = fr - fl
  o.update(fl=fl, fr=fr, ql=ql, qr=qr, pos=-(fl + fr) / 2 * 100.0, fok=(ql >= 2) & (qr >= 2) & (w > 2.5) & (w < 4.6))
  o.update(km=d["k_model"], kc=d["k_ctrl"], kme=d["k_meas"], ks=box(d["k_meas"], 100), kms=box(d["k_model"], 30))
  o["sft"] = d["sf_ctl"].astype(np.int64); o["sfc"] = d["sf_cc"].astype(np.int64); o["blk"] = (T // 10).astype(int)
  o["good"] = o["ho"] & o["fok"] & (d["lane_min"] >= 0.5)
  o["st"] = o["good"] & (np.abs(o["ks"]) < 0.5e-3) & (np.abs(o["kms"]) < 0.5e-3)
  ksl = np.r_[o["ks"][100:], np.full(100, o["ks"][-1])]; ksb = np.r_[np.full(100, o["ks"][0]), o["ks"][:-100]]
  o["cv"] = o["good"] & (np.abs(o["ks"]) >= 1.0e-3) & (np.abs(ksl - ksb) < 0.3 * np.abs(o["ks"]))
  o["op"] = bit(o["sfc"], 0) & bit(o["sfc"], 1)
  o["hoc"] = ~pr & (np.abs(d["tq_s"]) < 150) & ~bit(o["sfc"], 8)
  o["park"] = bit(o["sfc"], 6)
  o["b14"] = bit(o["sft"], 14); o["b15"] = bit(o["sft"], 15)
  o["new_bits"] = bool(o["b14"].any() or o["b15"].any())
  # controlsd sets bit 15 only without press / blinker / lane change / dropout hold (lc_ok). The log lags that by a frame or
  # two, so 'blocked' = the blocking context has lasted >= 4 frames. b15_new = episodes that begin while the context has been
  # free for 5 frames (a new release); an episode that begins on the frame the context frees is the 1.5 s recovery of a
  # release from before it (the weight is not reset by a lane change) and is no new release.
  blocked = pr | bl | bit(o["sft"], 5); o["lc_blocked"] = held(blocked, 4); new = np.zeros(n, bool)
  for a, e in runs(o["b15"]):
    if a >= 5 and not blocked[a - 5:a].any():
      new[a:e] = True
  o["b15_new"] = new
  # blinker truth: the raw 0x400 state when the npz has it (v8), else carState (equal from Phase 80a on)
  if "blink_state" in d and (d["blink_state"] >= 0).any():
    s = d["blink_state"]; o["tl"] = ((s >= 1) & (s <= 3)) | (s == 7); o["tr"] = (s >= 4) & (s <= 7); o["raw_blink"] = True
  else:
    o["tl"] = d["lb"].astype(bool); o["tr"] = d["rb"].astype(bool); o["raw_blink"] = False
  o["ev"] = [(float(e[0]), str(e[1])) for e in d["events"]] if "events" in d else []
  o["hours"] = max((v > 1).sum() / 360000.0, 1e-6)
  o["smean"] = o["pos"][o["st"] & (v >= 45)].mean() if (o["st"] & (v >= 45)).sum() > 100 else 0.0
  return o


# ---------------------------------------------------------------- build, log, data sufficiency
def build_and_log(o):
  d, T, v = o["d"], o["T"], o["v"]
  c = str(d["commit"]); new = len(c) >= 7 and (c.startswith(HEAD[:len(c)]) or HEAD.startswith(c[:8]))
  extra = ""
  if "commit_full" in d:
    extra = (f", branch {d['branch']}, dirty {int(d['dirty'])}, GitDiff {int(d['git_diff_len'])} B, initData commits "
             f"{len(d['commits'])}, '{d['build_desc']}'")
  bad = "commit_full" in d and (int(d["dirty"]) == 1 or int(d["git_diff_len"]) > 0 or len(d["commits"]) > 1)
  old = any(c.startswith(x) for x in OLD_BUILDS)
  ver = "FAIL" if old or bad else ("PASS" if new and o["new_bits"] else "CHECK")
  row("BUILD-1", "build = Phase 84 (2fcd4ed9) or a descendant", f"commit {c}{extra}; bit14 frames {int(o['b14'].sum())}"
      + ("" if new or old else f" -> run: git merge-base --is-ancestor {HEAD} {c}")
      + (f"; fields missing in this npz (defaults used): {','.join(o['miss'])}" if o["miss"] else ""), ver)
  dt = np.diff(T); val = f"npz v{o['ver']}, {T[-1] - T[0]:.0f} s, carState gap max {dt.max():.3f} s, gaps > 0.5 s: {int((dt > 0.5).sum())}"
  ok = (dt <= 0.5).all()
  if "segs" in d and len(d["segs"]):
    sg = np.sort(d["segs"]); miss = sorted(set(range(int(sg[-1]) + 1)) - set(sg.tolist()))
    val += f", segments {len(sg)} (0..{sg[-1]}), missing {miss}, truncated {d['seg_err'].tolist()}"
    ok = ok and not miss
  row("BUILD-2", "log complete, extractor v8", val, vd(ok, enough=o["ver"] >= 8), ts(T[1:][dt > 0.5]))


def sufficiency(o):
  T, v = o["T"], o["v"]; ho = o["ho"]
  b = [(ho & (v >= lo) & (v < hi)).sum() / 100 for lo, hi in ((45, 60), (60, 80), (80, 85), (85, 300))]
  c = o["cv"] & (v >= 45) & (v < 80); ne = sum(1 for a, e in runs(c) if e - a >= 100)
  seg65 = sum(e - a for a, e in straight_segs(o, 65, 300)) / 100; st85 = (o["st"] & (v >= 85)).sum() / 100
  ent = len(entries(o))
  short = [nm for nm, x, need in (("steady curve 45-80", c.sum() / 100, 60), ("straight segs 65+", seg65, 120),
                                  ("straight 85+", st85, 60), ("entries", ent, 20)) if x < need]
  row("LT-00", "enough hands-off data to judge Phases 82-84",
      f"ho>=45 {(ho & (v >= 45)).sum() / 100:.0f} s (45-60 {b[0]:.0f}, 60-80 {b[1]:.0f}, 85+ {b[3]:.0f}); steady curve 45-80 "
      f"{c.sum() / 100:.1f} s / {ne} ep; straight 65+ in >=15 s segs {seg65:.0f} s; straight 85+ {st85:.0f} s; entries {ent}",
      "PASS" if not short else "CHECK", ("short: " + ", ".join(short)) if short else "")


# ---------------------------------------------------------------- lane terms (Phases 78, 82, 83, 84)
def straight_segs(o, lo, hi):
  b = o["st"] & (o["v"] >= lo) & (o["v"] < hi)
  return [(a, e) for a, e in runs(b | (box(b, 100) > 0.7)) if e - a >= 1500]


def seg_stats(o, a, e):
  """detrended sd (cm), 6-12 s band rms (cm), wheel 3 s high-pass sd (deg) of one straight segment."""
  x = o["pos"][a:e]; x = x - np.polyval(np.polyfit(np.arange(e - a), x, 1), np.arange(e - a))
  y = band(x, 6, 12)[300:-300] if e - a > 1200 else np.zeros(1)
  z = o["d"]["ang"][a:e]
  return x.std(), np.sqrt((y ** 2).mean()), (z - box(z, 300)).std()


def entries(o):
  """Curve entries: |k_model 0.3 s| < 0.3e-3 for 1.5 s, then >= 1.2e-3 within 4 s, >= 45 km/h, hands-off -1.5..+3 s."""
  if "ent" in o:
    return o["ent"]
  ak = np.abs(o["kms"]); n = o["n"]; out = []
  for a, e in runs(ak < 0.3e-3):
    if e - a >= 150 and e + 400 < n and ak[e:e + 400].max() >= 1.2e-3 and o["v"][e] >= 45 and o["ho"][e - 150:e + 300].all():
      out.append(e)
  o["ent"] = out
  return out


def lane_bits(o):
  d, T, v = o["d"], o["T"], o["v"]; b14, b15 = o["b14"], o["b15"]; hb = o["ho"] & (v >= 45)
  if not o["new_bits"]:
    row("LT-01", "record bits 14 / 15 only where designed", "bits 14 / 15 never set (older build, or the lane fit is dead)", "NO DATA")
    row("LT-09", "avoidance release: how often, in which context", "bit 15 never set", "NO DATA")
    return
  a = (b14 & (v < 44)).sum() / 100; b = (b14 & ~bit(o["sft"], 11)).sum() / 100; cm = b15 & o["lc_blocked"]; c = cm.sum() / 100
  eng = np.flatnonzero(o["lat"][1:] & ~o["lat"][:-1]) + 1; lce = np.flatnonzero((d["lcs"][1:] == 0) & (d["lcs"][:-1] > 0)) + 1
  we = np.zeros(o["n"], bool); wl = np.zeros(o["n"], bool)
  for i in eng:
    we[i:i + 295] = True
  for i in lce:
    wl[i:i + 295] = True
  # after an engage the weight starts at 1: any bit 15 in the 3 s warm-up is wrong; after a lane change only a NEW release is
  dm = (b15 & we) | (o["b15_new"] & wl); dd = dm.sum() / 100; carry = (b15 & wl & ~o["b15_new"]).sum() / 100
  e = b14[hb].mean() * 100 if hb.any() else float("nan")
  ok = a <= 1.0 and b == 0 and c == 0 and dd == 0 and 40 <= e <= 80
  row("LT-01", "record bits 14 / 15 only where designed",
      f"(a) bit14 < 44 km/h {a:.2f} s, (b) bit14 w/o direct path {b:.2f} s, (c) bit15 with press / blinker (>= 4 frames) {c:.2f} s, "
      f"(d) bit15 < 2.95 s after engage, or newly set < 2.95 s after a lane change {dd:.2f} s (carried over a lane change "
      f"{carry:.2f} s), (e) bit14 share of ho>=45 {e:.1f} %",
      vd(ok, fail=b > 0 or c > 0 or dd > 0 or e == 0), ts(T[np.flatnonzero(dm | cm)[::100]]))
  # LT-09: episodes of bit 15 and what was going on when each started
  eps = runs(b15); dk = np.abs(o["kms"]); n = o["n"]; cnt = Counter(); dur = Counter()
  for s, t in eps:
    grow = dk[min(s + 150, n - 1)] - dk[max(s - 100, 0)]
    ctx = ("after_lane_change" if (d["lcs"][max(s - 800, 0):s] > 0).any() or o["bl"][max(s - 500, 0):s].any() else
           "after_press" if o["pr"][max(s - 500, 0):s].any() else "curve_entry" if grow > 0.3e-3 else
           "in_curve" if dk[s] >= 0.8e-3 else "curve_exit" if grow < -0.3e-3 else "straight")
    cnt[ctx] += 1; dur[ctx] += (t - s) / 100
  hrs = max((o["rel"] & (v >= 30)).sum() / 360000, 1e-6); sh = b15[hb].mean() * 100 if hb.any() else float("nan")
  long_ = sorted(eps, key=lambda x: x[0] - x[1])
  row("LT-09", "avoidance release: how often, in which context (video: LT-10)",
      f"bit15 {sh:.1f} % of ho>=45; {len(eps)} episodes ({len(eps) / hrs:.0f} / h), median {med([t - s for s, t in eps]) / 100:.1f} s, "
      f">= 5.9 s (deep release likely) {sum(1 for s, t in eps if t - s >= 590)}; context n / s: "
      + ", ".join(f"{k} {cnt[k]} / {dur[k]:.0f}" for k in sorted(cnt)),
      vd(sh <= 15 and cnt["curve_entry"] <= 0.5 * len(eps), fail=False),
      "longest " + ts([T[s] for s, t in long_], 6))


def steady_curves(o):
  T, v = o["T"], o["v"]; pos, ks = o["pos"], o["ks"]
  st45 = o["st"] & (v >= 45); ref_all = pos[st45].mean() if st45.sum() > 100 else 0.0; res = {}
  for lo, hi in ((45, 60), (60, 80), (80, 300), (45, 80)):
    b = (v >= lo) & (v < hi); s = o["st"] & b; c = o["cv"] & b
    ref = pos[s].mean() if s.sum() > 300 else ref_all
    ins = (pos[c] - ref) * np.sign(ks[c])
    L = c & (ks < 0); R = c & (ks > 0)
    res[lo, hi] = (ins.mean() if c.sum() > 50 else np.nan, boot_se(ins, o["blk"][c]) if c.sum() > 50 else np.nan, c.sum() / 100,
                   -(pos[L] - ref).mean() if L.sum() > 50 else np.nan, (pos[R] - ref).mean() if R.sum() > 50 else np.nan, ref)
  m, se, sec, L, R, ref = res[45, 80]; m6 = res[60, 80][0]
  c = o["cv"] & (v >= 45) & (v < 80)
  worst = sorted(((((pos[a:e] - ref) * np.sign(ks[a:e])).mean(), T[a]) for a, e in runs(c) if e - a >= 100), reverse=True)
  enough = sec >= 30 and se <= 3.0
  ok = m <= 8.5 and not m6 > 12 and not abs(L - R) > 10
  row("LT-03", "steady curve >= 45 km/h: cm inside of the straight-line position (+ = inside)",
      f"45-80 {m:+.1f} +- {se:.1f} ({sec:.0f} s; L {L:+.1f} / R {R:+.1f}); 45-60 {res[45, 60][0]:+.1f} ({res[45, 60][2]:.0f} s), "
      f"60-80 {m6:+.1f} ({res[60, 80][2]:.0f} s), 80+ {res[80, 300][0]:+.1f} ({res[80, 300][2]:.0f} s)",
      "NO DATA" if not np.isfinite(m) else vd(ok, fail=m >= 14, enough=enough), "most inside " + ts([t for _, t in worst]))
  # LT-04: what the lane terms add toward the outside of the curve
  add = o["kc"] - o["km"]; f = lambda c: (-(add[c]) * np.sign(o["km"][c])).mean() * 1e3 if c.sum() > 50 else np.nan  # noqa: E731
  c45 = o["cv"] & (v >= 45); out = f(c45); o6 = f(o["cv"] & (v >= 60) & (v < 80))
  cap = 0.4 / np.maximum(v / 3.6, 1.0) ** 2; hb = o["ho"] & (v >= 45); ratio = np.abs(add) / cap; over = hb & (ratio > 1.0)
  row("LT-04", "steady curve: outward curvature added to the model request (1e-3 1/m)",
      f">= 45 {out:+.3f} ({c45.sum() / 100:.0f} s); 45-60 {f(o['cv'] & (v >= 45) & (v < 60)):+.3f}, 60-80 {o6:+.3f}, 80+ "
      f"{f(o['cv'] & (v >= 80)):+.3f}; |add| / (0.4 / v^2) on ho>=45 p95 {pct(ratio[hb], 95):.2f}, above 1.0: {over.sum() / 100:.2f} s",
      "NO DATA" if not np.isfinite(out) else vd(out >= 0.07 and not o6 < 0.08, fail=False, enough=c45.sum() >= 3000),
      ts(T[np.flatnonzero(over)[::100]]))


def shift_ms(ref, x, lo=-30, hi=100):
  """Time shift (ms, 100 Hz series) of x after ref that maximises their normalised correlation, with a parabolic sub-sample
  step - a scale factor or an offset on x does not move it. Returns (shift, correlation at it, True if it sits on a search
  limit). ref / x must carry -lo frames before and hi frames after the compared span."""
  n = len(ref) - (hi - lo); a = ref[-lo:-lo + n]; aa = (a * a).sum(); c = np.zeros(hi - lo + 1)
  for k in range(lo, hi + 1):
    y = x[k - lo:k - lo + n]; den = np.sqrt(aa * (y * y).sum()); c[k - lo] = (a * y).sum() / den if den > 0 else 0.0
  j = int(np.argmax(c)); f = 0.0
  if 0 < j < len(c) - 1:
    d = c[j - 1] - 2 * c[j] + c[j + 1]; f = 0.5 * (c[j - 1] - c[j + 1]) / d if d < 0 else 0.0
  return (lo + j + f) * 10.0, float(c[j]), j in (0, len(c) - 1)


def own_cross(y, e, j, fr=0.2):
  """ms after frame e at which y (sign-corrected) passes fr of ITS OWN rise: pre-entry level (mean of e-1.5 .. e-0.5 s) to its
  own peak before j + 1 s. Returns (ms, pre-entry level, rise)."""
  base = y[e - 150:e - 50].mean(); seg = y[e - 100:j + 100] - base; pk = seg.max(); i = np.flatnonzero(seg >= fr * pk)
  if pk <= 0 or not len(i) or i[0] == 0:
    return np.nan, base, pk
  i = i[0]
  return (i - 1 + (fr * pk - seg[i - 1]) / (seg[i] - seg[i - 1]) - 100) * 10.0, base, pk


def entry_timing(km, kc, kw, e, j, s):
  """One curve entry (e = entry frame, j = frame of the model's peak, s = its sign); km / kc / kw = model, command and wheel
  curvature, all three smoothed with the SAME filter. Delay = shift_ms of the rate of change over 1 s before the entry .. the
  model's peak (insensitive to amplitude: a command that is smaller early because the lane terms push outward is not 'late').
  The shift is carried by the texture the three signals share (the rate minus its 1 s mean is 0.74 of the rate's sd in logged
  entries); on an ideal textureless ramp an additive term that starts with the entry cannot be told from a delay.
  Returns (wheel delay, command delay, wheel corr, command corr, own-level 20 % wheel - model, own-level 20 % command - model,
  early amplitude command / model, early wheel / model, rise command / model, rise wheel / model); early = the rise 0.5 s
  after the model passes 20 % of its own rise."""
  a, b = e - 130, j + 100
  dm, dc, dw = (np.diff(x[a:b + 1]) * s for x in (km, kc, kw))
  tc, cc, _ = shift_ms(dm, dc); tw, cw, _ = shift_ms(dm, dw)
  (om, bm, pm), (oc, bc, pc), (ow, bw, pw) = (own_cross(x * s, e, j) for x in (km, kc, kw))
  i0 = e + int(round((om if np.isfinite(om) else 0.0) / 10.0)) + 50
  rm = km[i0] * s - bm
  return (tw, tc, cw, cc, ow - om, oc - om, (kc[i0] * s - bc) / rm if rm > 0 else np.nan, (kw[i0] * s - bw) / rm if rm > 0 else np.nan,
          pc / pm if pm > 0 else np.nan, pw / pm if pm > 0 else np.nan)


# LT-05 baselines (11-28, hands-off entries >= 45 km/h): Phase 79 a8 + a9 n 16, Phase 81 aa + ac + ad n 16, Phases 80-84
# b0-b6 n 76 -> wheel delay median +265 / +274 / +267 ms (IQR 245..292, bootstrap se of the median 7), command delay
# +29 / +29 / +28 ms (IQR 26..32), early amplitude command / model 0.93 / 0.94 / 0.84
LT05_WHEEL_OK, LT05_WHEEL_FAIL, LT05_CMD_OK, LT05_CMD_FAIL, LT05_AMP_CHECK, LT05_N_FAIL = 300.0, 330.0, 40.0, 55.0, 0.70, 8


def curve_entries(o):
  T, v = o["T"], o["v"]; ak = np.abs(o["kms"]); rows_ = []
  km, kc, kw = box(o["km"], 20), box(o["kc"], 20), box(o["kme"], 20)

  def t20(x, e, s, pk, fr=0.2):
    i = np.flatnonzero(x[e - 150:e + 400] * s >= fr * pk)
    return (i[0] - 150) / 100 if len(i) else np.nan
  for e in entries(o):
    j = e + int(np.argmax(ak[e:e + 400])); s = np.sign(o["kms"][j]); pk = ak[j]
    if e < 180 or j + 250 > o["n"]:
      continue
    tm = t20(o["km"], e, s, pk)       # only places the position window (LT-06), as before
    i0 = e + int((tm if np.isfinite(tm) else 0) * 100)
    ok = o["fok"][i0 - 100:i0 + 300].mean() > 0.8
    ins = (o["pos"][i0:i0 + 300] - o["pos"][i0 - 100:i0].mean()) * s
    tim = entry_timing(km, kc, kw, e, j, s)
    rows_.append((T[e], v[e], tim[0], tim[1], ins.max() if ok else np.nan, ins[150:250].mean() if ok else np.nan,
                  o["b15"][e - 50:e + 300].mean(), bit(o["sft"][e - 50:e + 300], 13).mean(), *tim[2:]))
  E = np.array(rows_) if rows_ else np.zeros((0, 16)); o["E"] = E; n = len(E)
  if not n:
    row("LT-05", "curve entry: wheel / command delay after the model (ms)", "no hands-off entry", "NO DATA")
    row("LT-06", "curve entry: most inside in 0-3 s (cm)", "no hands-off entry", "NO DATA")
    return
  q = E[(E[:, 1] >= 60) & (E[:, 1] < 80)]
  w, c, w6, amp = med(E[:, 2]), med(E[:, 3]), med(q[:, 2]), med(E[:, 12])
  ok = w <= LT05_WHEEL_OK and not w6 > LT05_WHEEL_OK and c <= LT05_CMD_OK
  row("LT-05", "curve entry: delay of the wheel and of the command after the model (ms; time shift of the rate of change, "
      "amplitude-insensitive) and, separately, the amplitude ratio",
      f"n {n}: wheel delay median {w:+.0f} (IQR {pct(E[:, 2], 25):+.0f}..{pct(E[:, 2], 75):+.0f}); 60-80 km/h (n {len(q)}) {w6:+.0f}; "
      f"command delay {c:+.0f} (IQR {pct(E[:, 3], 25):+.0f}..{pct(E[:, 3], 75):+.0f}); correlation at the shift wheel "
      f"{med(E[:, 8]):.2f} / command {med(E[:, 9]):.2f} | amplitude (not a delay): 0.5 s into the entry command / model "
      f"{amp:.2f} (60-80 {med(q[:, 12]):.2f}), wheel / model {med(E[:, 13]):.2f}; whole rise command / model {med(E[:, 14]):.2f}, "
      f"wheel / model {med(E[:, 15]):.2f} | own-level 20 % crossing after the model's (shape-sensitive, not judged): wheel "
      f"{med(E[:, 10]):+.0f}, command {med(E[:, 11]):+.0f}",
      vd(ok, fail=n >= LT05_N_FAIL and (w > LT05_WHEEL_FAIL or c > LT05_CMD_FAIL), enough=n >= 15 and not amp < LT05_AMP_CHECK),
      "latest " + ts(E[np.argsort(-np.nan_to_num(E[:, 2]))][:, 0]))
  m, p9, m6 = med(E[:, 4]), pct(E[:, 4], 90), med(q[:, 4]); rl = E[E[:, 6] > 0]
  row("LT-06", "curve entry: most inside in 0-3 s, and mean of 1.5-2.5 s (cm)",
      f"n {int(np.isfinite(E[:, 4]).sum())}: max inside median {m:+.1f}, p90 {p9:+.1f}; 60-80 {m6:+.1f}; 1.5-2.5 s {med(E[:, 5]):+.1f}; "
      f"entries with bit 15: {len(rl)} (max inside {med(rl[:, 4]):+.1f} vs {med(E[E[:, 6] == 0][:, 4]):+.1f} without)",
      "NO DATA" if not np.isfinite(m) else vd(m <= 16 and p9 <= 35 and not m6 > 18, fail=m >= 24, enough=n >= 15),
      "deepest " + ts(E[np.argsort(-np.nan_to_num(E[:, 4]))][:, 0]))


def curve_exits(o):
  T, v = o["T"], o["v"]; ak = np.abs(o["kms"]); n = o["n"]; X = []
  for a, e in runs(ak < 0.3e-3):
    if a > 400 and e - a >= 200 and a + 500 < n and ak[a - 400:a].max() >= 1.2e-3 and v[a] >= 45 and o["ho"][a - 200:a + 400].all():
      if o["fok"][a - 100:a + 400].mean() < 0.8:
        continue
      s = np.sign(o["kms"][a - 400:a][np.argmax(ak[a - 400:a])])
      outp = -(o["pos"][a:a + 400] - o["pos"][a - 100:a].mean()) * s
      X.append((T[a], outp.max(), np.ptp(o["pos"][a:a + 400])))
  X = np.array(X) if X else np.zeros((0, 3))
  if not len(X):
    row("LT-07", "curve exit: swing to the outside in 0-4 s (cm)", "no hands-off exit", "NO DATA"); return
  m, p9, pp = med(X[:, 1]), pct(X[:, 1], 90), med(X[:, 2])
  row("LT-07", "curve exit: swing to the outside in 0-4 s (cm)", f"n {len(X)}: median {m:+.1f}, p90 {p9:+.1f}, p-p median {pp:.1f}",
      vd(m <= 7 and p9 <= 14 and pp <= 18, fail=m >= 10 or p9 >= 20, enough=len(X) >= 10), "widest " + ts(X[np.argsort(-X[:, 1])][:, 0]))


def straights(o):
  T, v, pos = o["T"], o["v"], o["pos"]; add = o["kc"] - o["km"]

  def stats(lo, hi):
    segs = straight_segs(o, lo, hi); S = [(e - a, *seg_stats(o, a, e), T[a], v[a:e].mean()) for a, e in segs]
    b = o["st"] & (v >= lo) & (v < hi); w = np.array([s[0] for s in S], float)
    f = lambda k: float(np.sqrt(np.average(np.array([s[k] for s in S]) ** 2, weights=w))) if S else np.nan  # noqa: E731
    return b, S, w.sum() / 100, f(1), f(2), f(3)
  b, S, tot, sd, br, wh = stats(65, 300)
  chg = (add[50:] - add[:-50])[b[50:] & b[:-50]]; chg = chg.std() * 1e3 if len(chg) > 100 else np.nan
  mu = pos[b].mean() if b.sum() > 100 else np.nan
  nbad = sum(1 for s in S if s[2] > 5); worst = sorted(S, key=lambda s: -s[2])
  ok = br <= 3.5 and sd <= 6 and not any(s[2] > 6 for s in S) and wh <= 0.25 and not chg > 0.08
  row("LT-08", "straight >= 65 km/h (segments >= 15 s): slow weave and wheel activity",
      f"{len(S)} segs / {tot:.0f} s: 6-12 s band {br:.1f} cm rms, detrended sd {sd:.1f} cm, wheel 3 s high-pass sd {wh:.2f} deg, "
      f"segs with band > 5 cm: {nbad}; added curvature 0.5 s change sd {chg:.3f}e-3",
      "NO DATA" if not S else vd(ok, fail=nbad >= 2, enough=tot >= 120),
      "worst " + ", ".join(f"t={s[4]:.0f} ({s[0] / 100:.0f} s, {s[5]:.0f} km/h, {s[2]:.1f} cm)" for s in worst[:3]))
  # LT-16: mean position on straights (the integrator itself needs the replay)
  out = []
  for lo, hi in ((45, 65), (65, 300)):
    m = o["st"] & (v >= lo) & (v < hi)
    out.append((pos[m].mean() if m.sum() > 100 else np.nan, boot_se(pos[m], o["blk"][m]) if m.sum() > 100 else np.nan, m.sum() / 100,
                add[m].mean() * 1e3 if m.sum() > 100 else np.nan))
  bias = (o["d"]["ang"] - o["d"]["mdps_ang"])[o["st"] & (v >= 45)].mean() if "mdps_ang" in o["d"] and (o["st"] & (v >= 45)).any() else np.nan
  row("LT-16", "straight: mean position (cm, + = right) and mean added curvature",
      f"45-65 {out[0][0]:+.1f} +- {out[0][1]:.1f} ({out[0][2]:.0f} s, add {out[0][3]:+.3f}e-3); 65+ {out[1][0]:+.1f} +- {out[1][1]:.1f} "
      f"({out[1][2]:.0f} s, add {out[1][3]:+.3f}e-3); wheel sensor - MDPS angle {bias:+.2f} deg",
      "NO DATA" if not np.isfinite(out[0][0]) else vd(all(not abs(x[0]) > 5 for x in out), fail=any(abs(x[0]) > 8 for x in out),
                                                     enough=min(x[2] for x in out) >= 60))
  # LT-11 / LT-15: excursions on straights, recovery time with and without bit 15
  s45 = o["st"] & (v >= 45); mu = o["smean"]; ex30 = s45 & (np.abs(pos - mu) > 30)
  ex50 = [(a, e) for a, e in runs(s45 & (np.abs(pos - mu) > 50) & o["b15"] & ~o["pr"]) if e - a >= 30]
  ps = box(pos, 50); rec = []
  for a, e in runs(s45 & (np.abs(ps - mu) > 20)):
    j = e
    while j < min(e + 1500, o["n"] - 1) and abs(ps[j] - mu) >= 10:
      j += 10
    if o["ho"][a:j].all() and (np.abs(o["kms"][a:j]) < 0.5e-3).all():
      rec.append((T[a], (j - a) / 100, bool(o["b15"][a:j].any()), v[a]))
  r1 = [x[1] for x in rec if x[2]]; r0 = [x[1] for x in rec if not x[2]]; sh = ex30.sum() / max(s45.sum(), 1) * 100
  slow = np.isfinite(med(r1)) and np.isfinite(med(r0)) and med(r1) > max(1.5 * med(r0), med(r0) + 1.0)
  row("LT-11", "straight: pushed > 30 cm off - does the avoidance release slow the return (option J)",
      f"> 30 cm {ex30.sum() / 100:.1f} s of {s45.sum() / 100:.0f} s ({sh:.2f} %), bit15 on {o['b15'][ex30].mean() * 100 if ex30.any() else 0:.0f} % "
      f"of it; > 50 cm with bit15 for >= 0.3 s, no press: {len(ex50)}; 20 cm excursions back < 10 cm: with bit15 n {len(r1)} median "
      f"{med(r1):.1f} s, without n {len(r0)} median {med(r0):.1f} s",
      "NO DATA" if s45.sum() < 1000 else vd(sh <= 1.0 and not slow, fail=bool(ex50)), ts(T[[a for a, e in ex50] + [a for a, e in runs(ex30)]]))
  b, S, tot, sd, br, wh = stats(85, 300); r85 = [x[1] for x in rec if x[3] >= 85]
  ex = (b & (np.abs(pos - (pos[b].mean() if b.any() else 0)) > 30)).sum() / max(b.sum(), 1) * 100
  row("LT-15", "straight >= 85 km/h hands-off: weave and return",
      f"{b.sum() / 100:.0f} s, {len(S)} segs >= 15 s / {tot:.0f} s: band {br:.1f} cm rms, detrended sd {sd:.1f} cm, wheel hp sd {wh:.2f} deg; "
      f"> 30 cm {ex:.1f} %; 20 cm excursions n {len(r85)} median return {med(r85):.1f} s",
      "NO DATA" if b.sum() < 1000 else vd(br <= 4 and sd <= 7 and ex <= 1 and not med(r85) > 4, fail=br > 6 or (ex > 3 and b.sum() >= 6000),
                                          enough=b.sum() >= 6000 and len(S) > 0),
      ", ".join(f"t={s[4]:.0f} ({s[2]:.1f} cm)" for s in sorted(S, key=lambda s: -s[2])[:3]))


def reengage(o):
  d, T, v = o["d"], o["T"], o["v"]; lat = o["lat"]; R = []
  for i in np.flatnonzero(lat[1:] & ~lat[:-1]) + 1:
    if i < 100 or i + 300 >= o["n"] or v[i] < 45:
      continue
    off = 0
    while off < 6000 and i - 1 - off >= 0 and not lat[i - 1 - off]:
      off += 1
    if not 50 <= off <= 500:
      continue
    R.append((T[i], v[i], off / 100, float(o["b14"][i]), o["b15"][i:i + 295].sum() / 100, abs(o["kc"][i] - o["kme"][i]) * 1e3,
              np.abs(d["ang"][i:i + 50] - d["ang"][i]).max(), np.abs(d["rate"][i:i + 50]).max(), float(o["pr"][i:i + 50].any()),
              float(abs(o["kms"][i]) < 0.5e-3)))
  R = np.array(R) if R else np.zeros((0, 10))
  if not len(R):
    row("LT-12", "steering switched off 0.5-5 s and on again >= 45 km/h: first 0.5 s", "no such event (the owner has to do it)", "NO DATA")
    return
  stq = R[(R[:, 9] > 0) & (R[:, 8] == 0)]
  hard = R[:, 3].sum() == 0 and R[:, 4].sum() == 0
  soft = not len(stq) or (stq[:, 6].max() <= 2.0 and stq[:, 7].max() <= 10)
  row("LT-12", "steering switched off 0.5-5 s and on again >= 45 km/h: first 0.5 s",
      f"n {len(R)}: bit14 on first frame {int(R[:, 3].sum())}, bit15 within 2.95 s {R[:, 4].sum():.2f} s, first-frame |k_ctrl - k_meas| median "
      f"{med(R[:, 5]):.3f}e-3 (not judged); straight, no press (n {len(stq)}): wheel move 0.5 s max {stq[:, 6].max() if len(stq) else np.nan:.1f} deg, "
      f"peak rate {stq[:, 7].max() if len(stq) else np.nan:.0f} deg/s", vd(hard and soft, fail=not hard, enough=len(R) >= 3 and o["new_bits"]), ts(R[:, 0], 8))


def entry_lead(o):
  v = o["v"]; b13 = bit(o["sft"], 13); hb = o["ho"] & (v >= 45); m = hb & b13
  if not m.any():
    row("LT-14", "entry lead (bit 13) frames: net added curvature along the curve (1e-3 1/m)", "bit 13 never set", "NO DATA"); return
  net = ((o["kc"] - o["km"]) * np.sign(o["kms"]))[m]; x = net.mean() * 1e3; E = o.get("E", np.zeros((0, 8)))
  sh2 = (b13 & o["op"] & o["hoc"]).sum() / max((o["op"] & o["hoc"]).sum(), 1) * 100
  row("LT-14", "entry lead (bit 13) frames: net added curvature along the curve (1e-3 1/m); G-1 share",
      f"bit13 {m.sum() / hb.sum() * 100:.1f} % of ho>=45 ({sh2:.1f} % of op hands-off); net add {x:+.3f} (same direction {np.mean(net > 0) * 100:.0f} %); "
      f"bit13 share of the entry windows {E[:, 7].mean() if len(E) else np.nan:.2f}", vd(x >= -0.13, fail=x < -0.15 and "LT-05" in failed()))


def failed():
  return {r[0] for r in ROWS if r[3] == "FAIL"}


def driver_presses(o):
  d, T, v = o["d"], o["T"], o["v"]; m = o["rel"] & (v >= 45); pr = o["pr"]
  ed = np.flatnonzero(pr[1:] & ~pr[:-1] & m[:-1]) + 1; mins = max(m.sum() / 6000, 1e-6); outw = inw = 0; tt = []
  for i in ed:
    if abs(o["ks"][i]) >= 1e-3 and not o["bl"][max(i - 300, 0):i + 100].any():
      j = i
      while j < o["n"] - 1 and pr[j]:
        j += 1
      pk = d["tq_s"][i:j + 1][np.argmax(np.abs(d["tq_s"][i:j + 1]))]
      if np.sign(pk) * np.sign(o["ks"][i]) > 0:
        outw += 1; tt.append(T[i])     # torque to the left in a right curve: correcting toward the outside
      else:
        inw += 1
  row("LT-17", "driver presses per minute >= 45 km/h; in curves, corrected to the outside / inside (+ owner impression)",
      f"{len(ed) / mins:.2f} / min ({len(ed)} in {mins:.0f} min); in curves without blinker: outward {outw}, inward {inw}",
      vd(len(ed) / mins <= 3.1, fail=False), "outward " + ts(tt, 6))


# ---------------------------------------------------------------- blinker and lane change (Phase 80)
def lane_changes(o):
  """One dict per lcs == 2 episode that starts above 30 km/h (sg = +1 left / -1 right: sign of angle / torque toward it)."""
  d, T, N = o["d"], o["T"], o["n"]; pr = o["pr"]; tq = d["tq_s"]; tl, tr = o["tl"], o["tr"]; sfc, sft = o["sfc"], o["sft"]; rows_ = []
  for a0, b0 in runs(d["lcs"] == 2):
    if d["v"][a0] < 30 or b0 >= N - 1:
      continue
    right = tr[max(a0 - 50, 0):b0].sum() >= tl[max(a0 - 50, 0):b0].sum()
    ld = d["lc_dir"][a0:b0] if "lc_dir" in d else np.zeros(1, int)
    if (ld > 0).any():
      right = (ld == 2).sum() >= (ld == 1).sum()
    sg = -1 if right else 1; tm = tr if right else tl
    if not tm[max(a0 - 300, 0):b0].any():
      tm = tl | tr      # no blinker of that side near the change: take any, never search back through the whole drive
    on = a0
    while on > 0 and not tm[on]:
      on -= 1
    while on > 0 and tm[on - 1]:
      on -= 1
    off = a0
    while off < N - 1 and not tm[off]:
      off += 1
    while off < N - 1 and tm[off]:
      off += 1
    f = o["fr"] if right else -o["fl"]; near = -o["fl"] if right else o["fr"]; q = o["qr"] if right else o["ql"]
    w0 = max(min(on, a0) - 100, 0); df = f[w0 + 20:b0] - f[w0:b0 - 20]
    cr = int(np.argmax(df > 1.0)) + 20 + w0 if (df > 1.0).any() else None
    r = dict(t=T[a0], i=(a0, b0), v=d["v"][a0], dir="R" if right else "L", sg=sg, on=T[on], off=T[off], end=T[b0],
             lcs_after_off=T[b0] - T[off], cross=T[cr] if cr else np.nan, on2cross=T[cr] - T[on] if cr else np.nan)
    pw = slice(max(on - 100, 0), max(cr if cr else b0, on + 50)); r["push_pk"] = np.abs(tq[pw]).max()
    p = np.flatnonzero(pr[max(on - 300, 0):on + 300])
    r["press_before_on"] = T[on] - T[p[0] + max(on - 300, 0)] if len(p) else np.nan
    r["press_lcs2"] = pr[a0:b0].mean(); r["gain_press"] = med(d["gain"][a0:b0][pr[a0:b0]])
    pin = pr[a0:b0] & (np.abs(d["apply"][a0:b0] - d["ang"][a0:b0]) > 2)
    r["pinned"] = (np.abs(d["wire"][a0:b0] - d["ang"][a0:b0]) < 0.5)[pin].mean() if pin.any() else np.nan
    # release: first 0.3 s without press and below 260 Nm
    rl = first_run(~pr[a0:b0 + 300] & (np.abs(tq[a0:b0 + 300]) < 260), 30)
    if rl is not None and a0 + rl < b0:
      rl += a0; m = np.zeros(N, bool); m[rl:b0] = True; m &= ~pr
      den = d["apply"] - d["ang"]; okd = m & (np.abs(den) > 0.5)
      r.update(rel=T[rl], ho_s=m.sum() / 100, gain_ho=med(d["gain"][m]), conc_ho=bit(sfc[m], 20).mean() if m.any() else np.nan,
               auth_ho=med((d["wire"][okd] - d["ang"][okd]) / den[okd]) if okd.sum() > 5 else np.nan)
      stuck = m & (np.abs(d["wire"] - d["ang"]) < 0.3) & (np.abs(den) > 2); r["stuck_s"] = max([e - a for a, e in runs(stuck)] + [0]) / 100
      g = first_run(d["gain"][rl:min(rl + 1500, N)] >= 0.9, 20)
      r["rel2gain09"] = g / 100 if g is not None else np.nan; r["off2gain09"] = T[rl + g] - T[off] if g is not None else np.nan
      r["rate_rel"] = np.abs(d["rate"][rl:rl + 100]).max()
      # re-grips after the release, and concession re-entries without a press (option L)
      seg = slice(rl + 30, min(b0 + 300, N)); pe = np.flatnonzero(~pr[seg][:-1] & pr[seg][1:]); r["regrip_n"] = len(pe)
      if len(pe):
        i = seg.start + pe[0] + 1; j = i
        while j < N - 1 and pr[j]:
          j += 1
        r["regrip_t"] = T[i]; r["regrip_tq"] = tq[i:j + 1][np.argmax(np.abs(tq[i:j + 1]))] * sg     # - = pulling back
        y = first_run(bit(sfc[i:j + 50], 20) | (np.abs(d["wire"][i:j + 50] - d["ang"][i:j + 50]) < 0.5), 1)
        r["regrip_yield"] = y / 100 if y is not None else np.nan
        g2 = first_run(d["gain"][j:min(j + 1500, N)] >= 0.9, 20); r["regrip2gain09"] = g2 / 100 if g2 is not None else np.nan
      c20 = bit(sfc, 20); re = [k for k in np.flatnonzero(~c20[rl + 30:b0 - 1] & c20[rl + 31:b0]) + rl + 31 if not pr[k]]
      r["reentry_n"] = len(re); r["reentry_spike"] = sum(1 for k in re if (np.abs(tq[k - 3:k + 1]) >= 260).sum() < 3)
      r["reentry_s"] = sum(next((e - a for a, e in runs(c20[k:b0]) if a == 0), 0) for k in re) / 100
    # after the state ends: blinker still on and hands off (option K)
    kk = np.zeros(N, bool); kk[b0:off] = True; kk &= ~pr
    r["K_conc_s"] = (kk & bit(sfc, 20)).sum() / 100; r["K_gain_min"] = d["gain"][kk].min() if kk.any() else np.nan
    tail = np.zeros(N, bool); tail[min(off, b0):b0] = True; tail &= ~pr
    r["tail_gain"] = med(d["gain"][tail]); r["tail_conc"] = bit(sfc[tail], 20).mean() if tail.any() else np.nan
    k = off
    while k < N - 1 and bit(sfc[k:k + 1], 20)[0]:
      k += 1
    r["conc_after_off"] = T[k] - T[off]
    w = slice(a0, min(b0 + 100, N)); r["blend_s"] = bit(sft[w], 4).sum() / 100; r["drop_s"] = bit(sft[w], 5).sum() / 100
    r["cap_post1s"] = bit(sft[b0:min(b0 + 100, N)], 1).sum() / 100
    r["outrun_s"] = bit(sfc[a0:b0], 25).sum() / 100; r["reject"] = int(d["tx_reject"][a0:b0].sum())
    cbk = (d["rb"] if right else d["lb"]).astype(bool)[a0:b0]     # the blinker as the car controller saw it
    bs = (d["bsr"] if right else d["bsl"]).astype(bool)[a0:b0] & cbk; r["bsm_n"] = int(bs.sum())
    r["bsm_conc"] = bit(sfc[a0:b0], 20)[bs].mean() if bs.any() else np.nan
    pb = pr[a0:b0] & cbk; r["press_conc"] = bit(sfc[a0:b0], 20)[pb].mean() if pb.any() else np.nan
    if cr:   # how far toward the far line of the new lane after the crossing
      e2 = cr + 60; lim = min(cr + 500, N - 21)
      while e2 < lim and not f[e2 + 20] - f[e2] > 1.0:
        e2 += 1
      w2 = slice(cr + 20, e2); okf = (q[w2] >= 2) & (f[w2] < 2.0); both = okf & (near[w2] < 2.0); ps = (near[w2] - f[w2]) / 2
      # clean = one crossing only, the far line seen for >= 1 s, and never under the car (that is a merge / relabelled line)
      r["clean"] = bool(e2 >= lim and okf.sum() >= 100 and f[w2][okf].min() >= 0.3)
      r["far_min"] = f[w2][okf].min() - HALF_W if okf.any() else np.nan; r["pos_max"] = ps[both].max() if both.any() else np.nan
      s = first_run(both & (np.abs(ps) < 0.25), 100); r["settle"] = s / 100 + 0.2 if s is not None else np.nan
      r["saved"] = T[b0] - max(T[cr], T[off])
    rows_.append(r)
  return rows_


def col(L, k):
  return np.array([r.get(k, np.nan) for r in L], float)


def blinker(o):
  d, T = o["d"], o["T"]; lb, rb = d["lb"].astype(bool), d["rb"].astype(bool)
  if not o["raw_blink"]:
    for i, w in (("LCB-01", "carState blinker = raw 0x400 state"), ("LCB-02", "blinker-off lag carState vs 0x400 state"),
                 ("LCB-03", "0x400 state with hazards on")):
      row(i, w, f"needs a v8 npz (raw 0x400 / 0x3E3); carState left + right together {(lb & rb).sum() / 100:.1f} s", "NO DATA")
    return
  s = d["blink_state"]; tl, tr = o["tl"], o["tr"]; hz = s == 7; nrun = len(runs(tl | tr))
  mm = [(lb & ~tl).sum(), (tl & ~lb).sum(), (rb & ~tr).sum(), (tr & ~rb).sum()]; both = (lb & rb & ~hz).sum() / 100
  wrong = 0     # a blinker run whose own lamp never lit or whose opposite lamp lit
  for a, e in runs((tl | tr) & ~hz):
    own, oth = (d["lamp_l"], d["lamp_r"]) if tl[a] else (d["lamp_r"], d["lamp_l"])
    wrong += int(e - a >= 100 and (not (own[a:e + 60] == 1).any() or (oth[a + 60:e] == 1).any()))
  tot = sum(mm) / 100; ok = tot <= 0.05 * 2 * max(nrun, 1) and both == 0 and wrong == 0
  bad = np.flatnonzero((lb != tl) | (rb != tr))
  row("LCB-01", "carState blinker = raw 0x400 state (no phantom left, no both, none missed)",
      f"{nrun} blinker runs; CS-L not state-L {mm[0] / 100:.2f} s, state-L not CS-L {mm[1] / 100:.2f} s, CS-R not state-R {mm[2] / 100:.2f} s, "
      f"state-R not CS-R {mm[3] / 100:.2f} s; both on (no hazard) {both:.2f} s; runs with the wrong lamp side {wrong}; "
      f"state values seen {sorted(set(np.unique(s).tolist()) - {-1})}", vd(ok), ts(T[bad[::200]]))
  lag = []
  tb = tl | tr; cb = lb | rb
  for i in np.flatnonzero(tb[:-1] & ~tb[1:]) + 1:
    k = i
    while k < o["n"] - 1 and cb[k] and not tb[k]:
      k += 1
    if not tb[k]:
      lag.append((T[k] - T[i], T[i]))
  L = np.array(lag) if lag else np.zeros((0, 2))
  row("LCB-02", "blinker-off lag carState vs 0x400 state (was 0.5 s)", f"n {len(L)}: median {med(L[:, 0]):.2f} s, max {L[:, 0].max() if len(L) else np.nan:.2f} s",
      "NO DATA" if not len(L) else vd(L[:, 0].max() <= 0.03), ts(L[L[:, 0] > 0.03][:, 1]))
  hl = held(box(d["lamp_l"] == 1, 100) > 0, 1) & (box(d["lamp_r"] == 1, 100) > 0)     # both lamps flashing within the same second
  if not hl.any() and not hz.any():
    row("LCB-03", "0x400 state with hazards on", "no interval with both lamps (hazard button not pressed)", "NO DATA"); return
  ivals = [(a, e) for a, e in runs(hl) if e - a >= 200]; keep = np.zeros(o["n"], bool)
  for a, e in ivals:
    keep[a:e] = True
  if not ivals and not hz.any():
    row("LCB-03", "0x400 state with hazards on", "no interval >= 2 s with both lamps (hazard button not pressed)", "NO DATA"); return
  hl = keep | hz; hist = Counter(s[hl].tolist())
  cs_both = (lb & rb)[hl].mean(); lcs_on = (d["lcs"][hl] > 0).sum() / 100
  ok = set(hist) <= {7, 0} and hist.get(7, 0) >= 0.8 * hl.sum() and cs_both >= 0.8 and lcs_on == 0
  row("LCB-03", "0x400 state with hazards on (7 assumed, never logged)",
      f"{len(ivals)} intervals / {hl.sum() / 100:.1f} s; state histogram {dict(sorted(hist.items()))}; carState left + right "
      f"{cs_both * 100:.0f} %; lane-change state on {lcs_on:.2f} s; concession bit {bit(o['sfc'][hl], 20).mean() * 100:.0f} %", vd(ok),
      ts([T[a] for a, e in ivals]))


def lane_change_items(o):
  d, T, v = o["d"], o["T"], o["v"]; L = lane_changes(o); o["LC"] = L; n = len(L); tt = lambda m: ts([r["t"] for r, x in zip(L, m, strict=True) if x], 6)  # noqa: E731
  ids = ("LCB-04", "LCB-05", "LCB-06", "LCB-07", "LCB-08", "LCB-09", "LCB-10", "LCB-11", "LCB-12", "LCB-13", "LCB-15", "LCB-16", "LCB-17", "LT-13")
  if not n:
    for i in ids:
      row(i, "lane change", "no lane change above 30 km/h", "NO DATA")
    return
  bl = col(L, "blend_s")
  row("LCB-04", "low-confidence blend off during op's lane change + 1 s (80b)",
      f"n {n}: blend bit {med(bl):.2f} s median, max {bl.max():.2f} s, lane changes with any {int((bl > 0).sum())}; dropout hold inside "
      f"{int((col(L, 'drop_s') > 0).sum())}", vd(bl.max() == 0), tt(bl > 0))
  R = [r for r in L if "rel" in r]; g, au, cc = col(R, "gain_ho"), col(R, "auth_ho"), col(R, "conc_ho"); stuck = col(R, "stuck_s")
  r2g, o2g = col(R, "rel2gain09"), col(R, "off2gain09")
  ok = med(cc) <= 0.20 and med(g) >= 0.70 and med(au) >= 0.85 and pct(au, 25) >= 0.6 and med(r2g) <= 2.0 and med(o2g) <= 0.3
  row("LCB-05", "hands off during op's lane change: concession ends, authority back (80c)",
      f"released in {len(R)} of {n}: concession bit {med(cc):.2f}, gain median {med(g):.2f} (p25 {pct(g, 25):.2f}), authority {med(au):.2f} "
      f"(p25 {pct(au, 25):.2f}), release -> gain 0.9 {med(r2g):.2f} s, blinker off -> gain 0.9 {med(o2g):+.2f} s, wire stuck on the wheel "
      f"max {stuck.max() if len(R) else np.nan:.2f} s", "NO DATA" if not R else vd(ok, fail=med(g) <= 0.5 or stuck.max() >= 1.0, enough=len(R) >= 6),
      "weakest " + ts([r["t"] for r in sorted(R, key=lambda r: r.get("gain_ho", 9))]))
  rt = col(R, "regrip_tq"); cnt = rt < 0; y = col(R, "regrip_yield"); sh = cnt.sum() / max(len(R), 1)
  row("LCB-06", "re-grip after the release: how often against the change, does op yield at once",
      f"re-grip in {int(np.isfinite(rt).sum())} of {len(R)}, against the change {int(cnt.sum())} ({sh * 100:.0f} %), peak median "
      f"{med(np.abs(rt[cnt])):.0f} (max {np.abs(rt[cnt]).max() if cnt.any() else np.nan:.0f}); yield after the press edge max {np.nanmax(y) if np.isfinite(y).any() else np.nan:.2f} s; "
      f"press end -> gain 0.9 median {med(col(R, 'regrip2gain09')):.2f} s", "NO DATA" if not R else
      vd(sh <= 0.30 and not np.nanmax(np.r_[y, 0]) > 0.05 and not med(col(R, "regrip2gain09")) > 1.5, fail=False, enough=len(R) >= 6),
      "against " + ts([r["regrip_t"] for r, x in zip(R, cnt, strict=True) if x], 6))
  C = [r for r in L if r.get("clean")]; fm, pm, st = col(C, "far_min"), col(C, "pos_max"), col(C, "settle")
  row("LCB-07", "after the crossing: body clearance to the far line (m, - = over it), offset to the far side, settle time",
      f"clean {len(C)} of {n}: far-line clearance median {med(fm):+.2f}, worst {np.nanmin(fm) if len(C) else np.nan:+.2f}; offset max median "
      f"{med(pm):.2f} (<= 0.35 in {np.mean(pm <= 0.35) * 100 if len(C) else np.nan:.0f} %); settle median {med(st):.1f} s",
      "NO DATA" if not C else vd(np.nanmin(fm) >= 0.20 and np.mean(pm <= 0.35) >= 0.8 and not med(st) > 3.0, fail=np.nanmin(fm) < 0, enough=len(C) >= 6),
      "closest " + ts([r["t"] for r in sorted(C, key=lambda r: r["far_min"] if np.isfinite(r["far_min"]) else 9)]))
  la, tg, tc = col(L, "lcs_after_off"), col(L, "tail_gain"), col(L, "tail_conc")
  bare = []    # blinker cancelled without a lane change: time to gain 0.9
  tb = o["tl"] | o["tr"]
  for i in np.flatnonzero(tb[:-1] & ~tb[1:]) + 1:
    if v[i] > 30 and o["lat"][i] and not (d["lcs"][max(i - 300, 0):i + 1] == 2).any() and i + 400 < o["n"]:
      gq = first_run((d["gain"][i:i + 400] >= 0.9) & ~o["pr"][i:i + 400], 20)
      bare.append(gq / 100 if gq is not None else 4.0)
  row("LCB-08", "'icon stays on' tail: state outlives the blinker, is op holding in that tail",
      f"state after blinker off median {med(la):.2f} s (max {np.nanmax(la):.2f}); in the tail hands-off: gain {med(tg):.2f}, concession bit "
      f"{med(tc):.2f}; bare blinker cancel -> gain 0.9: n {len(bare)} median {med(bare):.2f} s",
      vd(med(tg) >= 0.9 and not med(tc) > 0 and not med(bare) > 1.0, fail=med(tg) < 0.6, enough=np.isfinite(tg).sum() >= 3), tt(tg < 0.9))
  # LCB-09: a lane-turn event (sunnypilot 'Turning Left / Right') inside a running lane change = the desire was replaced
  if "events_sp" in d:
    turn = [float(t) for t, nm in d["events_sp"] if str(nm).startswith("laneTurn") and any(r["t"] <= t <= r["end"] for r in L)]
    try:
      vt = float(str(d["lane_turn"]).split(",")[1]) * 1.609
    except (ValueError, IndexError):
      vt = 32.0
    cand = [r["t"] for r in L if ((d["v"][r["i"][0]:r["i"][1]] < vt) & (d["lb"] | d["rb"])[r["i"][0]:r["i"][1]]).any()]
    row("LCB-09", "a running lane change keeps its desire below the lane-turn speed (80d)",
        f"lane-turn events inside a lane change: {len(turn)}; lane changes that went below {vt:.0f} km/h with the blinker on (the case 80d "
        f"covers): {len(cand)} of {n} (params LaneTurnDesire,LaneTurnValue = '{d['lane_turn']}')",
        "FAIL" if turn else ("PASS" if cand else "NO DATA"), ts(turn) if turn else (ts(cand) if cand else "not exercised"))
  else:
    lo = (d["lcs"] == 2) & (v < 40)
    row("LCB-09", "a running lane change keeps its desire below the lane-turn speed (80d)",
        f"needs a v8 npz (lane-turn events); lcs == 2 below 40 km/h {lo.sum() / 100:.2f} s", "NO DATA", ts(T[[a for a, e in runs(lo)]]))
  kc, kg = col(L, "K_conc_s"), col(L, "K_gain_min"); rn, rs, rsp = col(R, "reentry_n"), col(R, "reentry_s"), col(R, "reentry_spike")
  okK = np.nanmax(kc) <= 0.3 and not np.nanmin(np.r_[kg, 1.0]) < 0.6
  okL = not R or (np.mean(rsp > 0) <= 0.10 and np.nanmax(np.r_[rs, 0]) <= 0.3)
  row("LCB-10", "option K (concession back after the state ends, lever still on) / L (short torque spike re-arms it)",
      f"K: concession with blinker on after the state ended, hands off: max {np.nanmax(kc):.2f} s in {int((kc > 0).sum())} of {n}, gain min "
      f"{np.nanmin(np.r_[kg, np.nan]) if np.isfinite(kg).any() else np.nan:.2f}; L: re-entries without press in {int((rn > 0).sum())} of {len(R)}, by < 3 samples "
      f">= 260 in {int((rsp > 0).sum())}, re-armed max {np.nanmax(np.r_[rs, 0]):.2f} s", vd(okK and okL, fail=False),
      tt((col(L, "K_conc_s") > 0.3) | (np.nan_to_num(col(L, "reentry_spike")) > 0)))
  cp = col(L, "cap_post1s"); far = col(L, "far_min"); hit = (cp > 0.2) & (far < 0.2)
  row("LCB-11", "option M: Phase 41 rate cap in the first second after a lane change",
      f"capped > 0.2 s in {int((cp > 0.2).sum())} of {n} (max {cp.max():.2f} s), together with a far-line clearance < 0.2 m: {int(hit.sum())}",
      vd((cp > 0.2).mean() <= 0.10 and not hit.any(), fail=hit.sum() >= 2), tt(cp > 0.2))
  light = [r for r in L if not r["press_before_on"] > 0 and r["push_pk"] < 350 and r["on2cross"] >= 3.0]
  lf, lp = col(light, "far_min"), col(light, "pos_max"); lr = np.nan_to_num(col(light, "regrip_tq")) < 0
  row("LCB-12", "blinker-first light-nudge lane changes (push < 350, blinker -> crossing >= 3 s): does the plan still overshoot",
      f"{len(light)} of {n} (blinker -> crossing median {med(col(L, 'on2cross')):.2f} s, push peak median {med(col(L, 'push_pk')):.0f}, pressed before "
      f"the blinker in {int((col(L, 'press_before_on') > 0).sum())}); light ones: clearance worst {np.nanmin(lf) if len(light) and np.isfinite(lf).any() else np.nan:+.2f} m, "
      f"offset max {np.nanmax(lp) if len(light) and np.isfinite(lp).any() else np.nan:.2f} m, re-grips against {int(lr.sum())}",
      "NO DATA" if not light else vd(not np.nanmin(np.r_[lf, 9]) < 0.25 and not np.nanmax(np.r_[lp, 0]) > 0.30 and not lr.any(), enough=len(light) >= 5),
      ts([r["t"] for r in light], 8))
  sv = col(L, "saved")
  row("LCB-13", "option D: what ending the state at max(crossing, blinker off) would save (owner decides)",
      f"median {med(sv):.2f} s, >= 1.5 s in {int((sv >= 1.5).sum())} of {int(np.isfinite(sv).sum())}", "CHECK" if np.isfinite(sv).any() else "NO DATA")
  row("LCB-15", "option H inputs: how much the driver steers the change (owner decides)",
      f"pressed {med(col(L, 'press_lcs2')) * 100:.0f} % of the state, gain while pressed {med(col(L, 'gain_press')):.2f}, request pinned to the wheel "
      f"{med(col(L, 'pinned')) * 100:.0f} % of pressed frames, push peak median {med(col(L, 'push_pk')):.0f}", "CHECK")
  pc, bc, orun, rj = col(L, "press_conc"), col(L, "bsm_conc"), col(L, "outrun_s"), col(L, "reject")
  sat = [t for t, nm in o["ev"] if nm == "steerSaturated" and any(r["t"] <= t <= r["end"] for r in L)]
  ok = not np.nanmin(np.r_[pc, 1]) < 0.95 and not np.nanmin(np.r_[bc, 1]) < 0.95 and rj.sum() == 0 and orun.max() <= 0.2 and not sat
  row("LCB-16", "Phase 80 must not change: yield while pressing, toward an occupied blind spot; no rejects / outrun",
      f"concession while pressed + blinker: min {np.nanmin(np.r_[pc, np.nan]) if np.isfinite(pc).any() else np.nan:.2f}; blind-spot side frames "
      f"{int(col(L, 'bsm_n').sum())} (concession min {np.nanmin(bc) if np.isfinite(bc).any() else np.nan:.2f}); TX rejects {int(rj.sum())}; wheel-outrun max "
      f"{orun.max():.2f} s; steerSaturated inside {len(sat)}", vd(ok), tt((orun > 0.2) | (rj > 0)) + " " + ts(sat))
  rr = col(R, "rate_rel")
  row("LCB-17", "release feel proxy: peak wheel rate in the 1 s after the release (deg/s; + owner impression)",
      f"n {len(R)}: median {med(rr):.0f}, p90 {pct(rr, 90):.0f}, max {rr.max() if len(R) else np.nan:.0f}", "CHECK" if R else "NO DATA",
      "fastest " + ts([r["t"] for r in sorted(R, key=lambda r: -r.get("rate_rel", 0))]))
  # LT-13: settling in the new lane (Phase 84: the usual position is forgotten, 3 s warm-up)
  S = []
  for i in np.flatnonzero((d["lcs"][1:] == 0) & (d["lcs"][:-1] > 0)) + 1:
    if i + 800 >= o["n"] or v[i] < 45 or o["rel"][i:i + 600].mean() <= 0.9:
      continue
    x = np.abs(o["pos"][i:i + 800] - o["smean"]); okf = o["fok"][i:i + 800]; bad = np.flatnonzero((x > 15) & okf)
    S.append((T[i], o["b15_new"][i:i + 295].sum() / 100, (bad[-1] + 1) / 100 if len(bad) else 0.0,
              np.nanmax(np.where(okf[100:600], x[100:600], np.nan)) if okf[100:600].any() else np.nan, bit(o["sft"][i:i + 500], 12).mean(),
              o["pr"][i:i + 300].mean()))
  S = np.array(S) if S else np.zeros((0, 6))
  if not len(S):
    row("LT-13", "after a lane change: settling in the new lane", "no lane change >= 45 km/h with delivery after it", "NO DATA"); return
  ok = S[:, 1].sum() == 0 and med(S[:, 2]) <= 3.5 and not med(S[:, 3]) > 30 and S[:, 4].mean() >= 0.7
  row("LT-13", "after a lane change: bit 15 in the 3 s warm-up, time until within 15 cm, largest offset 1-6 s",
      f"n {len(S)}: bit15 newly set in 0-2.95 s {S[:, 1].sum():.2f} s; within 15 cm after median {med(S[:, 2]):.1f} s (p90 {pct(S[:, 2], 90):.1f}); max offset "
      f"1-6 s median {med(S[:, 3]):.0f} cm; lane-centre bit first 5 s {S[:, 4].mean():.2f}; pressed first 3 s {S[:, 5].mean():.2f}",
      vd(ok, fail=S[:, 1].sum() > 0, enough=len(S) >= 4 and o["new_bits"]), "slowest " + ts(S[np.argsort(-S[:, 2])][:, 0]))


def pre_lane_change(o):
  d, T = o["d"], o["T"]; P = []
  for a, b in runs((d["lcs"] == 1) & (o["v"] > 30) & o["lat"]):
    if b - a < 30:
      continue
    sg = -1 if o["tr"][a:b].sum() >= o["tl"][a:b].sum() else 1; push = d["tq_s"][a:b] * sg > 150
    if push.any():
      P.append((T[a], ((d["wire"][a:b] - d["ang"][a:b]) * -sg)[push].max(), ((d["ang"][a] - d["ang"][a:b]) * sg).max()))
  P = np.array(P) if P else np.zeros((0, 3))
  if not len(P):
    row("LCB-14", "option E: before the change starts, does op steer back against a push toward the blinker side", "no pushed preLaneChange episode", "NO DATA")
    return
  hard = (P[:, 1] >= 4) & (P[:, 2] >= 3)
  row("LCB-14", "option E: before the change starts, does op steer back against a push toward the blinker side (deg)",
      f"n {len(P)}: request against the wheel max {P[:, 1].max():.1f}, <= 2 in {np.mean(P[:, 1] <= 2) * 100:.0f} %; >= 4 with a wheel reversal >= 3: "
      f"{int(hard.sum())}", vd(np.mean(P[:, 1] <= 2) >= 0.9, fail=hard.sum() >= 2), ts(P[np.argsort(-P[:, 1])][:, 0]))


# ---------------------------------------------------------------- parking latch, prompt, boot (Phases 76, 81)
def parking(o):
  d, T, v, n = o["d"], o["T"], o["v"], o["n"]; ang = d["ang"]; park = o["park"]; op = o["op"]; P = runs(park)
  seen = d["gps_seen"] if "gps_seen" in d and (d["gps_seen"] >= 0).any() else None
  unexp = []; mid = []; notes = []
  for k, (a, b) in enumerate(P):
    s = a
    while s > 0 and v[s - 1] < 30:
      s -= 1
    s = min(s, max(a - 300, 0))
    why = ("wheel" if np.abs(ang[s:a + 1]).max() >= 300 else "gear" if np.isin(d["gear"][s:a + 1], (1, 4)).any() else
           "no GPS fix yet" if (seen is not None and seen[a] == 0) or (seen is None and k == 0 and T[a] < 60) else "?")
    notes.append(f"{T[a]:.0f}-{T[min(b, n - 1)]:.0f} ({why})")
    if why == "?":
      unexp.append(T[a])
    if k > 0 and T[-1] - T[a] > 600:
      mid.append(T[a])
  fast = max([e - a for a, e in runs(park & (v >= 30))] + [0]) / 100
  row("PARK-1", "parking latch only at departure / arrival (no false latch on the road)",
      f"{len(P)} latches: {', '.join(notes)}; unexplained {len(unexp)}; mid-drive {len(mid)}; longest latched stretch >= 30 km/h {fast:.1f} s",
      vd(not unexp and fast <= 3.2, enough=not mid), ts(unexp + mid))
  # PARK-2: release time predicted by the two rules that are left, and by the removed cruise rule
  gapok = np.abs(d["apply"] - ang) < 10; res = []; f = lambda m, a: T[a + int(np.argmax(m))] if m.any() else np.inf  # noqa: E731
  for a, b in P:
    if b >= n:
      continue
    sl = slice(a, n)
    tsp = f(held(v[sl] >= 30, 300), a); tln = f(held((d["lane_min"][sl] >= 0.5) & (v[sl] >= 15) & (np.abs(ang[sl]) < 30) & gapok[sl], 300), a)
    tcr = f(held(d["cruise"][sl] & (v[sl] >= 15) & (np.abs(ang[sl]) < 30) & gapok[sl], 100), a)
    res.append((T[b], min(tsp, tln), "speed" if tsp <= tln else "lanes", tcr))
  if res:
    err = [abs(x[0] - x[1]) for x in res]; byc = [x for x in res if abs(x[0] - x[3]) <= 0.3 and x[1] > x[0] + 1]
    row("PARK-2", "latch released by the two remaining rules, never by cruise (81a)",
        "; ".join(f"actual {x[0]:.2f} s, rule {x[1]:.2f} ({x[2]}), old cruise rule {x[3]:.2f}" for x in res),
        vd(max(err) <= 0.3 and not byc, fail=bool(byc) or max(err) > 1.0), ts([x[0] for x in byc]))
    e = int(np.searchsorted(T, res[0][0])); w = slice(e, min(n, e + 4000)); lo = (op[w] & (d["lane_min"][w] < 0.3)).sum() / 100
    ml = float(np.median(d["lane_min"][w])); ma = np.abs(ang[w]).max()
    row("PARK-3", "40 s after the first release: op steering where there are no lanes (the a9 car park)",
        f"op {op[w].sum() / 100:.1f} s, hands-off {(op[w] & o['hoc'][w]).sum() / 100:.1f} s, op with lane prob < 0.3 {lo:.1f} s, median lane prob {ml:.2f}, "
        f"max |wheel| {ma:.0f} deg", vd(lo <= 5 and ml >= 0.5 and ma < 90, fail=lo > 10), f"t={T[e]:.0f}")
  else:
    row("PARK-2", "latch released by the two remaining rules, never by cruise (81a)", "no release in the log", "NO DATA")
    row("PARK-3", "40 s after the first release: op steering where there are no lanes", "no release in the log", "NO DATA")
  wt = d["cruise"] & (v >= 15) & park; rw = [((wt & (d["lane_min"] >= 0.5) & (np.abs(ang) < 30))[a:b].sum() / 100, T[a]) for a, b in P]
  r4 = max([e - a for a, e in runs(o["lat"] & (v >= 30) & ~op & (d["lat_reason"] == 4))] + [0]) / 100
  fo = T[int(np.argmax(op))] if op.any() else np.nan; fc = T[int(np.argmax(d["cruise"]))] if d["cruise"].any() else np.nan
  row("PARK-4", "cost of 81a: wait with cruise on and no steering",
      f"cruise on, >= 15 km/h, latched {wt.sum() / 100:.1f} s; of it on a marked road per latch max {max([x[0] for x in rw] + [0]):.1f} s; parking pause "
      f">= 30 km/h longest {r4:.1f} s; first cruise {fc:.1f} s, first op steering {fo:.1f} s",
      vd(max([x[0] for x in rw] + [0]) <= 6 and r4 <= 3.2, fail=False), ts([x[1] for x in rw if x[0] > 6]))
  # PROMPT-N: expected 'Steering Paused / Parking Mode' prompts from the npz, actual from the alert texts (v8)
  cond = d["lat_paused"] & (d["lat_reason"] == 4) & d["cruise"] & (v > 15) & ~o["pr"]; nop = ~(box(np.r_[np.zeros(50), o["pr"][:-50]], 100) > 0)
  due = held(cond, 150) & nop; exp_ = []; last = -1e9; ep = -1; epi = np.cumsum(np.r_[0, np.diff(d["lat_paused"].astype(int)) != 0])
  for i in np.flatnonzero(due):
    if epi[i] != ep and T[i] - last >= 20:
      exp_.append(T[i]); last = T[i]; ep = epi[i]
  if "alerts" in d:
    act = [(float(a[0]), str(a[2])) for a in d["alerts"] if str(a[1]) == "Steering Paused"]
    pk = [t for t, t2 in act if "Park" in t2]; rs_ = [int(d["lat_reason"][min(int(np.searchsorted(T, t)), n - 1)]) for t, _ in act]
    bad = [t for (t, _), r in zip(act, rs_, strict=True) if r in (5, 6, 10)]
    off = [t for t in pk if not any(abs(t - e) <= 0.5 for e in exp_)] + [e for e in exp_ if not any(abs(t - e) <= 0.5 for t in pk)]
    row("PROMPT-N", "'Steering Paused / Parking Mode' prompt: when expected, never for driver-steering reasons",
        f"expected {len(exp_)} ({ts(exp_)}), shown {len(pk)} parking + {len(act) - len(pk)} other; reasons at the prompts {dict(Counter(rs_))}; "
        f"condition time {cond.sum() / 100:.1f} s", vd(not off and not bad, fail=bool(bad)), ts(sorted(off + bad), 6))
  else:
    row("PROMPT-N", "'Steering Paused / Parking Mode' prompt: when expected", f"expected {len(exp_)} ({ts(exp_)}), condition time "
        f"{cond.sum() / 100:.1f} s; alert texts need a v8 npz", "NO DATA")
  # GPS-1 (81c): a GPS-only latch after the car has been on the road
  if seen is None:
    row("GPS-1", "no-GPS-fix signal cannot latch after the car was on the road (81c)", "needs a v8 npz (gpsFixSeen)", "NO DATA"); return
  tf = T[int(np.argmax(seen == 1))] if (seen == 1).any() else np.inf; rd = held((v >= 30) & ~park, 300)
  tr_ = T[int(np.argmax(rd))] if rd.any() else np.inf
  late = [T[a] for k, (a, b) in enumerate(P) if tr_ < T[a] < tf and "no GPS" in notes[k]]
  row("GPS-1", "no-GPS-fix signal cannot latch after the car was on the road (81c)",
      f"first fix seen {tf:.1f} s, on the road (30 km/h 3 s, latch clear) {tr_:.1f} s; GPS-only latches between: {len(late)}",
      "FAIL" if late else ("PASS" if tr_ < tf else "NO DATA"), ts(late) if late else ("" if tr_ < tf else "path not exercised (fix came first)"))


def boot(o):
  d, T = o["d"], o["T"]; ev = o["ev"]; mm = [t for t, nm in ev if nm == "controlsMismatch"]
  val = f"controlsMismatch: {sum(1 for t in mm if t < 30)} before 30 s, {sum(1 for t in mm if t >= 30)} after"
  if "rx_invalid" in d and (d["rx_invalid"] >= 0).any():
    iv = runs(d["rx_invalid"] == 1)
    val += "; RX checks invalid " + (", ".join(f"{T[a]:.2f}-{T[min(b, o['n'] - 1)]:.2f} s" for a, b in iv) if iv else "never")
    if iv and not mm:
      val += " (race happened, grace covered it)"
  else:
    val += "; RX-check flag needs a v8 npz"
  row("BOOT-1", "no controlsMismatch at boot (81b), none on the road", val, vd(not mm), ts(mm))
  early = [(t, nm) for t, nm in ev if t < 30 and nm not in ("selfdriveInitializing", "steerOverride", "gasPressedOverride", "preEnableStandstill",
                                                                "controlsMismatch")]    # the mismatch is BOOT-1's
  fl = T[int(np.argmax(o["lat"]))] if o["lat"].any() else np.nan; fc = T[int(np.argmax(d["cruise"]))] if d["cruise"].any() else np.nan
  fm = T[int(np.argmax(d["mads_state"] >= 0))] if "mads_state" in d and (d["mads_state"] >= 0).any() else np.nan
  row("BOOT-2", "start-up sequence unchanged", f"first carState {T[0]:.1f} s, first MADS state {fm:.1f} s, first lateral {fl:.1f} s, first cruise {fc:.1f} s; "
      f"other events before 30 s: {[nm for _, nm in early]}", vd(not early), ts([t for t, _ in early]))


# ---------------------------------------------------------------- open items F, hands-off large angle, storage, health
def item_f(o):
  d, T, v = o["d"], o["T"], o["v"]; kc, km, g = o["kc"], o["kme"], d["gain"]; m = o["op"] & o["hoc"] & (v >= 45)
  und = m & (g < 0.8) & (np.abs(kc) >= 2e-3) & (np.sign(kc) * (kc - km) >= 1e-3)
  ep = [(T[a], (b - a) / 100, v[a], np.abs(kc[a:b] - km[a:b]).max() * 1e3) for a, b in runs(und) if b - a >= 100]
  sat = [t for t, nm in o["ev"] if nm == "steerSaturated" and np.interp(t, T, v) >= 45]
  per_h = sum(1 for e in ep if e[2] >= 60) / o["hours"]; city = (bit(o["sfc"], 18) & (v > 45.5)).sum() / 100
  rl = [(b - a) / 100 for a, b in runs(m & (g < 0.8)) if b - a >= 20]
  row("F-1", "option F: after a release >= 45 km/h the gain climbs slowly while a curve request builds (a8 945 s)",
      f"under-delivery {und.sum() / 100:.2f} s; episodes >= 1.0 s: {len(ep)} ({per_h:.1f} / h at >= 60 km/h)"
      + "".join(f" [t={e[0]:.1f} {e[1]:.2f} s {e[2]:.0f} km/h {e[3]:.2f}e-3]" for e in ep[:4]) +
      f"; steerSaturated >= 45 km/h {len(sat)}; gain < 0.8 runs p50 / p90 / max {med(rl):.2f} / {pct(rl, 90):.2f} / {max(rl + [0]):.2f} s; "
      f"city-release bit > 45 km/h {city:.1f} s", vd(per_h <= 1 and not sat and city == 0, fail=(per_h >= 2 and sum(1 for e in ep if e[2] >= 60) >= 2) or any(e[3] >= 3 and e[1] >= 1.5 for e in ep)),
      ts([e[0] for e in ep] + sat, 6))


def hands_off_angle(o):
  d, T, v = o["d"], o["T"], o["v"]; m = o["op"] & o["hoc"]; blk = (d["lb"] | d["rb"]).astype(bool); ang = np.abs(d["ang"])
  A = m & (ang >= 60); An = A & ~blk; B = m & (d["lane_min"] < 0.3) & (np.abs(o["kc"]) >= 10e-3); Bn = B & ~blk
  C = m & (np.abs(d["rate"]) >= 60) & (v >= 20)
  sat = [t for t, nm in o["ev"] if nm == "steerSaturated" and np.interp(t, T, v) < 30 and m[min(int(np.searchsorted(T, t)), o["n"] - 1)]]
  ok = An.sum() == 0 and Bn.sum() / 100 / o["hours"] <= 2 and not sat
  row("HOFF-1", "hands-off: op turns the wheel far on a low-confidence scene (a8 925-928 s; classify with video)",
      f"|wheel| >= 60 deg {A.sum() / 100:.2f} s (no blinker {An.sum() / 100:.2f} s; >= 90 deg {(m & (ang >= 90)).sum() / 100:.2f} s); lane prob < 0.3 "
      f"and radius < 100 m {B.sum() / 100:.2f} s (no blinker {Bn.sum() / 100:.2f} s = {Bn.sum() / 100 / o['hours']:.1f} s / h); wheel rate >= 60 deg/s "
      f"at >= 20 km/h {C.sum() / 100:.2f} s; steerSaturated < 30 km/h hands-off {len(sat)}", "PASS" if ok else "CHECK",
      ts([T[a] for a, e in runs(box(An | Bn, 300) > 0)], 8))


def storage_and_health(o):
  d, T, v, n = o["d"], o["T"], o["v"], o["n"]; ev = o["ev"]; names = Counter(nm for _, nm in ev)
  if "free_pct" in d and (d["free_pct"] >= 0).any():
    f = d["free_pct"][d["free_pct"] >= 0]
    row("STOR-1", "free space above the deleter threshold (10 %), far from the 7 % alert",
        f"start {f[0]:.2f} %, end {f[-1]:.2f} %, min {f.min():.2f} %, {(f[0] - f[-1]) / max((T[-1] - T[0]) / 60, 1e-6):.3f} % / min",
        vd(f.min() >= 10.0, fail=f.min() < 7.0 or "outOfSpace" in names))
  else:
    row("STOR-1", "free space above the deleter threshold", f"needs a v8 npz; outOfSpace events {names.get('outOfSpace', 0)}",
        "FAIL" if "outOfSpace" in names else "NO DATA")
  bad = [(t, nm) for t, nm in ev if nm in EV_BAD]; other = [(t, nm) for t, nm in ev if nm not in EV_OK and nm not in EV_BAD]
  sat = [t for t, nm in ev if nm == "steerSaturated"]; sat_ho = [t for t in sat if np.interp(t, T, v) >= 45 and o["hoc"][min(int(np.searchsorted(T, t)), n - 1)]]
  per30 = len(sat) / max((T[-1] - T[0]) / 1800, 1e-6)
  row("HEALTH-1", "events: nothing new or safety-relevant", f"{dict(names.most_common())}"
      + (f"; alert texts {dict(Counter(str(a[1]).replace(chr(10), ' ') for a in d['alerts']).most_common(8))}" if "alerts" in d and len(d["alerts"]) else ""),
      vd(not bad and not other and per30 <= 1 and not sat_ho, fail=bool(bad) or bool(sat_ho)),
      " ".join(f"{nm}@{t:.1f}" for t, nm in (bad + other)[:6]) + (" steerSaturated " + ts(sat) if sat else ""))
  sfc = o["sfc"]; sec = lambda m: m.sum() / 100  # noqa: E731
  rj = d["tx_reject"] > 0; burst = max([e - a for a, e in runs(rj & (T > T[0] + 30))] + [0]); rs = Counter(d["lat_reason"].tolist())
  odd = sum(rs.get(k, 0) for k in (1, 2, 8, 9)) / 100; lfa = sec(d["lfa_sym"] == 2) if (d["lfa_sym"] >= 0).any() else 0.0
  sat_s = sec(bit(sfc, 27)) / max((T[-1] - T[0]) / 1800, 1e-6)
  ok = (sec(d["mdps_fault"] > 0) == 0 and rj.sum() <= 10 and burst <= 3 and sec(bit(sfc, 24)) == 0 and sec(bit(sfc, 22)) == 0 and lfa == 0
        and sat_s <= 5 and odd <= 0.2)
  row("HEALTH-2", "steering chain: MDPS fault, panda rejects, camera-stale, factory LFA",
      f"MDPS fault {sec(d['mdps_fault'] > 0):.2f} s; TX reject frames {int(rj.sum())} (longest burst after boot {burst}); TX saturated {sec(bit(sfc, 27)):.2f} s; camera "
      f"fault {sec(bit(sfc, 24)):.2f} s; VM reject {sec(bit(sfc, 22)):.2f} s; wheel outrun {sec(bit(sfc, 25)):.2f} s; TX active w/o MDPS active "
      f"{sec((d['tx_active'] > 0) & (d['mdps_active'] == 0)):.2f} s; lateral w/o panda grant {sec(o['lat'] & ~d['lat_granted']):.2f} s; factory LFA active "
      f"{lfa:.1f} s; pause reasons (s) { {int(k): round(c / 100, 1) for k, c in sorted(rs.items()) if k} }",
      vd(ok, fail=sec(d["mdps_fault"] > 0) > 0 or sec(bit(sfc, 24)) > 0 or lfa > 0), ts(T[np.flatnonzero(rj)], 6))
  m = o["op"] & o["hoc"] & (v >= 30); gp = np.abs(d["apply"] - d["ang"]); big = o["op"] & o["hoc"] & (gp >= 10)
  dl = (o["op"] & (v > 30)).sum() / max((v > 30).sum(), 1) * 100; dm = (o["op"] & (v > 1)).sum() / max((v > 1).sum(), 1) * 100
  row("HEALTH-3", "wheel follows the request; delivery",
      f"|request - wheel| hands-off >= 30 km/h p50 / p95 / p99 {pct(gp[m], 50):.2f} / {pct(gp[m], 95):.2f} / {pct(gp[m], 99):.2f} deg, > 3 deg "
      f"{np.mean(gp[m] > 3) * 100 if m.any() else np.nan:.1f} %, >= 10 deg {big.sum() / 100:.2f} s; delivery of moving time {dm:.1f} %, above 30 km/h {dl:.1f} %",
      "NO DATA" if not m.any() else vd(pct(gp[m], 50) <= 0.6 and pct(gp[m], 95) <= 3.5 and dl >= 95, fail=dl < 90),
      ts([T[a] for a, e in sorted(runs(big), key=lambda x: x[0] - x[1])]))
  nf = [k for k in d if d[k].dtype.kind == "f" and d[k].ndim == 1 and len(d[k]) == n and not np.isfinite(d[k]).all()]
  val = f"non-finite fields {nf}; carState gaps > 0.05 s: {int((np.diff(T) > 0.05).sum())}"; ok = not nf; tw = ""
  if "can_valid" in d:
    cvb = np.flatnonzero(~d["can_valid"]); late = cvb[T[cvb] > T[0] + 20] if len(cvb) else cvb
    mg = d["model_dt"] > 0.2; inv = {str(r[0]): (int(r[1]), round(float(r[3]), 1)) for r in d["invalid"] if str(r[0]) in ("controlsState", "carControl", "carState", "modelV2")}
    val += (f"; canValid False {len(cvb)} frames ({len(late)} after 20 s); frameDropPerc max {d['frame_drop'].max():.1f} %; model gaps > 0.2 s "
            f"{len(runs(mg))}; CPU temp max {d['cpu_temp'].max():.1f} C, memory max {d['mem_pct'].max():.0f} %; invalid frames (n, last t) {inv}")
    ok = ok and len(late) == 0 and len(cvb) <= 20 and d["frame_drop"].max() < 5 and not mg.any() and d["cpu_temp"].max() < 80
    tw = ts(T[late[::50]]) + " " + ts([T[a] for a, e in runs(mg)])
  row("HEALTH-4", "CAN validity, model timing, non-finite values", val, vd(ok, fail=bool(nf), enough="can_valid" in d), tw)


MANUAL = (
  ("LT-02", "replay the route through HEAD controlsd (probes/qa/e180_full_route_replay.py on replay_state_control.py; needs the plan\n"
            "         npz S/la/plan_<route>.npz from the raw rlog): logged bits 14 / 15 and k_ctrl must match"),
  ("LT-09 / LT-11", "how deep the release went (weight < 0.5) is not in the log: same replay, column O[:, 9]"),
  ("LT-10", "video at the LT-09 time stamps: real avoidance, curve entry or drift? was the car pulled back toward the obstacle?"),
  ("LT-16", "integrator of the lane-centre term (|_lc_i| max < 0.25e-3): replay column O[:, 6]"),
  ("G-1", "size of the entry lead: e174_entry_lead_replay.py"),
  ("HOFF-1", "video at each listed time: intended junction turn / road bend / spurious"),
  ("LT-17, LCB-13, LCB-15, LCB-17", "owner's impression (curve side, weave, tug at release, icon too long, push = trigger?)"),
)


def run(path):
  ROWS.clear(); o = load(path)
  for f in (build_and_log, sufficiency, lane_bits, steady_curves, curve_entries, curve_exits, straights, reengage, entry_lead,
            driver_presses, blinker, lane_change_items, pre_lane_change, parking, boot, item_f, hands_off_angle, storage_and_health):
    k = len(ROWS)
    try:
      f(o)
    except Exception as e:  # noqa: BLE001 - one broken item must not hide the others
      del ROWS[k:]
      row(f.__name__, "(item group failed)", f"{type(e).__name__}: {e}", "CHECK")
  return list(ROWS), o


def main(argv):
  base = argv[argv.index("--baseline") + 1:] if "--baseline" in argv else []
  path = argv[0]
  B = [(p.split("/")[-1].replace(".npz", "")[-2:], {r[0]: r[2] for r in run(p)[0]}) for p in base]
  R, o = run(path)
  print(f"=== {path.split('/')[-1]}: commit {o['d']['commit']}, npz v{o['ver']}, {(o['T'][-1] - o['T'][0]) / 60:.1f} min, "
        f"moving {o['hours'] * 60:.1f} min, lane changes {len(o.get('LC', []))}")
  for i, what, val, ver, when in R:
    print(f"{i:9s} {ver:7s} {what}\n{'':17s} now: {val}")
    for nm, b in B:
      print(f"{'':17s} {nm:>3s}: {b.get(i, '-')}")
    if when.strip():
      print(f"{'':17s} see: {when.strip()}")
  c = Counter(r[3] for r in R)
  print("--- " + ", ".join(f"{k} {c.get(k, 0)}" for k in ("PASS", "FAIL", "CHECK", "NO DATA")) + "; FAIL: " + " ".join(r[0] for r in R if r[3] == "FAIL"))
  if o.get("LC"):
    print("--- lane changes (t = state start; times in s after it; clearance to the far line in m)")
    for r in o["LC"]:
      g = lambda k, f="{:.2f}": f.format(r[k]) if k in r and np.isfinite(r[k]) else "-"  # noqa: E731
      print(f"  t={r['t']:7.1f} {r['dir']} {r['v']:3.0f} km/h | blinker on {r['on'] - r['t']:+.1f} off {r['off'] - r['t']:+.1f} cross "
            f"{g('cross', '{:.1f}') if not np.isfinite(r['cross']) else format(r['cross'] - r['t'], '+.1f')} end {r['end'] - r['t']:+.1f} | push {r['push_pk']:.0f} "
            f"| released gain {g('gain_ho')} auth {g('auth_ho')} conc {g('conc_ho')} -> 0.9 in {g('rel2gain09')} s | re-grip {g('regrip_tq', '{:+.0f}')} "
            f"| blend {r['blend_s']:.2f} s | clearance {g('far_min', '{:+.2f}')} offset {g('pos_max')} settle {g('settle', '{:.1f}')} {'clean' if r.get('clean') else 'unclean'}")
  print("--- not computed here")
  for i, t in MANUAL:
    print(f"  {i}: {t}")


if __name__ == "__main__":
  if len(sys.argv) < 2:
    sys.exit(__doc__)
  main(sys.argv[1:])
