"""E64a: route-fold fits at the common 0.20 s delay (for cross-validated compensator parameters in E64)."""
import json, pickle, sys
import numpy as np
sys.path.insert(0, sys.argv[1] + "/qa")
S = sys.argv[1]
src = open(f"{S}/qa/e63_delay_smooth.py").read()
pre = src[:src.index("DG = np.arange")]
exec(pre.replace("@njit(cache=True)", "@njit"))
out = {}
for fo, tag in ((0, "A"), (1, "B")):
  P = packs(fo)
  for b in range(1, 6):
    out[f"{tag}{b}"] = fit_at(0.20, P[b])[0]
    print(tag, b, np.round(out[f"{tag}{b}"], 3).tolist(), flush=True)
json.dump(out, open(f"{S}/qa/e64_common_fold.json", "w"), indent=1)
