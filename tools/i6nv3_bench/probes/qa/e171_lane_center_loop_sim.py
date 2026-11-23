"""Closed-loop check of the lane-centre term. Car: e'' = v^2 (k_act - k_road + k_dist), heading psi' = v (k_act - k_road).
Model law (a7 fit): K = k_road_ff - 0.045 psi - 0.786e-3 (e - p*) (20 Hz, 0.1 s old); actuator: delay 0.19 s + lag 0.13 s.
Lane-centre term as in controlsd: P on the plan at L = 2 s (plan(L) = p* + (e - p*) exp(-L / 25 m)), low-pass 0.3 s,
integral (straights only, capped), new: KE on the car's own offset. e, p*, target in m, + = right."""
import numpy as np, sys
def run(v, W, KI, ICAP, KE, pstar, kdist, kroad_t, T=80.0, noise=0.0, seed=0, target=-0.034, tau_e=0.3):
    dt = 0.01; n = int(T / dt); rng = np.random.default_rng(seed)
    e = pstar; psi = 0.0; act = 0.0; buf = [0.0] * 19; mbuf = [(e, psi)] * 10; p = 0.0; I = 0.0; fe = 0.0
    E = np.zeros(n); A = np.zeros(n); kmodel = 0.0; ns = 0.0
    L = float(np.clip(v * 2.0, 12, 80)); cap = 0.4 / v ** 2
    for i in range(n):
        kroad = kroad_t(i * dt)
        if i % 5 == 0:
            em, pm = mbuf[0]; ns = 0.9 * ns + noise * rng.standard_normal()      # lane-line noise (m)
            kmodel = kroad - 0.045 * pm - 0.786e-3 * (em - pstar)
            planL = pstar + (em - pstar) * np.exp(-L / 25.0)
            raw = float(np.clip(W * 2.0 * ((target + ns) - planL) / L ** 2, -cap, cap)); e0 = (target + ns) - em     # centre - car ... sign: push toward target
        p += dt / (0.3 + dt) * (raw - p)
        fe += dt / (tau_e + dt) * (float(np.clip(KE * e0, -cap, cap)) - fe)
        if abs(kmodel) < 1e-3: I = float(np.clip(I + KI * e0 * dt, -ICAP, ICAP))
        cmd = kmodel + p + I + fe
        buf.append(cmd); d = buf.pop(0); act += dt / (0.13 + dt) * (d - act)
        psi += v * (act - kroad) * dt; e += v * psi * dt + 0.5 * v * v * kdist * dt * dt
        psi += v * kdist * dt
        mbuf.append((e, psi)); mbuf.pop(0); E[i] = e; A[i] = act - kroad
    return E, A
OLD = dict(W=0.5, KI=0.03e-3, ICAP=0.15e-3, KE=0.0)
CANDS = {"old": OLD, "KI.06 cap.30": dict(W=0.5, KI=0.06e-3, ICAP=0.30e-3, KE=0.0),
         "KE.5 + KI.06": dict(W=0.5, KI=0.06e-3, ICAP=0.30e-3, KE=0.5e-3), "KE.8 + KI.06": dict(W=0.5, KI=0.06e-3, ICAP=0.30e-3, KE=0.8e-3),
         "KE1.2 + KI.06": dict(W=0.5, KI=0.06e-3, ICAP=0.30e-3, KE=1.2e-3), "KE.8 + KI.10": dict(W=0.5, KI=0.10e-3, ICAP=0.30e-3, KE=0.8e-3)}
print("straight: model prefers +2.5 cm, disturbance -> needs -0.14e-3 (road crown), target -3.4 cm. curve: 20 s straight then K=2.5e-3 with the model's plan 12 cm inside")
for v in (40 / 3.6, 60 / 3.6, 80 / 3.6, 110 / 3.6):
    print(f"-- {v*3.6:.0f} km/h")
    for nm, c in CANDS.items():
        E, A = run(v, pstar=0.025, kdist=0.14e-3, kroad_t=lambda t: 0.0, **c)
        e20 = (E[2000] + 0.034) * 100; e60 = (E[-1] + 0.034) * 100
        # step response damping: overshoot count around the final value
        x = E - E[-1]; cross = int(np.sum(np.diff(np.sign(x[200:])) != 0))
        Ec, Ac = run(v, pstar=0.025, kdist=0.14e-3, kroad_t=lambda t: 0.0 if t < 30 else 2.5e-3, T=50, **c)
        # in the curve the model's preferred position moves 12 cm inside (right curve -> +)
        Ec2, _ = run(v, pstar=0.12, kdist=0.14e-3, kroad_t=lambda t: 2.5e-3, T=25, **dict(c, KI=0.0))
        En, An = run(v, pstar=0.025, kdist=0.14e-3, kroad_t=lambda t: 0.0, noise=0.012, T=120, **c)
        print(f"   {nm:16s} straight error vs target: 20 s {e20:+5.1f} cm, 80 s {e60:+5.1f} cm, zero crossings {cross:2d} | curve steady {(Ec2[-1]+0.034)*100:+5.1f} cm inside (peak {np.max(Ec2[500:]+0.034)*100:+5.1f}) | with line noise: scatter {En[3000:].std()*100:4.1f} cm, wheel-eq curvature std {An[3000:].std()*1e3:.3f}e-3")
