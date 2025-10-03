#!/usr/bin/env python3
# gyro_turns_fit.py
# Robustly fit a yaw-rate FOPDT from step tests and synthesize IMC PI gains.
# Usage:  python gyro_turns_fit.py run1.csv [run2.csv ...]

import sys, numpy as np

def imc_pi(K, tau, theta=0.0, lam_factor=0.5):
    # IMC PI for FOPDT; lambda tied to tau, not smaller than theta
    tau = max(float(tau), 1e-6)
    theta = max(float(theta), 0.0)
    lam = max(theta, lam_factor * tau)
    Kp = tau / (K * (lam + theta))
    Ti = min(tau, 4.0 * (lam + theta))
    Ki = Kp / Ti if Ti > 1e-9 else 0.0
    return Kp, Ki, Ti, lam

def _names(d):
    return {n.lower(): n for n in d.dtype.names}

def fit_file(path):
    d = np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding=None)
    if d.size < 10 or d.dtype.names is None:
        raise RuntimeError(f"{path}: not enough data / no header")

    nm = _names(d)

    # Require the signed command fields
    for req in ("t_ms","left_cmd","right_cmd","gyro_z"):
        if req not in nm:
            raise RuntimeError(f"{path}: missing column '{req}'")

    t  = d[nm["t_ms"]].astype(float) / 1000.0
    L  = d[nm["left_cmd"]].astype(float)     # signed: +fwd, -rev
    R  = d[nm["right_cmd"]].astype(float)    # signed
    gz = d[nm["gyro_z"]].astype(float)       # deg/s

    # Effective yaw input u = R - L
    u = R - L

    # --- Detect step start & end from u ---
    # Baseline = median of first ~0.4 s (or first 20 samples)
    n0 = max(20, int(0.4 / max(np.mean(np.diff(t)), 1e-3)))
    u0 = float(np.median(u[:min(n0, len(u)//3)]))

    # Find first index where |u - u0| exceeds threshold
    thr = max(50.0, 5.0 * np.median(np.abs(u[:n0] - u0)))  # robust threshold
    step_on = np.where(np.abs(u - u0) > thr)[0]
    if step_on.size == 0:
        raise RuntimeError(f"{path}: no step detected (thr={thr:.1f})")
    si = int(step_on[0])

    # Find step OFF: first time after si where |u - u0| drops back near baseline
    step_off_candidates = np.where(np.abs(u[si+1:] - u0) <= thr * 0.5)[0]
    if step_off_candidates.size == 0:
        so = len(u) - 1
    else:
        so = si + 1 + int(step_off_candidates[0])

    # Guard windows inside the step for plateau stats
    # Skip a short transient (~0.12 s) after step ON and before OFF.
    dt = np.mean(np.diff(t)) if len(t) > 1 else 0.01
    guard = max(3, int(0.12 / max(dt, 1e-3)))
    a = min(so - 1, si + guard)
    b = max(a + 3, so - guard)  # ensure non-empty

    if b <= a + 2:
        raise RuntimeError(f"{path}: step too short for plateau (si={si}, so={so})")

    # Pre-step baseline medians
    y0 = float(np.median(gz[max(0, si - n0):si])) if si > 0 else float(np.median(gz[:max(5, n0)]))
    u_pre = float(np.median(u[max(0, si - n0):si]))

    # In-step plateau medians (THIS is our "y_inf" and "u1")
    y_plateau = float(np.median(gz[a:b]))
    u_plateau = float(np.median(u[a:b]))

    dPWM = u_plateau - u_pre
    if abs(dPWM) < 1e-6:
        raise RuntimeError(f"{path}: ΔPWM too small inside step")

    # Process gain from plateau inside step
    K_proc = (y_plateau - y0) / dPWM

    # Time constant & dead time from 10%/63% crossings INSIDE the step
    tgt10 = y0 + 0.10 * (y_plateau - y0)
    tgt63 = y0 + 0.632 * (y_plateau - y0)
    trend = np.sign(y_plateau - y0) or 1.0

    # search after si within [si:so]
    def first_hit(target):
        seg = trend * (gz[si:so] - target)
        idx = np.where(seg >= 0)[0]
        return (t[si + idx[0]] - t[si]) if idx.size else None

    theta = first_hit(tgt10) or 0.0
    tau   = first_hit(tgt63) or max(0.01, (t[so] - t[si]) / 4.0)

    # PI gains (Kd=0 to start)
    Kp_g, Ki_g, Ti, lam = imc_pi(K_proc, tau, theta, lam_factor=0.5)

    # OPTIONAL micro Kd suggestion from noise proxy (stay tiny or zero)
    d_gz = np.diff(gz) / np.diff(t) if len(gz) > 2 else np.array([0.0])
    noise = np.median(np.abs(d_gz - np.median(d_gz)))
    Kd_g = 0.0 if noise >= 150.0 else 0.02 * Ti

    return {
        "file": path,
        "si": si, "so": so,
        "u0": u0, "u_plateau": u_plateau, "dPWM": dPWM,
        "y0": y0, "y_plateau": y_plateau,
        "K_proc": K_proc, "tau": tau, "theta": theta,
        "Kp_g": Kp_g, "Ki_g": Ki_g, "Kd_g": Kd_g, "Ti": Ti, "lambda": lam,
    }

def robust_avg(vals):
    vals = np.array(vals, dtype=float)
    med  = np.median(vals)
    mad  = np.median(np.abs(vals - med)) + 1e-9
    keep = np.abs(vals - med) <= 3.5 * mad
    return float(np.mean(vals[keep])), int(np.sum(keep)), int(len(vals))

def main(paths):
    if not paths:
        print("Usage: python gyro_turns_fit.py run1.csv [run2.csv ...]")
        sys.exit(1)

    rows = []
    for p in paths:
        try:
            r = fit_file(p)
            rows.append(r)
            print(f"[{p}] step [{r['si']}..{r['so']}]  ΔPWM={r['dPWM']:.1f}  "
                  f"K={r['K_proc']:.6f}  tau={r['tau']:.3f}  theta={r['theta']:.3f}")
            print(f"      PI: Kp_g={r['Kp_g']:.6f}  Ki_g={r['Ki_g']:.6f}  "
                  f"(Ti={r['Ti']:.3f}, λ={r['lambda']:.3f})  Suggested Kd_g={r['Kd_g']:.4f}\n")
        except Exception as e:
            print(f"[{p}] ERROR: {e}")

    if not rows:
        print("No valid files.")
        return

    Kp_avg, k1, kN = robust_avg([r['Kp_g'] for r in rows])
    Ki_avg, k2, _  = robust_avg([r['Ki_g'] for r in rows])
    Kd_avg, k3, _  = robust_avg([r['Kd_g'] for r in rows])

    print("=== FINAL RECOMMENDED (robust average; outliers auto-dropped) ===")
    print(f"Kp_g = {Kp_avg:.6f}   Ki_g = {Ki_avg:.6f}   Kd_g = {Kd_avg:.6f}")
    print(f"(kept {k1}/{kN} for Kp, {k2}/{kN} for Ki; keep Kd_g = 0.0 if unsure)")

if __name__ == "__main__":
    main(sys.argv[1:])
