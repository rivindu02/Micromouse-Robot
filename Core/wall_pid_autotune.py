#!/usr/bin/env python3
import argparse, sys, math
import numpy as np
import pandas as pd

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None

def load_series(path, t_col, u_col, y_col):
    df = pd.read_csv(path)
    if t_col not in df.columns or u_col not in df.columns or y_col not in df.columns:
        raise ValueError(f"CSV must have columns: {t_col}, {u_col}, {y_col}")
    t = df[t_col].to_numpy(dtype=float)
    u = df[u_col].to_numpy(dtype=float)
    y = df[y_col].to_numpy(dtype=float)

    # ensure sorted by time
    idx = np.argsort(t); t, u, y = t[idx], u[idx], y[idx]
    m = np.isfinite(t) & np.isfinite(u) & np.isfinite(y)
    t, u, y = t[m], u[m], y[m]

    # --- FORCE seconds ---
    # If time appears to be in milliseconds (dt > 1), convert to seconds.
    if len(t) > 2:
        dt_guess = np.median(np.diff(t))
        if dt_guess > 1.0:      # e.g., 57 => ms
            t = t / 1000.0
    return t, u, y


def moving_avg(x, w):
    if w <= 1: return x.copy()
    y = np.convolve(x, np.ones(w)/w, mode='same')
    y[:w]  = np.mean(x[:w])
    y[-w:] = np.mean(x[-w:])
    return y

def fit_arx_first_order(t, u, y, delay_max_s=1.5):
    """ Fit y[k] = a*y[k-1] + b*u[k-d]. Scan delay d, LS for (a,b). """
    dt = np.median(np.diff(t))
    if dt <= 0:
        raise ValueError("Non-positive dt from time vector")
    dmax = int(max(0, round(delay_max_s / dt)))
    N = len(y)
    best = None
    us = moving_avg(u, max(1, int(0.03 / max(dt,1e-3))))  # ~30 ms
    ys = moving_avg(y, max(1, int(0.05 / max(dt,1e-3))))  # ~50 ms
    for d in range(0, min(dmax, N-3)+1):
        k_from = 1 + d
        if k_from+50 >= N: break
        Y = ys[k_from:N]
        phi_y = ys[k_from-1:N-1]
        phi_u = us[k_from-d:N-d]
        Phi = np.column_stack([phi_y, phi_u])
        try:
            theta, *_ = np.linalg.lstsq(Phi, Y, rcond=None)
        except np.linalg.LinAlgError:
            continue
        a, b = theta
        Y_hat = Phi @ theta
        resid = Y - Y_hat
        rss = float(np.mean(resid**2))
        penalty = 0.0 if (0.0 < a < 1.2) else 1.0  # prefer stable-ish a
        score = rss * (1.0 + penalty)
        if (best is None) or (score < best[0]):
            best = (score, a, b, d)
    if best is None:
        raise RuntimeError("ARX fit failed.")
    _, a, b, d = best
    return a, b, d, dt

def arx_to_fopdt(a, b, dt):
    """ a = e^{-dt/tau} -> tau;  K = b/(1-a) """
    a_use = min(max(a, 1e-6), 0.999999)
    tau = -dt / math.log(a_use)
    K = b / (1.0 - a)
    return K, tau

def simulate_arx_pid(t, r, a, b, d, Kp, Ki, Kd, der_alpha=0.9, u_cap=1.5):
    dt = np.median(np.diff(t))
    N = len(t)
    y = np.zeros(N)
    u_cmd = np.zeros(N)
    ei = 0.0
    e_prev = 0.0
    d_f = 0.0
    buf = [0.0]*(d+1)  # delay buffer
    for k in range(1, N):
        e = r[k] - y[k-1]
        ei += e*dt
        de = (e - e_prev)/dt
        d_f = der_alpha*d_f + (1.0 - der_alpha)*de
        e_prev = e
        uc = Kp*e + Ki*ei + Kd*d_f
        uc = float(np.clip(uc, -u_cap, u_cap))
        buf.append(uc)
        u_delayed = buf[-(d+1)]
        y[k] = a*y[k-1] + b*u_delayed
        u_cmd[k] = uc
    return y, u_cmd

def cost_iae_effort(t, r, y, u_cmd, effort_w=0.02):
    e = r - y
    iae = np.trapz(np.abs(e), t)
    ue2 = np.trapz(u_cmd*u_cmd, t)
    return iae + effort_w*ue2

def tune_pid(t, y_meas, a, b, d, sign_hint=None, effort_w=0.02, pi_only=False):
    # Build reference to match measured post-step level
    k0 = int(np.argmax(np.abs(np.diff(y_meas, prepend=y_meas[0]))))
    tail = max(int(0.80*len(y_meas)), k0+20)
    y_ss = float(np.median(y_meas[tail:])) if tail < len(y_meas) else float(y_meas[-1])
    r = np.zeros_like(t); r[k0:] = y_ss

    Kc, tau = arx_to_fopdt(a, b, np.median(np.diff(t)))
    Kmag = max(abs(Kc), 1e-6); tau_eff = max(tau, 1e-3)

    # Choose sign: positive if Kc>0, else negative (or force with --sign)
    sgn = -1.0 if (sign_hint == -1) else (1.0 if (sign_hint == 1) else (1.0 if Kc > 0 else -1.0))

    Kp_vals = sgn * np.geomspace(0.05/Kmag, 1.5/Kmag, 12)
    Ki_vals = np.geomspace(0.02/tau_eff, 1.5/tau_eff, 10)
    Kd_vals = [0.0] if pi_only else np.geomspace(0.02*tau_eff, 1.0*tau_eff, 8)

    best = (1e18, 0.0, 0.0, 0.0)
    for Kp in Kp_vals:
        for Ki in Ki_vals:
            for Kd in Kd_vals:
                ys, us = simulate_arx_pid(t, r, a, b, d, Kp, Ki, Kd)
                J = cost_iae_effort(t, r, ys, us, effort_w=effort_w)
                if J < best[0]:
                    best = (J, Kp, Ki, Kd)
    return {"Kp": best[1], "Ki": best[2], "Kd": best[3], "ref": r, "J": best[0], "Kc": Kc, "tau": tau, "theta": d*np.median(np.diff(t))}

def main():
    ap = argparse.ArgumentParser(description="Robust PID autotune for wall-follow from CSV.")
    ap.add_argument("--csv", required=True)
    ap.add_argument("--t-col", default="t_ms")
    ap.add_argument("--u-col", default="u")
    ap.add_argument("--y-col", default="e")
    ap.add_argument("--delay-max-s", type=float, default=1.5)
    ap.add_argument("--effort-w", type=float, default=0.02)
    ap.add_argument("--pi-only", action="store_true")
    ap.add_argument("--sign", type=int, choices=[-1,0,1], default=0, help="Force PID sign (+1/-1); 0=auto")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--out-prefix", default="")
    ap.add_argument("--mode", default="wall", help="Ignored; kept for CLI compatibility")  # <-- added
    args = ap.parse_args()

    t, u, y = load_series(args.csv, args.t_col, args.u_col, args.y_col)
    a, b, d, dt = fit_arx_first_order(t, u, y, delay_max_s=args.delay_max_s)
    res = tune_pid(t, y, a, b, d, sign_hint=args.sign, effort_w=args.effort_w, pi_only=args.pi_only)

    print("=== Identified discrete ARX(1) ===")
    print(f"a={a:.6f}  b={b:.6f}  delay_steps={d}  dt={dt:.6f} s")
    print("=== Continuous FOPDT approx ===")
    print(f"K={res['Kc']:.6f}  tau={res['tau']:.6f} s  theta≈{res['theta']:.6f} s")
    print("=== PID gains (parallel form) ===")
    print(f"Kp={res['Kp']:.6f}  Ki={res['Ki']:.6f}  Kd={res['Kd']:.6f}")
    print(f"Cost J={res['J']:.6f} (IAE + {args.effort_w}*∫u^2)")

    if args.plot:
        if plt is None:
            print("matplotlib not available; skipping plot.", file=sys.stderr)
        else:
            ys, us = simulate_arx_pid(t, res["ref"], a, b, d, res["Kp"], res["Ki"], res["Kd"])
            import os
            base = args.out_prefix if args.out_prefix else os.path.splitext(os.path.basename(args.csv))[0]
            png = f"{base}_autotune_preview.png"
            plt.figure(figsize=(9,5))
            plt.plot(t, y, label="measured e", linewidth=1.5)
            plt.plot(t, ys, "--", label="sim e (PID)", linewidth=1.5)
            plt.xlabel("time [s]"); plt.ylabel("error e"); plt.legend(); plt.grid(True, alpha=0.3)
            plt.tight_layout(); plt.savefig(png, dpi=140); plt.close()
            print(f"Saved preview: {png}")

    if args.out_prefix:
        with open(f"{args.out_prefix}_autotune_result.txt","w") as f:
            f.write(f"a={a:.6f} b={b:.6f} d={d} dt={dt:.6f}\n")
            f.write(f"K={res['Kc']:.6f} tau={res['tau']:.6f} theta={res['theta']:.6f}\n")
            f.write(f"Kp={res['Kp']:.6f} Ki={res['Ki']:.6f} Kd={res['Kd']:.6f}\n")

if __name__ == "__main__":
    main()



'''
Gyro:
Save serial to gyro_step.csv
python pid_autotune_from_step.py --csv gyro_step.csv --mode gyro

Wall:
Save serial to wall_step.csv
python pid_autotune_from_step.py --csv wall_step.csv --mode wall --u-col u --y-col emk



// Example usage in main.c after init:
run_gyro_turn_step_test(
    /*base_pwm=*/350,   /*delta_pwm=*/250,
    /*step_delay_ms=*/1000, /*step_duration_ms=*/2500,
    /*sample_ms=*/10,   /*total_ms=*/6000);

// Or for wall test (robot moving forward in corridor)
run_wall_lateral_step_test(
    /*base_pwm=*/500,   /*delta_pwm=*/150,
    /*step_delay_ms=*/1000, /*step_duration_ms=*/2000,
    /*sample_ms=*/10,   /*total_ms=*/6000);


'''