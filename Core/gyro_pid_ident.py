#!/usr/bin/env python3
"""
Gyro rate-loop PID identification from step-test CSV logs.

Input CSV format (as produced by your run_gyro_step_test):
    t_ms,left_pwm,right_pwm,gyro_z,left_counts,right_counts,left_rate,right_rate

What it does, per CSV:
  1) Derives the differential command u(t) = (right_pwm - left_pwm)/2  [PWM units].
  2) Detects the step window automatically.
  3) Estimates:
        K     [deg/s per PWM]  : DC gain = steady_rate / step_amplitude
        tau   [s]               : time constant via 63.2% rise method
        theta [s]               : apparent dead time (5% threshold)
  4) Computes PID gains for the gyro *rate* loop using SIMC-style tuning:
        Kp = tau / (K*(lambda + theta))
        Ki = Kp / tau
        Kd = Kp * (Td_frac * tau)

Also computes a recommended derivative low-pass alpha for your discrete filter:
    Tf = (Td_frac * tau) / N,  alpha = dt / (Tf + dt)  with N in [8..12]

Usage example:
    python gyro_pid_ident.py file1.csv file2.csv file3.csv file4.csv \
        --lambda 0.5 --td-frac 0.15 --N 10 --smooth 5 --tail-sec 0.5
"""

import argparse
import math
import statistics
from dataclasses import dataclass
from typing import List, Tuple, Optional

import pandas as pd
import numpy as np


@dataclass
class StepFit:
    file: str
    K: float
    tau: float
    theta: float
    U: float
    omega_ss: float
    dt_med: float
    t_step_start: float
    t_step_end: float
    ok: bool
    note: str = ""


def moving_average(x: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return x
    c = np.cumsum(np.insert(x, 0, 0.0))
    y = (c[window:] - c[:-window]) / float(window)
    pad = np.full(window-1, y[0] if len(y) > 0 else (x[0] if len(x) else 0.0))
    return np.concatenate([pad, y]) if len(y) else x


def detect_step(u: np.ndarray, t: np.ndarray) -> Tuple[int, int]:
    """
    Detect the main step interval in u(t).
    Strategy:
      - Baseline u0 = median of first 10% of samples.
      - Largest contiguous region where |u - u0| exceeds 30% of max deviation.
      - Return (start_idx, end_idx) of that region.
    """
    n = len(u)
    if n < 10:
        return 0, n-1
    i0 = int(0.1 * n)
    u0 = np.median(u[:max(1, i0)])
    du = u - u0
    max_dev = np.max(np.abs(du)) if np.any(np.isfinite(du)) else 0.0
    if max_dev < 1e-6:
        return 0, n-1
    thresh = 0.3 * max_dev
    mask = np.abs(du) >= thresh

    best_len = 0
    best_start = 0
    cur_start = None
    for i, m in enumerate(mask):
        if m and cur_start is None:
            cur_start = i
        elif (not m) and cur_start is not None:
            L = i - cur_start
            if L > best_len:
                best_len = L
                best_start = cur_start
            cur_start = None
    if cur_start is not None:
        L = n - cur_start
        if L > best_len:
            best_len = L
            best_start = cur_start
    start = best_start
    end = min(n-1, start + best_len - 1)
    return start, end


def estimate_first_order_params(t: np.ndarray,
                                u: np.ndarray,
                                y: np.ndarray,
                                smooth_win: int = 5,
                                tail_sec: float = 0.5) -> Tuple[StepFit, Optional[str]]:
    """
    Given time (s), input u (PWM), output y (deg/s), estimate K, tau, theta.
    """
    if len(t) < 5:
        return StepFit("", float("nan"), float("nan"), float("nan"),
                       float("nan"), float("nan"), float("nan"), 0, 0, False), "too few samples"

    y_s = moving_average(y, max(1, smooth_win))

    i_start, i_end = detect_step(u, t)
    if i_end - i_start < 5:
        return StepFit("", float("nan"), float("nan"), float("nan"),
                       float("nan"), float("nan"), float("nan"), 0, 0, False), "step not detected"

    t_start = t[i_start]
    t_end = t[i_end]
    dt_med = np.median(np.diff(t))

    # Input amplitude (signed) using medians
    u0 = np.median(u[max(0, i_start - int(0.1*len(u))):i_start]) if i_start > 5 else np.median(u[:i_start+1])
    u1 = np.median(u[i_start:i_end+1])
    U = u1 - u0

    # Output baseline and steady-state
    y0 = np.median(y_s[max(0, i_start - int(0.1*len(y_s))):i_start]) if i_start > 5 else np.median(y_s[:i_start+1])
    tail_start_time = max(t_start, t_end - max(tail_sec, 3*dt_med))
    tail_mask = (t >= tail_start_time) & (t <= t_end)
    if not np.any(tail_mask):
        tail_mask = np.arange(i_end-3, i_end+1) >= 0
    y_ss = float(np.mean(y_s[tail_mask]))

    dy = y_ss - y0
    if abs(U) < 1e-6 or abs(dy) < 1e-6:
        return StepFit("", float("nan"), float("nan"), float("nan"),
                       float("nan"), float("nan"), float("nan"), 0, 0, False), "zero step or response"

    K = dy / U  # deg/s per PWM

    # Time constant via 63.2% rise
    y_target = y0 + 0.6321205588 * dy
    dead_thresh = y0 + 0.05 * dy

    idx_after = np.where(t >= t_start)[0]
    t_tau = None
    t_theta = None
    for i in idx_after:
        if t_theta is None and ((dy > 0 and y_s[i] >= dead_thresh) or (dy < 0 and y_s[i] <= dead_thresh)):
            t_theta = t[i]
        if t_tau is None and ((dy > 0 and y_s[i] >= y_target) or (dy < 0 and y_s[i] <= y_target)):
            t_tau = t[i]
            break
    if t_theta is None:
        t_theta = t_start
    if t_tau is None:
        return StepFit("", float("nan"), float("nan"), float("nan"),
                       float("nan"), float("nan"), dt_med, t_start, t_end, False), "tau not reached"

    theta = max(0.0, float(t_theta - t_start))
    tau = max(dt_med, float(t_tau - t_theta))

    fit = StepFit("", float(K), float(tau), float(theta), float(abs(U)), float(y_ss), float(dt_med),
                  float(t_start), float(t_end), True)
    return fit, None


def process_file(path: str, smooth_win: int, tail_sec: float) -> StepFit:
    df = pd.read_csv(path)
    required = ["t_ms", "left_pwm", "right_pwm", "gyro_z"]
    for c in required:
        if c not in df.columns:
            raise ValueError(f"{path}: missing column '{c}'")
    t = df["t_ms"].to_numpy(dtype=float) * 1e-3
    u = (df["right_pwm"].to_numpy(dtype=float) - df["left_pwm"].to_numpy(dtype=float)) * 0.5
    y = df["gyro_z"].to_numpy(dtype=float)

    fit, err = estimate_first_order_params(t, u, y, smooth_win=smooth_win, tail_sec=tail_sec)
    fit.file = path
    if not fit.ok and err:
        fit.note = err
    return fit


def median_or_nan(vals: List[float]) -> float:
    vals = [v for v in vals if np.isfinite(v)]
    return float(np.median(vals)) if vals else float("nan")


def main():
    ap = argparse.ArgumentParser(description="Identify K, tau, theta from gyro step-test CSVs and compute PID gains.")
    ap.add_argument("csvs", nargs="+", help="CSV files from run_gyro_step_test")
    ap.add_argument("--lambda", dest="lam", type=float, default=0.5, help="Desired closed-loop time constant (s) for rate loop (SIMC). Default 0.5")
    ap.add_argument("--td-frac", dest="td_frac", type=float, default=0.15, help="Derivative time as fraction of tau (0.1..0.25). Default 0.15")
    ap.add_argument("--N", dest="N", type=float, default=10.0, help="Derivative filter ratio (Tf = Td/N). Default 10")
    ap.add_argument("--smooth", type=int, default=5, help="Moving-average window (samples) for gyro_z smoothing. Default 5")
    ap.add_argument("--tail-sec", type=float, default=0.5, help="Seconds at end of step used for steady-state averaging. Default 0.5")
    args = ap.parse_args()

    fits: List[StepFit] = []
    for p in args.csvs:
        try:
            fit = process_file(p, smooth_win=args.smooth, tail_sec=args.tail_sec)
            fits.append(fit)
        except Exception as e:
            print(f"[ERROR] {p}: {e}")
            continue

    if not fits:
        print("No valid files processed.")
        return

    print("# Per-file identification results")
    print("file,K[deg/s per PWM],tau[s],theta[s],U[PWM],omega_ss[deg/s],dt_med[s],ok,note")
    for f in fits:
        print(f"{f.file},{f.K:.6g},{f.tau:.6g},{f.theta:.6g},{f.U:.6g},{f.omega_ss:.6g},{f.dt_med:.6g},{int(f.ok)},{f.note}")

    ok_fits = [f for f in fits if f.ok]
    if not ok_fits:
        print("\nNo valid fits to aggregate. Tune manually from per-file rows above.")
        return

    K_med   = median_or_nan([f.K for f in ok_fits])
    tau_med = median_or_nan([f.tau for f in ok_fits])
    theta_med = median_or_nan([f.theta for f in ok_fits])
    dt_med  = median_or_nan([f.dt_med for f in ok_fits])

    lam = max(dt_med * 10.0, args.lam)  # ensure not too fast for sampling
    Td  = args.td_frac * tau_med

    # Ideal PID form in your code: u = Kp*e + Ki*∫e dt + Kd*de/dt
    Kp = tau_med / (K_med * (lam + theta_med))
    Ki = Kp / tau_med
    Kd = Kp * Td

    Tf = Td / max(1e-6, args.N)
    alpha = dt_med / (Tf + dt_med) if Tf > 0 else 1.0

    print("\n# Recommended controller gains (ideal PID form: u = Kp*e + Ki*∫e dt + Kd*de/dt)")
    print(f"Kp_g = {Kp:.6g}")
    print(f"Ki_g = {Ki:.6g}")
    print(f"Kd_g = {Kd:.6g}")
    print(f"DERIV_FILTER_ALPHA ≈ {alpha:.6g}    # computed from dt_med={dt_med:.4g}s, Td={Td:.4g}s, N={args.N}")

    print("\n# Aggregate plant (median over OK runs)")
    print(f"K      = {K_med:.6g}  [deg/s per PWM]")
    print(f"tau    = {tau_med:.6g}  [s]")
    print(f"theta  = {theta_med:.6g}  [s]")
    print(f"dt_med = {dt_med:.6g}  [s]")
    print(f"lambda = {lam:.6g}  [s]  # used for tuning")
    print(f"Td     = {Td:.6g}  [s]   # derivative time (Td_frac * tau)")

    print("\n# Notes")
    print("- Ensure no saturation during the step window (your logger clamps 0..1000).")
    print("- If fits disagree, inspect CSVs; slips/bumps corrupt identification.")
    print("- Start with PI if noise is high; add Kd only if you see overshoot/oscillation.")
    print("- Keep runtime guards: deadband, leak, cooldown, anti-windup.")
    print("- If K varies with base speed, choose the safer (lower) Kp from the higher K value.")
    

if __name__ == "__main__":
    main()
