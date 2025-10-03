#!/usr/bin/env python3
"""
Wall-follow PID tuner from CSV logs (Micromouse / IR-centering + gyro fusion)
-----------------------------------------------------------------------------
This script estimates PID constants for the *wall-follow* loop. It supports:
  • Tyreus–Luyben (TL) and Ziegler–Nichols (ZN) from sustained oscillations
  • Relay test method (if you logged a relay amplitude column 'relay_h')
  • IMC (model-based) if you identified a simple FOPDT model (K, tau, theta)

Expected CSV columns (names are case-insensitive; the script is flexible):
  t_ms                : timestamp in milliseconds  (or 'time' in seconds)
  side_left, side_right : raw/filtered ADC readings of side sensors
  wall_error          : (optional) precomputed wall error (use normalized if you can)
  gyro_z              : (optional) gyro z (deg/s) for reference plots
  gyro_correction     : (optional) gyro PID output
  wall_correction     : (optional) wall PID output (or relay output)
  total_correction    : combined output sent to motors
  motor_left, motor_right : motor PWMs
  mode                : 0=NONE,1=LEFT,2=RIGHT,3=BOTH (optional)
  relay_h             : (optional) relay amplitude used during relay test

Typical usage
-------------
# Safer, less aggressive tuning (recommended)
python wall_follow_pid_tuner.py wall_log.csv --method tl --norm both

# If you measured ultimate gain Ku and period Pu yourself
python wall_follow_pid_tuner.py wall_log.csv --method tl --ku 0.85 --pu 0.42

# IMC after identifying process gain/time-constant
python wall_follow_pid_tuner.py wall_log.csv --method imc --K 1.7 --tau 0.25 --lambda 0.6

# Headless run (skip plots) and print a C snippet for your firmware
python wall_follow_pid_tuner.py wall_log.csv --method tl --no-plot --c-snippet

Notes
-----
• If 'wall_error' is NOT in your CSV, the script builds a normalized error:
    BOTH : e = (R - L) / (R + L)
    LEFT : e = (target - L) / target     (target inferred from median)
    RIGHT: e = (R - target) / target     (target inferred from median)
• For ZN/TL, you either provide Ku,Pu OR the script will try to detect Pu from
  sustained oscillations. Ku can be derived automatically if you did a relay test
  and logged 'relay_h' (Ku = 4h/(pi*A), where A is the oscillation amplitude).

"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from typing import Tuple, Optional, Dict

# ----------------------
# CSV Loading & Utilities
# ----------------------

def load_csv(path: str) -> Tuple[pd.DataFrame, np.ndarray]:
    df = pd.read_csv(path)
    df.columns = [c.strip().lower() for c in df.columns]
    if 't_ms' in df.columns:
        t = df['t_ms'].astype(float).to_numpy() / 1000.0
    elif 'time' in df.columns:
        t = df['time'].astype(float).to_numpy()
    else:
        # fallback to a 10 ms sample assumption
        t = np.arange(len(df)) * 0.01
    return df, t


def smooth(x: np.ndarray, win: int = 9) -> np.ndarray:
    if win <= 1:
        return x.copy()
    win = int(win)
    if win % 2 == 0:
        win += 1
    pad = win // 2
    xpad = np.pad(x, (pad, pad), mode='edge')
    kernel = np.ones(win) / win
    return np.convolve(xpad, kernel, mode='valid')


# ----------------------
# Error construction
# ----------------------

def compute_wall_error(df: pd.DataFrame, mode_hint: str = 'both') -> np.ndarray:
    cols = set(df.columns)
    if 'wall_error' in cols:
        return df['wall_error'].astype(float).to_numpy()

    # Build from left/right
    if 'side_left' not in cols or 'side_right' not in cols:
        return np.zeros(len(df))

    L = df['side_left'].astype(float).to_numpy()
    R = df['side_right'].astype(float).to_numpy()

    if mode_hint == 'both':
        denom = np.clip(R + L, 1e-3, None)
        e = (R - L) / denom
    elif mode_hint == 'left':
        target = np.median(L[L > 0]) if np.any(L > 0) else max(np.median(L), 1.0)
        denom = max(target, 1e-3)
        e = (target - L) / denom
    elif mode_hint == 'right':
        target = np.median(R[R > 0]) if np.any(R > 0) else max(np.median(R), 1.0)
        denom = max(target, 1e-3)
        e = (R - target) / denom
    else:
        e = np.zeros(len(df))
    return e


# ----------------------
# Oscillation detection & Tuning rules
# ----------------------

def detect_oscillation_params(df: pd.DataFrame, t: np.ndarray, e: np.ndarray,
                              min_cycles: int = 5) -> Tuple[Optional[float], Optional[float], Dict]:
    """Estimate Pu from sustained oscillations and Ku from relay test if present.
    Returns (Ku, Pu, info). Ku may be None if no relay_h column.
    """
    y = smooth(e, win=9)
    # zero-crossings
    zc = np.where(np.diff(np.signbit(y)))[0]
    info = {}
    if len(zc) < (2 * min_cycles + 1):
        info['reason'] = 'not enough zero crossings for sustained oscillations'
        return None, None, info

    # full-cycle periods (every two zero-crossings)
    periods = []
    for i in range(len(zc) - 2):
        t0 = t[zc[i]]
        t2 = t[zc[i + 2]]
        periods.append(t2 - t0)
    periods = np.array(periods)
    periods = periods[periods > 0]
    if periods.size == 0:
        info['reason'] = 'no positive periods'
        return None, None, info
    Pu = float(np.median(periods))

    # oscillation amplitude (robust)
    seg = y[zc[1]:zc[-1]]
    if seg.size < 10:
        info['reason'] = 'oscillation segment too short'
        return None, Pu, info
    A = 0.5 * (np.percentile(seg, 95) - np.percentile(seg, 5))
    info['A'] = float(A)

    Ku = None
    if 'relay_h' in df.columns:
        hvals = df['relay_h'].astype(float).to_numpy()
        hv = hvals[hvals != 0]
        if hv.size > 0 and A > 0:
            h = float(np.median(hv))
            Ku = 4.0 * h / (np.pi * A)
            info['relay_h_used'] = h
    return Ku, Pu, info


def zn_pid(Ku: float, Pu: float) -> Tuple[float, float, float]:
    """Ziegler–Nichols classic PID -> (Kp, Ki, Kd) in parallel form"""
    Kp = 0.60 * Ku
    Ti = Pu / 2.0
    Td = Pu / 8.0
    Ki = Kp / Ti
    Kd = Kp * Td
    return Kp, Ki, Kd


def tl_pid(Ku: float, Pu: float) -> Tuple[float, float, float]:
    """Tyreus–Luyben (safer for noisy plants) -> (Kp, Ki, Kd)"""
    Kp = 0.45 * Ku
    Ti = 2.2 * Pu
    Td = Pu / 6.3
    Ki = Kp / Ti
    Kd = Kp * Td
    return Kp, Ki, Kd


def imc_pid(K: float, tau: float, lambd: float, theta: float = 0.0) -> Tuple[float, float, float]:
    """IMC-based PID for FOPDT -> (Kp, Ki, Kd)"""
    Kc = (tau) / (K * (lambd + max(theta, 1e-6)))
    Ti = tau + theta / 2.0
    Td = (tau * theta) / (2.0 * (tau + max(theta, 1e-6)))
    Kp = Kc
    Ki = Kc / Ti
    Kd = Kc * Td
    return Kp, Ki, Kd


# ----------------------
# Pretty print helpers
# ----------------------

def print_results(tag: str, Kp: float, Ki: float, Kd: float, c_snippet: bool = False):
    print(f"[{tag}] Recommended PID gains:")
    print(f"  Kp_wall = {Kp:.6f}")
    print(f"  Ki_wall = {Ki:.6f}")
    print(f"  Kd_wall = {Kd:.6f}")
    if c_snippet:
        print("\n// C snippet (parallel form)\n"
              f"static float Kp_wall = {Kp:.6f}f;\n"
              f"static float Ki_wall = {Ki:.6f}f;\n"
              f"static float Kd_wall = {Kd:.6f}f;\n")


# ----------------------
# Main
# ----------------------

def main():
    ap = argparse.ArgumentParser(description='Wall-follow PID tuner from CSV')
    ap.add_argument('csv', help='Path to log CSV')
    ap.add_argument('--norm', choices=['both', 'left', 'right', 'none'], default='both',
                    help='How to build wall error if not provided in CSV')
    ap.add_argument('--method', choices=['tl', 'zn', 'imc'], default='tl',
                    help='Tuning method (TL recommended)')
    ap.add_argument('--ku', type=float, default=None, help='Ultimate gain Ku (optional)')
    ap.add_argument('--pu', type=float, default=None, help='Ultimate period Pu in seconds (optional)')
    ap.add_argument('--K', dest='Kproc', type=float, default=None, help='Process gain K (IMC)')
    ap.add_argument('--tau', type=float, default=None, help='Process time constant τ (IMC)')
    ap.add_argument('--theta', type=float, default=0.0, help='Deadtime θ (IMC)')
    ap.add_argument('--lambda', dest='lambd', type=float, default=None, help='Closed-loop speed λ (IMC)')
    ap.add_argument('--no-plot', action='store_true', help='Disable plots')
    ap.add_argument('--c-snippet', action='store_true', help='Also print a C snippet of the gains')

    args = ap.parse_args()

    df, t = load_csv(args.csv)
    e = compute_wall_error(df, mode_hint=args.norm)
    e_s = smooth(e, win=9)

    if not args.no-plot:
        plt.figure()
        plt.plot(t, e_s, label='wall_error (smoothed)')
        if 'total_correction' in df.columns:
            plt.plot(t, df['total_correction'].astype(float).to_numpy(), label='total_correction')
        plt.xlabel('Time [s]')
        plt.ylabel('Normalized error / correction')
        plt.legend()
        plt.title('Wall-follow traces')
        plt.tight_layout()

    method = args.method.lower()
    if method in ('zn', 'tl'):
        Ku, Pu = args.ku, args.pu
        info = {}
        if Pu is None:
            Ku_est, Pu_est, info = detect_oscillation_params(df, t, e_s)
            if Pu is None and Pu_est is not None:
                Pu = Pu_est
            if Ku is None and Ku_est is not None:
                Ku = Ku_est
        if Pu is None:
            print('Could not determine Pu. Provide --pu (and optionally --ku) or try --method imc.')
            if info: print('Info:', info)
            return
        if Ku is None:
            print('Ultimate gain Ku is unknown. Provide --ku or use a relay test with relay_h logged.')
            if info: print('Info:', info)
            return
        if method == 'zn':
            Kp, Ki, Kd = zn_pid(Ku, Pu)
            print_results('ZN', Kp, Ki, Kd, c_snippet=args.c_snippet)
        else:
            Kp, Ki, Kd = tl_pid(Ku, Pu)
            print_results('TL', Kp, Ki, Kd, c_snippet=args.c_snippet)

    elif method == 'imc':
        if args.Kproc is None or args.tau is None or args.lambd is None:
            print('IMC requires --K, --tau and --lambda.')
            return
        Kp, Ki, Kd = imc_pid(args.Kproc, args.tau, args.lambd, theta=args.theta)
        print_results('IMC', Kp, Ki, Kd, c_snippet=args.c_snippet)

    else:
        raise ValueError('Unknown method')

    if not args.no-plot:
        plt.show()


if __name__ == '__main__':
    main()
