#!/usr/bin/env python3
import argparse
import csv
import math
from statistics import median
from pathlib import Path

# ===== LUT copied from firmware (wall_handle.c) =====
# NOTE: Keep this in sync with your firmware.
R_LUT_ADC  = [43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11]
R_LUT_DIST = [1.17,1.23,1.29,1.36,1.43,1.51,1.59,1.67,1.76,1.85,1.95,2.05,2.16,2.28,2.40,2.53,2.67,2.82,2.98,3.16,3.34,3.55,3.76,4.00,4.26,4.54,4.85,5.19,5.57,5.98,6.45,6.96]
L_LUT_ADC  = [42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11]
L_LUT_DIST = [1.17,1.23,1.29,1.36,1.43,1.51,1.59,1.67,1.76,1.85,1.95,2.05,2.16,2.28,2.40,2.53,2.67,2.82,2.98,3.16,3.34,3.55,3.76,4.00,4.26,4.54,4.85,5.19,5.57,5.98,6.45,6.96]

def lut_lookup(raw, adc_table, dist_table):
    # Clamp below/above
    if raw >= adc_table[0]: return dist_table[0]
    if raw <= adc_table[-1]: return dist_table[-1]
    # Find segment
    for i in range(len(adc_table)-1):
        a_hi = adc_table[i]; a_lo = adc_table[i+1]
        if raw <= a_hi and raw >= a_lo:
            t = (raw - a_lo) / (a_hi - a_lo)
            dhi = dist_table[i]; dlo = dist_table[i+1]
            return dlo + t * (dhi - dlo)
    return dist_table[-1]

def load_series(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return rows

def build_signals(rows, mode, warm_ms=300):
    t=[]; u=[]; e_cm=[]; L_cm_list=[]; R_cm_list=[]

    # optional target capture for single-wall
    # compute target as average over warm_ms window
    if mode in ("left","right"):
        # measure average cm over initial warm_ms
        warm_t0 = float(rows[0]["t_ms"])
        acc = []; key = "L_raw" if mode=="left" else "R_raw"
        for r in rows:
            if float(r["t_ms"]) - warm_t0 > warm_ms: break
            try:
                acc.append(int(r[key]))
            except: pass
        if not acc:
            raise RuntimeError("No warm-up samples to set target.")
        target_raw = sum(acc)/len(acc)
        if mode=="left":
            target_cm = lut_lookup(target_raw, L_LUT_ADC, L_LUT_DIST)
        else:
            target_cm = lut_lookup(target_raw, R_LUT_ADC, R_LUT_DIST)
    else:
        target_cm = None

    for r in rows:
        try:
            t_ms = float(r["t_ms"])
            U = float(r["u"])
        except Exception:
            continue

        # Prefer explicit cm columns if firmware provided; else compute from raw
        if "L_cm" in r and r["L_cm"]!="":
            Lcm = float(r["L_cm"])
        else:
            Lraw = int(r.get("L_raw", "0"))
            Lcm = lut_lookup(Lraw, L_LUT_ADC, L_LUT_DIST)

        if "R_cm" in r and r["R_cm"]!="":
            Rcm = float(r["R_cm"])
        else:
            Rraw = int(r.get("R_raw", "0"))
            Rcm = lut_lookup(Rraw, R_LUT_ADC, R_LUT_DIST)

        if mode == "both":
            E = (Lcm - Rcm)  # cm difference
        elif mode == "left":
            E = (target_cm - Lcm)
        else: # right
            E = (Rcm - target_cm)

        t.append(t_ms); u.append(U); e_cm.append(E); L_cm_list.append(Lcm); R_cm_list.append(Rcm)

    return t,u,e_cm,L_cm_list,R_cm_list

def zero_crossings_up(t, x):
    tzc=[]
    for i in range(1, len(x)):
        if x[i-1] < 0.0 and x[i] >= 0.0:
            x0, x1 = x[i-1], x[i]
            t0, t1 = t[i-1], t[i]
            if x1 == x0:
                tc = t0
            else:
                frac = (-x0) / (x1 - x0)
                tc = t0 + frac * (t1 - t0)
            tzc.append(tc)
    return tzc

def osc_periods(tzc):
    return [ (tzc[i] - tzc[i-1]) / 1000.0 for i in range(1, len(tzc)) ]

def segment_peaks(t, x, tzc):
    As=[]
    # cycles delimited by up-crossings
    for i in range(1, len(tzc)):
        t_lo = tzc[i-1]; t_hi = tzc[i]
        # find indices spanning this window
        # simple linear search (data is small)
        idx_lo = min(range(len(t)), key=lambda k: abs(t[k]-t_lo))
        idx_hi = min(range(len(t)), key=lambda k: abs(t[k]-t_hi))
        if idx_hi <= idx_lo+1:
            continue
        seg = x[idx_lo:idx_hi+1]
        xmax = max(seg); xmin = min(seg)
        As.append(0.5*(xmax-xmin))
    return As

def compute_relay_gains(csv_path, mode="both", method="TL", drop_head_frac=0.2):
    rows = load_series(csv_path)
    if not rows:
        raise RuntimeError("Empty CSV.")

    t, u, e = [], [], []
    # Construct signals (and cm) from raw
    t,u,e,_,_ = build_signals(rows, mode)

    n = len(t)
    if n < 200:
        raise RuntimeError("Too few samples (<200). Collect a longer log.")

    # Drop initial transient
    cut = int(n * drop_head_frac)
    t = t[cut:]; u = u[cut:]; e = e[cut:]

    # Relay amplitude h
    absu = [abs(x) for x in u if abs(x) > 1e-4]
    if not absu:
        raise RuntimeError("No non-zero relay input detected in 'u' column.")
    h = median(absu)

    # Period and amplitude
    tzc = zero_crossings_up(t, e)
    if len(tzc) < 4:
        raise RuntimeError("Not enough oscillations detected. Increase total_ms or relay amplitude.")
    periods = osc_periods(tzc)
    Tu = median(periods)  # s

    As = segment_peaks(t, e, tzc)
    if not As:
        raise RuntimeError("Failed to measure oscillation amplitude.")
    A = median(As)

    # Describing function
    Ku = math.pi * A / (4.0 * h)

    method = method.upper()
    if method == "ZN":
        Kp = 0.60 * Ku
        Ti = 0.50 * Tu
        Td = 0.125 * Tu
    elif method == "TL":
        Kp = 0.454 * Ku
        Ti = 2.20 * Tu
        Td = 0.159 * Tu
    else:
        raise ValueError("Unknown method. Use 'ZN' or 'TL'.")

    Ki = Kp / Ti
    Kd = Kp * Td

    return {
        "samples": n, "trimmed_samples": len(t),
        "h_u_norm": h, "A_e_cm": A, "Tu_s": Tu, "Ku": Ku,
        "method": method, "mode": mode, "Kp": Kp, "Ki": Ki, "Kd": Kd
    }

def main():
    ap = argparse.ArgumentParser(description="Compute wall-follow PID gains from relay test CSV.")
    ap.add_argument("csv_path", type=Path, help="CSV from the robot (relay test)")
    ap.add_argument("--mode", choices=["both","left","right"], default="both", help="Wall-follow mode used in the test (default both)")
    ap.add_argument("--method", choices=["ZN", "TL"], default="TL", help="Tuning rule (default TL)")
    ap.add_argument("--drop", type=float, default=0.2, help="Fraction to drop from head as transient (default 0.2)")
    args = ap.parse_args()

    res = compute_relay_gains(args.csv_path, mode=args.mode, method=args.method, drop_head_frac=args.drop)

    print("# Relay test analysis")
    for k in ["samples","trimmed_samples","h_u_norm","A_e_cm","Tu_s","Ku","method","mode","Kp","Ki","Kd"]:
        print(f"{k}: {res[k]}")

    # sidecar JSON
    try:
        import json
        out = args.csv_path.with_suffix(f".pid_{args.method.lower()}_{args.mode}.json")
        with open(out, "w") as f:
            json.dump(res, f, indent=2)
        print(f"\nSaved: {out}")
    except Exception:
        pass

if __name__ == "__main__":
    main()
