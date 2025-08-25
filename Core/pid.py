# pid_tuner.py  -- improved robust version
# Usage: python pid_tuner.py <csv_file>
# Requires: numpy, optionally matplotlib

import sys
import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_PLOT = True
except Exception:
    HAS_PLOT = False

def load_csv(fname):
    with open(fname, 'r', encoding='utf-8') as f:
        header = f.readline().strip().split(',')
    data = np.loadtxt(fname, delimiter=',', skiprows=1)
    return header, data

def find_col(header, names):
    for n in names:
        for i,h in enumerate(header):
            if h.lower().strip() == n.lower().strip():
                return i
    return None

def detect_step(col_pwm):
    # detect first index where pwm deviates from initial plateau
    baseline = np.median(col_pwm[:max(3, len(col_pwm)//20)])
    diffs = np.abs(col_pwm - baseline)
    idxs = np.where(diffs > 1.0)[0]
    if idxs.size == 0:
        return None
    return idxs[0]

def steady_avg(arr, pct=0.15):
    n = len(arr)
    tail = max(1, int(n * pct))
    return np.mean(arr[-tail:])

def estimate_tau_theta(t, y, t_step, y0, y_inf):
    dy = y_inf - y0
    if abs(dy) < 1e-9:
        return None, None
    target63 = y0 + 0.632 * dy
    idx_after = np.where(t > t_step)[0]
    if idx_after.size == 0:
        return None, None
    ia = idx_after[0]
    # find first crossing after ia
    cross_idx = None
    for j in range(ia, len(y)):
        if (y[j] - target63) * dy >= 0:
            cross_idx = j
            break
    if cross_idx is None:
        tau = None
    else:
        tau = t[cross_idx] - t_step
    # estimate theta by 10% crossing
    target10 = y0 + 0.10 * dy
    cross10 = None
    for j in range(ia, len(y)):
        if (y[j] - target10) * dy >= 0:
            cross10 = j
            break
    theta = (t[cross10] - t_step) if cross10 is not None else 0.0
    return tau, theta

def imc_pi(Kp_proc, tau, theta, lambda_factor=0.5):
    lam = max(theta, lambda_factor * tau)
    if abs(Kp_proc) < 1e-12:
        Kp_proc = 1e-12
    Kc = tau / (Kp_proc * (lam + theta))
    Ti = min(tau, 4.0 * (lam + theta))
    Ki = Kc / Ti if Ti != 0 else 0.0
    return Kc, Ki, Ti, lam

def analyze_encoder(csv_file):
    header, data = load_csv(csv_file)
    # find columns
    t_idx = find_col(header, ['t_ms','time'])
    l_pw_idx = find_col(header, ['left_pwm','l_pwm','left_pwm'])
    r_pw_idx = find_col(header, ['right_pwm','r_pwm','right_pwm'])
    l_counts_idx = find_col(header, ['left_counts','l_counts','left_counts'])
    r_counts_idx = find_col(header, ['right_counts','r_counts','right_counts'])
    l_rate_idx = find_col(header, ['left_rate','l_rate','left_rate'])
    r_rate_idx = find_col(header, ['right_rate','r_rate','right_rate'])

    if t_idx is None or l_pw_idx is None or r_pw_idx is None:
        raise RuntimeError("CSV missing required pwm/time columns. Header: " + ",".join(header))

    t = data[:, t_idx] / 1000.0
    left_pwm = data[:, l_pw_idx]
    right_pwm = data[:, r_pw_idx]
    # rates: either available or compute from counts
    if l_rate_idx is not None and r_rate_idx is not None:
        left_rate = data[:, l_rate_idx]
        right_rate = data[:, r_rate_idx]
    else:
        if l_counts_idx is None or r_counts_idx is None:
            raise RuntimeError("CSV missing counts and rate columns; cannot compute rates.")
        left_counts = data[:, l_counts_idx]
        right_counts = data[:, r_counts_idx]
        dt = np.diff(t, prepend=t[0]+(t[1]-t[0]) if len(t)>1 else 0.01)
        # compute derivative safely
        left_rate = np.concatenate(([0.0], np.diff(left_counts) / dt[1:]))
        right_rate = np.concatenate(([0.0], np.diff(right_counts) / dt[1:]))

    # detect pwm change indices for left & right
    step_idx_left = detect_step(left_pwm)
    step_idx_right = detect_step(right_pwm)
    # try to detect global earliest step
    candidate_idxs = [i for i in [step_idx_left, step_idx_right] if i is not None]
    if len(candidate_idxs) == 0:
        raise RuntimeError("No PWM step detected in encoder CSV.")

    step_idx = min(candidate_idxs)
    t_step = t[step_idx]

    pre_l = np.median(left_pwm[:max(3, step_idx)])
    pre_r = np.median(right_pwm[:max(3, step_idx)])
    post_l = np.median(left_pwm[-max(3, len(left_pwm)//10):])
    post_r = np.median(right_pwm[-max(3, len(left_pwm)//10):])

    dL = post_l - pre_l
    dR = post_r - pre_r
    delta_diff = (dR - dL)

    # compute rate differences and per-wheel responses
    y_diff = left_rate - right_rate
    y0_diff = np.mean(y_diff[:max(3, step_idx)])
    yinf_diff = steady_avg(y_diff, pct=0.15)

    # per-wheel
    y0_L = np.mean(left_rate[:max(3, step_idx)])
    yinf_L = steady_avg(left_rate, pct=0.15)
    y0_R = np.mean(right_rate[:max(3, step_idx)])
    yinf_R = steady_avg(right_rate, pct=0.15)

    print("=== Encoder analysis ===")
    print("file:", csv_file)
    print(f"t_step = {t_step:.3f}s  (left_step_idx={step_idx_left}, right_step_idx={step_idx_right})")
    print(f"pre Lpw={pre_l:.1f}, pre Rpw={pre_r:.1f} ; post Lpw={post_l:.1f}, post Rpw={post_r:.1f}")
    print()

    # Case A: differential change available -> compute differential K and IMC PI for balance controller
    if abs(delta_diff) > 0.5:
        K_proc_diff = (yinf_diff - y0_diff) / delta_diff
        tau_diff, theta_diff = estimate_tau_theta(t, y_diff, t_step, y0_diff, yinf_diff)
        if tau_diff is None:
            tau_diff = max(0.01, (t[-1] - t_step)/4)
        Kc_diff, Ki_diff, Ti_diff, lam_diff = imc_pi(K_proc_diff, tau_diff, theta_diff)
        print("-> Differential response detected (useful for wheel-balance PID)")
        print(f"  delta_pwm_diff = (dR - dL) = {delta_diff:.3f}")
        print(f"  y0_diff = {y0_diff:.3f} counts/s, y_inf_diff = {yinf_diff:.3f} counts/s")
        print(f"  K_proc_diff = {K_proc_diff:.6f} (counts/s) per (PWM diff)")
        print(f"  tau = {tau_diff:.3f}s, theta = {theta_diff:.3f}s")
        print("  IMC PI for differential controller:")
        print(f"    Kp_diff = {Kc_diff:.6f}   (PWM per (count/s) )")
        print(f"    Ki_diff = {Ki_diff:.6f}   (PWM per (count/s) per s)")
        print()
    else:
        print("-> No clear differential PWM change detected (delta_diff ≈ 0).")

    # Case B: per-wheel individual gains (speed vs pwm)
    if abs(dL) > 0.5:
        Kp_proc_L = (yinf_L - y0_L) / dL
        tau_L, theta_L = estimate_tau_theta(t, left_rate, t_step, y0_L, yinf_L)
        if tau_L is None:
            tau_L = max(0.01, (t[-1]-t_step)/4)
        KcL, KiL, TiL, lamL = imc_pi(Kp_proc_L, tau_L, theta_L)
        print("-> Left wheel step detected:")
        print(f"  dL = {dL:.3f}, y0_L={y0_L:.3f}, yinf_L={yinf_L:.3f}")
        print(f"  K_proc_left = {Kp_proc_L:.6f} (counts/s per PWM)")
        print(f"  tau_L = {tau_L:.3f}s, theta_L = {theta_L:.3f}s")
        print(f"  IMC PI left: Kp_left = {KcL:.6f}, Ki_left = {KiL:.6f}, Ti_left = {TiL:.3f}")
        print()
    else:
        print("-> No left-wheel PWM step detected.")

    if abs(dR) > 0.5:
        Kp_proc_R = (yinf_R - y0_R) / dR
        tau_R, theta_R = estimate_tau_theta(t, right_rate, t_step, y0_R, yinf_R)
        if tau_R is None:
            tau_R = max(0.01, (t[-1]-t_step)/4)
        KcR, KiR, TiR, lamR = imc_pi(Kp_proc_R, tau_R, theta_R)
        print("-> Right wheel step detected:")
        print(f"  dR = {dR:.3f}, y0_R={y0_R:.3f}, yinf_R={yinf_R:.3f}")
        print(f"  K_proc_right = {Kp_proc_R:.6f} (counts/s per PWM)")
        print(f"  tau_R = {tau_R:.3f}s, theta_R = {theta_R:.3f}s")
        print(f"  IMC PI right: Kp_right = {KcR:.6f}, Ki_right = {KiR:.6f}, Ti_right = {TiR:.3f}")
        print()
    else:
        print("-> No right-wheel PWM step detected.")

    # Optional plots
    if HAS_PLOT:
        plt.figure(figsize=(8,4))
        plt.plot(t, left_rate, label='left_rate (counts/s)')
        plt.plot(t, right_rate, label='right_rate (counts/s)')
        plt.axvline(t_step, color='k', linestyle='--', label='step')
        plt.legend(); plt.grid(True); plt.xlabel('t (s)'); plt.ylabel('counts/s'); plt.title('Wheel rates')
        plt.show()

def analyze_gyro(csv_file):
    header, data = load_csv(csv_file)
    t_idx = find_col(header, ['t_ms','time'])
    l_pw_idx = find_col(header, ['left_pwm','l_pwm','left_pwm'])
    r_pw_idx = find_col(header, ['right_pwm','r_pwm','right_pwm'])
    gyro_idx = find_col(header, ['gyro_z','gyro','gz'])
    if t_idx is None or l_pw_idx is None or r_pw_idx is None or gyro_idx is None:
        raise RuntimeError("CSV missing required gyro columns.")
    t = data[:, t_idx] / 1000.0
    left_pwm = data[:, l_pw_idx]
    right_pwm = data[:, r_pw_idx]
    gyro_z = data[:, gyro_idx]

    step_idx = detect_step(left_pwm)
    if step_idx is None:
        step_idx = detect_step(right_pwm)
    if step_idx is None:
        raise RuntimeError("No PWM step detected in gyro CSV.")
    t_step = t[step_idx]
    pre_l = np.median(left_pwm[:max(3, step_idx)])
    pre_r = np.median(right_pwm[:max(3, step_idx)])
    post_l = np.median(left_pwm[-max(3, len(left_pwm)//10):])
    post_r = np.median(right_pwm[-max(3, len(left_pwm)//10):])
    dL = post_l - pre_l
    dR = post_r - pre_r
    delta_diff = dR - dL

    y0 = np.mean(gyro_z[:max(3, step_idx)])
    yinf = steady_avg(gyro_z, pct=0.15)
    K_proc = (yinf - y0) / (delta_diff if abs(delta_diff)>1e-9 else 1e-9)
    tau, theta = estimate_tau_theta(t, gyro_z, t_step, y0, yinf)
    if tau is None:
        tau = max(0.01, (t[-1]-t_step)/4)
    Kc, Ki, Ti, lam = imc_pi(K_proc, tau, theta)
    print("=== Gyro analysis ===")
    print(f"t_step={t_step:.3f}s post delta diff={delta_diff:.3f}")
    print(f"y0={y0:.3f} deg/s yinf={yinf:.3f} deg/s")
    print(f"K_proc (deg/s per PWM) = {K_proc:.6f}, tau={tau:.3f}, theta={theta:.3f}")
    print("IMC PI for gyro (rate controller):")
    print(f"  Kp_gyro = {Kc:.6f}  (PWM per deg/s)")
    print(f"  Ki_gyro = {Ki:.6f}")
    if HAS_PLOT:
        plt.figure(); plt.plot(t, gyro_z); plt.axvline(t_step); plt.grid(True); plt.show()

def main():
    if len(sys.argv) > 1:
        fname = sys.argv[1]
    else:
        fname = input("CSV filename (encoder or gyro): ").strip()
    header, _ = load_csv(fname)
    h = ",".join(header).lower()
    if 'gyro' in h or 'gyro_z' in h:
        analyze_gyro(fname)
    else:
        analyze_encoder(fname)

if __name__ == "__main__":
    main()
