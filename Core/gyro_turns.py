# gyro_turns_simple.py
# Robust, <50 lines. Works on your header: t_ms,left_pwm,right_pwm,gyro_z,...
import sys, numpy as np

def imc_pi(K, tau, theta=0.0, lam_factor=0.5):
    lam = max(theta, lam_factor * tau)
    Kp = tau / (K * (lam + theta))
    Ti = min(tau, 4.0 * (lam + theta))
    Ki = Kp / Ti if Ti > 1e-9 else 0.0
    return Kp, Ki, Ti, lam

def main(fname):
    # Read CSV with header names
    d = np.genfromtxt(fname, delimiter=',', names=True, dtype=None, encoding=None)
    t   = d['t_ms'] / 1000.0
    L   = d['left_pwm'].astype(float)
    R   = d['right_pwm'].astype(float)
    gz  = d['gyro_z'].astype(float)

    # Detect first PWM change
    L0, R0 = np.median(L[:5]), np.median(R[:5])
    step_idx = np.where((np.abs(L - L0) > 1) | (np.abs(R - R0) > 1))[0]
    if step_idx.size == 0:
        raise RuntimeError("No PWM step detected.")
    si = int(step_idx[0]); t_step = t[si]

    # Post-step plateau: window just after the step (handles files that later revert)
    dt = np.mean(np.diff(t)) if len(t) > 1 else 0.01
    w  = max(3, int(0.6 / dt))  # 0.6 s window
    L1 = float(np.median(L[si+1:si+1+w])); R1 = float(np.median(R[si+1:si+1+w]))
    dL, dR = L1 - L0, R1 - R0
    dPWM = dR - dL
    if abs(dPWM) < 1e-6:
        raise RuntimeError("Step found but (dR-dL)≈0. Use a persistent differential step.")

    y0   = float(np.mean(gz[:si]))
    yinf = float(np.mean(gz[-max(5, len(gz)//7):]))  # last ~15%
    K    = (yinf - y0) / dPWM

    # Tau from 63% crossing (fallback if not reached)
    target = y0 + 0.632 * (yinf - y0)
    ia = si + 1
    hit = np.where((gz[ia:] - target) * (yinf - y0) >= 0)[0]
    tau = (t[ia + hit[0]] - t_step) if hit.size else max(0.01, (t[-1]-t_step)/4.0)

    # Optional quick theta from 10% crossing
    target10 = y0 + 0.10 * (yinf - y0)
    hit10 = np.where((gz[ia:] - target10) * (yinf - y0) >= 0)[0]
    theta = (t[ia + hit10[0]] - t_step) if hit10.size else 0.0

    Kp, Ki, Ti, lam = imc_pi(K, tau, theta)

    print("t_step=%.3f s  ΔPWM(diff)=%.1f" % (t_step, dPWM))
    print("y0=%.3f  y_inf=%.3f  K_proc=%.6f  tau=%.3f  theta=%.3f" % (y0, yinf, K, tau, theta))
    print("\nIMC PI (rate loop):")
    print("  Kp_g = %.6f   (PWM per deg/s)" % Kp)
    print("  Ki_g = %.6f   (PWM per deg/s per s)" % Ki)
    print("  Ti   = %.3f s   lambda=%.3f" % (Ti, lam))

if __name__ == "__main__":
    fname = sys.argv[1] if len(sys.argv) > 1 else input("CSV filename: ").strip()
    main(fname)
