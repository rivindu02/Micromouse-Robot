/* logging_tests.c
 * Serial CSV logging helpers for step tests (gyro & encoder).
 *
 * Usage:
 * 1) Paste this file into Core/Src/.
 * 2) Add prototype in a header or call functions directly from main/debug menu:
 *      run_gyro_step_test(base_pwm, delta_pwm, step_delay_ms, step_duration_ms, sample_ms, total_ms);
 *      run_encoder_step_test(base_pwm, delta_pwm, step_delay_ms, step_duration_ms, sample_ms, total_ms);
 *
 * 3) Capture serial output (UART/BLE) with Tera Term / Serial Monitor and save as CSV.
 *
 * NOTE: Configure LOG_USE_HAL_UART and LOG_UART_HANDLE if you want to use HAL_UART_Transmit,
 *       otherwise the code will fall back to send_bluetooth_printf(...) if available.
 */

#include "main.h"      // HAL_GetTick
#include "micromouse.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* small wrapper to keep printf usage consistent */
static void log_printf(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    send_bluetooth_printf("%s", buf);
}
/* If your send_bluetooth_printf does NOT accept va_list directly,
   use this alternative implementation (uncomment and use):
static void log_printf(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    send_bluetooth_printf("%s", buf);
}
*/

/* small clamp helper */
static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static inline int clampi(int v, int lo, int hi){ return v < lo ? lo : (v > hi ? hi : v); }

/* extern your gyro PID gains if they live elsewhere */
extern float Kp_g, Ki_g, Kd_g;

/* =========================
   Gyro step test logger
   ========================= */
void run_gyro_step_test(int base_pwm, int delta_pwm, uint32_t step_delay_ms, uint32_t step_duration_ms, uint32_t sample_ms, uint32_t total_ms) {
    if (sample_ms == 0) sample_ms = 5;
    if (total_ms < step_delay_ms + step_duration_ms + 50) total_ms = step_delay_ms + step_duration_ms + 50;

    reset_encoder_totals();
    int left_pwm = base_pwm;
    int right_pwm = base_pwm;
    motor_set(0, true, (uint16_t)left_pwm);
    motor_set(1, true, (uint16_t)right_pwm);

    /* CSV header */
    log_printf("t_ms,left_pwm,right_pwm,gyro_z,left_counts,right_counts,left_rate,right_rate\r\n");

    uint32_t t0 = HAL_GetTick();
    uint32_t next_sample = t0;
    int32_t prev_left = get_left_encoder_total();
    int32_t prev_right = get_right_encoder_total();
    uint32_t start = t0;

    while ((HAL_GetTick() - t0) <= total_ms) {
        uint32_t now = HAL_GetTick();
        uint32_t t_rel = now - start;

        /* apply step */
        if (t_rel >= step_delay_ms && t_rel < (step_delay_ms + step_duration_ms)) {
            left_pwm  = (int)clampf((float)base_pwm - (float)delta_pwm, 0.0f, 1000.0f);
            right_pwm = (int)clampf((float)base_pwm + (float)delta_pwm, 0.0f, 1000.0f);
            motor_set(0, true, (uint16_t)left_pwm);
            motor_set(1, true, (uint16_t)right_pwm);
        } else {
            left_pwm  = base_pwm;
            right_pwm = base_pwm;
            motor_set(0, true, (uint16_t)left_pwm);
            motor_set(1, true, (uint16_t)right_pwm);
        }

        if (now >= next_sample) {
            int32_t left_total = get_left_encoder_total();
            int32_t right_total = get_right_encoder_total();

            float dt = (float)sample_ms / 1000.0f;
            float left_rate = (float)(left_total - prev_left) / dt;
            float right_rate = (float)(right_total - prev_right) / dt;
            mpu9250_read_gyro();

            float gyro_z = mpu9250_get_gyro_z_compensated();

            log_printf("%lu,%d,%d,%.3f,%ld,%ld,%.2f,%.2f\r\n",
                       (unsigned long)t_rel,
                       left_pwm, right_pwm,
                       gyro_z,
                       (long)left_total, (long)right_total,
                       left_rate, right_rate);

            prev_left = left_total;
            prev_right = right_total;
            next_sample += sample_ms;
        }
        HAL_Delay(1);
    }

    stop_motors();
    log_printf("#DONE: gyro step test finished\r\n");
}

/* =========================
   Encoder step test logger
   ========================= */
void run_encoder_step_test(int base_pwm, int delta_pwm, uint32_t step_delay_ms, uint32_t step_duration_ms, uint32_t sample_ms, uint32_t total_ms) {
    if (sample_ms == 0) sample_ms = 5;
    if (total_ms < step_delay_ms + step_duration_ms + 50) total_ms = step_delay_ms + step_duration_ms + 50;

    reset_encoder_totals();
    int left_pwm = base_pwm;
    int right_pwm = base_pwm;
    motor_set(0, true, (uint16_t)left_pwm);
    motor_set(1, true, (uint16_t)right_pwm);

    log_printf("t_ms,left_pwm,right_pwm,left_counts,right_counts,left_rate,right_rate\r\n");

    uint32_t t0 = HAL_GetTick();
    uint32_t next_sample = t0;
    int32_t prev_left = get_left_encoder_total();
    int32_t prev_right = get_right_encoder_total();
    uint32_t start = t0;

    while ((HAL_GetTick() - t0) <= total_ms) {
        uint32_t now = HAL_GetTick();
        uint32_t t_rel = now - start;

        /* apply step */
        if (t_rel >= step_delay_ms && t_rel < (step_delay_ms + step_duration_ms)) {
            left_pwm  = (int)clampf((float)base_pwm - (float)delta_pwm, 0.0f, 1000.0f);
            right_pwm = (int)clampf((float)base_pwm + (float)delta_pwm, 0.0f, 1000.0f);
            motor_set(0, true, (uint16_t)left_pwm);
            motor_set(1, true, (uint16_t)right_pwm);
        } else {
            left_pwm  = base_pwm;
            right_pwm = base_pwm;
            motor_set(0, true, (uint16_t)left_pwm);
            motor_set(1, true, (uint16_t)right_pwm);
        }

        if (now >= next_sample) {
            int32_t left_total = get_left_encoder_total();
            int32_t right_total = get_right_encoder_total();

            float dt = (float)sample_ms / 1000.0f;
            float left_rate = (float)(left_total - prev_left) / dt;
            float right_rate = (float)(right_total - prev_right) / dt;

            log_printf("%lu,%d,%d,%ld,%ld,%.2f,%.2f\r\n",
                       (unsigned long)t_rel,
                       left_pwm, right_pwm,
                       (long)left_total, (long)right_total,
                       left_rate, right_rate);

            prev_left = left_total;
            prev_right = right_total;
            next_sample += sample_ms;
        }
        HAL_Delay(1);
    }

    stop_motors();
    log_printf("#DONE: encoder step test finished\r\n");
}


void run_gyro_turn_step_test(int base_pwm, int delta_pwm,
                             uint32_t step_delay_ms, uint32_t step_duration_ms,
                             uint32_t sample_ms, uint32_t total_ms)
{
    send_bluetooth_printf("t_ms,Lpwm,Rpwm,gz,left_cnt,right_cnt\r\n");

    uint32_t t0 = HAL_GetTick();
    uint32_t next_sample = t0;
    start_encoders();

    while ((HAL_GetTick() - t0) <= total_ms) {
        uint32_t now = HAL_GetTick();
        uint32_t t_rel = now - t0;

        int lpwm = base_pwm;
        int rpwm = base_pwm;

        if (t_rel >= step_delay_ms && t_rel < (step_delay_ms + step_duration_ms)) {
            lpwm = clampf(base_pwm - delta_pwm, 0, 1000);
            rpwm = clampf(base_pwm + delta_pwm, 0, 1000);
        }

        mpu9250_read_gyro();
        // in-place turn: left backward, right forward
        motor_set(0, false, lpwm);
        motor_set(1, true,  rpwm);

        if (now >= next_sample) {
            float gz = mpu9250_get_gyro_z_compensated();
            int32_t lc = get_left_encoder_total();
            int32_t rc = get_right_encoder_total();
            send_bluetooth_printf("%lu,%d,%d,%.3f,%ld,%ld\r\n",
                                  (unsigned long)t_rel, lpwm, rpwm, gz,
                                  (long)lc, (long)rc);
            next_sample += sample_ms;
        }

        HAL_Delay(1);
    }

    stop_motors();
    send_bluetooth_printf("#DONE gyro step test\r\n");
}


// === Wall lateral step test: t_ms,u,e,L,R,FL,FR,Lpwm,Rpwm,left_cnt,right_cnt ===
void run_wall_lateral_step_test(int base_pwm, int delta_pwm,
                                uint32_t step_delay_ms, uint32_t step_duration_ms,
                                uint32_t sample_ms, uint32_t total_ms)
{
    send_bluetooth_printf("t_ms,u,e,L,R,FL,FR,Lpwm,Rpwm,left_cnt,right_cnt\r\n");

    uint32_t t0 = HAL_GetTick();
    uint32_t next_sample = t0;
    start_encoders();

    while ((HAL_GetTick() - t0) <= total_ms) {
        uint32_t now = HAL_GetTick();
        uint32_t t_rel = now - t0;

        int lpwm = base_pwm;
        int rpwm = base_pwm;

        // Inject a lateral step by skewing wheel PWMs
        if (t_rel >= step_delay_ms && t_rel < (step_delay_ms + step_duration_ms)) {
            lpwm = clampi(base_pwm - delta_pwm, 0, 1000);
            rpwm = clampi(base_pwm + delta_pwm, 0, 1000);
        }

        // Drive forward (both wheels forward)
        motor_set(0, true, (uint16_t)lpwm);
        motor_set(1, true, (uint16_t)rpwm);

        // Sample & log at fixed cadence
        if (now >= next_sample) {
            // Fresh sensors
            update_sensors();

            int L  = sensors.side_left;
            int R  = sensors.side_right;
            int FL = sensors.front_left;
            int FR = sensors.front_right;

            // Log-ratio lateral "output"
            float e = logf((float)L + 1.0f) - logf((float)R + 1.0f);

            // Normalized differential input (what we "commanded" to the plant)
            // ΔPWM = (R - L); normalize by full-scale 1000
            float u = ((float)rpwm - (float)lpwm) / 1000.0f;

            int32_t lc = get_left_encoder_total();
            int32_t rc = get_right_encoder_total();

            send_bluetooth_printf("%lu,%.6f,%.6f,%d,%d,%d,%d,%d,%d,%ld,%ld\r\n",
                                  (unsigned long)t_rel, u, e, L, R, FL, FR,
                                  lpwm, rpwm, (long)lc, (long)rc);

            next_sample += sample_ms;
        }

        HAL_Delay(1);
    }

    stop_motors();
    send_bluetooth_printf("#DONE wall step test\r\n");
}

// --- Single-wall (LEFT) lateral step test: t_ms,u,e,L,FL,FR,Lpwm,Rpwm,left_cnt,right_cnt ---
void run_wall_single_left_step_test(int base_pwm, int delta_pwm,
                                    uint32_t step_delay_ms, uint32_t step_duration_ms,
                                    uint32_t sample_ms, uint32_t total_ms)
{
    send_bluetooth_printf("t_ms,u,e,L,FL,FR,Lpwm,Rpwm,left_cnt,right_cnt\r\n");

    // --- Warm up & capture target over ~200 ms (20 samples @ 10 ms) ---
    float target_left = 0.0f;
    const int warm_N = 20;
    for (int i = 0; i < warm_N; i++) {
        update_sensors();
        target_left += (float)sensors.side_left;
        HAL_Delay(10);
    }
    target_left /= (float)warm_N;

    uint32_t t0 = HAL_GetTick();
    uint32_t next_sample = t0;
    start_encoders();

    while ((HAL_GetTick() - t0) <= total_ms) {
        uint32_t now  = HAL_GetTick();
        uint32_t t_rel = now - t0;

        int lpwm = base_pwm;
        int rpwm = base_pwm;

        // Inject lateral step by skewing wheel PWMs (forward motion)
        if (t_rel >= step_delay_ms && t_rel < (step_delay_ms + step_duration_ms)) {
            int d = (delta_pwm >= 0) ? delta_pwm : -delta_pwm;
            lpwm = base_pwm - d; if (lpwm < 0) lpwm = 0; if (lpwm > 1000) lpwm = 1000;
            rpwm = base_pwm + d; if (rpwm < 0) rpwm = 0; if (rpwm > 1000) rpwm = 1000;
        }

        motor_set(0, true, (uint16_t)lpwm);
        motor_set(1, true, (uint16_t)rpwm);

        // Sample & log at fixed cadence
        if (now >= next_sample) {
            update_sensors();
            int L  = sensors.side_left;
            int FL = sensors.front_left;
            int FR = sensors.front_right;

            // Single-wall error (left): reference to captured target at start
            float e = logf((float)L + 1.0f) - logf(target_left + 1.0f);

            // Normalized differential input (ΔPWM / 1000)
            float u = ((float)rpwm - (float)lpwm) / 1000.0f;

            int32_t lc = get_left_encoder_total();
            int32_t rc = get_right_encoder_total();

            send_bluetooth_printf("%lu,%.6f,%.6f,%d,%d,%d,%d,%d,%ld,%ld\r\n",
                                  (unsigned long)t_rel, u, e, L, FL, FR,
                                  lpwm, rpwm, (long)lc, (long)rc);

            next_sample += sample_ms;
        }

        HAL_Delay(1);
    }

    stop_motors();
    send_bluetooth_printf("#DONE wall single-left step test\r\n");
}






///* sign helper */
//static inline float sgnf(float x) { return (x >= 0.0f) ? 1.0f : -1.0f; }
//
///* Compute normalized wall error from sensors + mode.
//   BOTH  : e = (R - L) / (R + L)
//   LEFT  : e = (target - L) / target
//   RIGHT : e = (R - target) / target
//   Returns whether a wall reference is valid (so you know if wall controller is active). */
//static bool wall_error_normalized(WallFollowMode_t mode, float target_adc, float *e_out) {
//    float L = (float)sensors.side_left;
//    float R = (float)sensors.side_right;
//
//    switch (mode) {
//    case WALL_FOLLOW_BOTH:
//        if (sensors.wall_left && sensors.wall_right) {
//            float denom = fmaxf(L + R, 1e-3f);
//            *e_out = (R - L) / denom;
//            return true;
//        }
//        return false;
//
//    case WALL_FOLLOW_LEFT:
//        if (sensors.wall_left) {
//            float tgt = (target_adc > 1.0f) ? target_adc : fmaxf(L, 1.0f);
//            *e_out = (tgt - L) / tgt;
//            return true;
//        }
//        return false;
//
//    case WALL_FOLLOW_RIGHT:
//        if (sensors.wall_right) {
//            float tgt = (target_adc > 1.0f) ? target_adc : fmaxf(R, 1.0f);
//            *e_out = (R - tgt) / tgt;
//            return true;
//        }
//        return false;
//
//    default:
//        *e_out = 0.0f;
//        return false;
//    }
//}
//
///* Main logger:
//   - base_pwm: forward drive (e.g., 600–700)
//   - wall_mode: BOTH/LEFT/RIGHT
//   - Kp/Ki/Kd: wall PID gains
//   - deriv_alpha: 0..1 (e.g., 0.75..0.9)
//   - blend_ratio: 0..1  (1=gyro only, 0=wall only)
//   - sample_ms: log/control interval (e.g., 10)
//   - total_ms: duration
//   - relay_h: set >0 to use relay test (wall control u = +/- relay_h) instead of PID
//   - single_wall_target: ADC target for LEFT/RIGHT (0 = auto from first reading)
//*/
//void run_wall_follow_pid_test(int base_pwm,
//                              WallFollowMode_t wall_mode,
//                              float Kp_wall, float Ki_wall, float Kd_wall,
//                              float deriv_alpha, float blend_ratio,
//                              uint32_t sample_ms, uint32_t total_ms,
//                              int relay_h, float single_wall_target)
//{
//    if (sample_ms == 0) sample_ms = 10;
//
//    /* State */
//    float wall_err_prev = 0.0f;
//    float wall_int = 0.0f;
//    float wall_deriv_f = 0.0f;
//
//    float gyro_int = 0.0f;
//    float gyro_prev_err = 0.0f;
//
//    /* For single-wall target auto-capture */
//    float target_adc = single_wall_target;
//
//    /* CSV header (matches tuner script) */
//    log_printf("t_ms,side_left,side_right,wall_error,gyro_z,gyro_correction,wall_correction,total_correction,motor_left,motor_right,mode,relay_h\r\n");
//
//    /* Start */
//    reset_encoder_totals();
//    uint32_t t0 = HAL_GetTick();
//    uint32_t last = t0;
//    uint32_t next = t0;
//
//    int ml = base_pwm, mr = base_pwm;
//    motor_set(0, true, (uint16_t)ml);
//    motor_set(1, true, (uint16_t)mr);
//
//    while ((HAL_GetTick() - t0) <= total_ms) {
//        uint32_t now = HAL_GetTick();
//        if (now < next) { HAL_Delay(1); continue; }
//
//        float dt = (now - last) / 1000.0f;
//        if (dt <= 0.0f) dt = sample_ms / 1000.0f;
//        last = now;
//        next += sample_ms;
//
//        /* Update sensors and gyro */
//        update_sensors();
//        mpu9250_read_gyro();
//        float gz = mpu9250_get_gyro_z_compensated();
//
//        /* Wall error (normalized), with lazy target capture for single-wall */
//        float e_norm = 0.0f;
//        bool wall_active = wall_error_normalized(wall_mode, target_adc, &e_norm);
//
//        if ((wall_mode == WALL_FOLLOW_LEFT) && wall_active && target_adc <= 1.0f)
//            target_adc = (float)sensors.side_left;
//        if ((wall_mode == WALL_FOLLOW_RIGHT) && wall_active && target_adc <= 1.0f)
//            target_adc = (float)sensors.side_right;
//
//        /* --- Gyro straight PID (rate-based) --- */
//        float gyro_err = gz;  /* zero desired yaw rate while going straight */
//        gyro_int += gyro_err * dt;
//        gyro_int = fmaxf(-2000.0f, fminf(2000.0f, gyro_int));
//        float gyro_der = (gyro_err - gyro_prev_err) / dt;
//        float gyro_u = (Kp_g * gyro_err) + (Ki_g * gyro_int) + (Kd_g * gyro_der);
//        gyro_prev_err = gyro_err;
//
//        /* --- Wall control (PID or relay) --- */
//        float wall_u = 0.0f;
//        if (wall_active) {
//            if (relay_h > 0) {
//                wall_u = (float)relay_h * sgnf(e_norm);
//            } else {
//                float wall_der = (e_norm - wall_err_prev) / dt;
//                wall_deriv_f = deriv_alpha * wall_deriv_f + (1.0f - deriv_alpha) * wall_der;
//
//                /* tentative wall_u to test saturation */
//                float tentative = (Kp_wall * e_norm) + (Ki_wall * wall_int) + (Kd_wall * wall_deriv_f);
//
//                /* Saturation-aware integration (anti-windup) */
//                bool will_sat = false;
//                {
//                    int ml_t = base_pwm + (int)lroundf( (blend_ratio * gyro_u) + ((1.0f - blend_ratio) * tentative) );
//                    int mr_t = base_pwm - (int)lroundf( (blend_ratio * gyro_u) + ((1.0f - blend_ratio) * tentative) );
//                    if (ml_t < 0 || ml_t > 1000 || mr_t < 0 || mr_t > 1000) will_sat = true;
//                }
//                if (!will_sat) {
//                    wall_int += e_norm * dt;
//                    /* clip integral in a normalized range;  build Ki around this */
//                    wall_int = fmaxf(-1.5f, fminf(1.5f, wall_int));
//                }
//
//                wall_u = (Kp_wall * e_norm) + (Ki_wall * wall_int) + (Kd_wall * wall_deriv_f);
//                wall_err_prev = e_norm;
//            }
//        }
//
//        /* --- Blend fusion --- */
//        float total_u = wall_active ? (blend_ratio * gyro_u + (1.0f - blend_ratio) * wall_u)
//                                    : gyro_u;
//
//        /* Motor mix and clamp */
//        ml = base_pwm + (int)lroundf(total_u);
//        mr = base_pwm - (int)lroundf(total_u);
//        ml = (int)clampf((float)ml, 0.0f, 1000.0f);
//        mr = (int)clampf((float)mr, 0.0f, 1000.0f);
//
//        motor_set(0, true, (uint16_t)ml);
//        motor_set(1, true, (uint16_t)mr);
//
//        /* Log line (matches the Python tuner) */
//        log_printf("%lu,%u,%u,%.5f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d\r\n",
//                   (unsigned long)(now - t0),
//                   (unsigned int)sensors.side_left,
//                   (unsigned int)sensors.side_right,
//                   e_norm,           /* wall_error (normalized) */
//                   gz,               /* gyro_z */
//                   gyro_u,           /* gyro_correction */
//                   wall_u,           /* wall_correction (PID or relay) */
//                   total_u,          /* total_correction */
//                   ml, mr,
//                   (int)wall_mode,
//                   relay_h);
//    }
//
//    stop_motors();
//    log_printf("#DONE: wall-follow pid test finished\r\n");
//}
//
///* Convenience wrappers with sane defaults
//   - BOTH walls, safe-ish tuning start
//   - LEFT/RIGHT single-wall (reduced gains), auto target capture */
//void run_wall_follow_pid_test_both_default(int base_pwm, uint32_t sample_ms, uint32_t total_ms) {
//    run_wall_follow_pid_test(base_pwm, WALL_FOLLOW_BOTH,
//                             /*Kp*/0.18f, /*Ki*/0.00f, /*Kd*/0.025f,
//                             /*deriv_alpha*/0.80f, /*blend*/0.70f,
//                             sample_ms, total_ms,
//                             /*relay_h*/0, /*target_adc*/0.0f);
//}
//void run_wall_follow_pid_test_left_default(int base_pwm, uint32_t sample_ms, uint32_t total_ms) {
//    run_wall_follow_pid_test(base_pwm, WALL_FOLLOW_LEFT,
//                             /*Kp*/0.09f, /*Ki*/0.00f, /*Kd*/0.012f,
//                             0.80f, 0.70f,
//                             sample_ms, total_ms,
//                             0, 0.0f);
//}
//void run_wall_follow_pid_test_right_default(int base_pwm, uint32_t sample_ms, uint32_t total_ms) {
//    run_wall_follow_pid_test(base_pwm, WALL_FOLLOW_RIGHT,
//                             /*Kp*/0.09f, /*Ki*/0.00f, /*Kd*/0.012f,
//                             0.80f, 0.70f,
//                             sample_ms, total_ms,
//                             0, 0.0f);
//}
