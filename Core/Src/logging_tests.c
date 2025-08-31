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

/* =========================
   Gyro step test logger
   ========================= */
void run_gyro_step_test(int base_pwm, int delta_pwm, uint32_t step_delay_ms, uint32_t step_duration_ms, uint32_t sample_ms, uint32_t total_ms) {
    if (sample_ms == 0) sample_ms = 5;
    if (total_ms < step_delay_ms + step_duration_ms + 50) total_ms = step_delay_ms + step_duration_ms + 50;

    reset_encoder_totals();
    int left_pwm = base_pwm;
    int right_pwm = base_pwm;
    motor_set_fixed(0, true, (uint16_t)left_pwm);
    motor_set_fixed(1, true, (uint16_t)right_pwm);

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
            motor_set_fixed(0, true, (uint16_t)left_pwm);
            motor_set_fixed(1, true, (uint16_t)right_pwm);
        } else {
            left_pwm  = base_pwm;
            right_pwm = base_pwm;
            motor_set_fixed(0, true, (uint16_t)left_pwm);
            motor_set_fixed(1, true, (uint16_t)right_pwm);
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
    motor_set_fixed(0, true, (uint16_t)left_pwm);
    motor_set_fixed(1, true, (uint16_t)right_pwm);

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
            motor_set_fixed(0, true, (uint16_t)left_pwm);
            motor_set_fixed(1, true, (uint16_t)right_pwm);
        } else {
            left_pwm  = base_pwm;
            right_pwm = base_pwm;
            motor_set_fixed(0, true, (uint16_t)left_pwm);
            motor_set_fixed(1, true, (uint16_t)right_pwm);
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
        motor_set_fixed(0, false, lpwm);
        motor_set_fixed(1, true,  rpwm);

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
