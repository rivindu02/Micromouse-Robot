/*
 * turns.c
 *
 *  Created on: Oct 3, 2025
 *      Author: ASUS
 */


#include "micromouse.h"
#include "movement.h"


//// ---------- If your project already defines these, delete the guards ----------
//#ifndef CLAMPI_LOCAL_DEFINED
//#define CLAMPI_LOCAL_DEFINED
//static inline int clampi_local(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
//#endif
//
//#ifndef TARGET_ALIGN_DEFINED
//#define TARGET_ALIGN_DEFINED
//#define target_align 90   // front sensor ADC target (counts) at desired distance
//#endif
//// --- Front-wall alignment (PI + PI)---
//
//
//// ----- Deadzone model per wheel -----
//typedef struct {
//    int min_fwd;           // duty where motion starts (forward)
//    int min_rev;           // duty where motion starts (reverse)
//    uint8_t was_zero;      // last cycle command was zero
//    uint32_t kick_until;   // kick active until timestamp
//} WheelDZ;
//
//// Map controller command (|raw| in [0..U_FULL]) -> PWM duty with deadzone compensation
//static inline int map_cmd_to_duty(int raw, int base_pwm, int U_FULL, int min_fwd, int min_rev)
//{
//    if (raw == 0) return 0;
//    int s = (raw >= 0) ? 1 : -1;
//    int min_move = (s > 0) ? min_fwd : min_rev;
//    if (min_move >= base_pwm) return s * base_pwm;   // degenerate safety
//
//    int a = raw; if (a < 0) a = -a;
//    if (a > U_FULL) a = U_FULL;
//    int span = base_pwm - min_move;
//
//    // affine map: |raw|∈[0..U_FULL] → duty∈[min_move..base_pwm]
//    int duty = min_move + (int)((1LL * a * span) / (U_FULL > 0 ? U_FULL : 1));
//    if (duty > base_pwm) duty = base_pwm;
//    return s * duty;
//}
//
//// =========================================================================================
//
//bool align_front_to_wall(int base_pwm, uint32_t timeout_ms)
//{
//    // ---- Gains (kept as you provided) ----
//    const float Kp_d = 30.0f, Ki_d = 0.3f;   // distance PI (average error)
//    const float Kp_a = 10.5f, Ki_a = 0.1f;   // angle PI (left-right difference)
//
//    // ---- Finish criteria (raw counts; tighten later if needed) ----
//    const int   DIST_TOL = 10;
//    const int   ANG_TOL  = 12;
//    const uint32_t STABLE_DWELL_MS = 150;
//
//    // ---- Independent output limits ----
//    const int V_MAX = (base_pwm > 300 ? 220 : base_pwm - 80);  // forward/back headroom
//    const int W_MAX = (base_pwm > 400 ? 320 : base_pwm - 80);  // yaw headroom
//    const int U_FULL = (V_MAX + W_MAX > 1) ? (V_MAX + W_MAX) : 1; // scale for mixer→duty
//
//    // ---- Deadzone compensation (per-wheel thresholds; calibrate once) ----
//    static WheelDZ dzL = { .min_fwd = 540, .min_rev = 560, .was_zero = 1, .kick_until = 0 };
//    static WheelDZ dzR = { .min_fwd = 520, .min_rev = 550, .was_zero = 1, .kick_until = 0 };
//    const int      KICK     = 120;    // extra duty added at start to break stiction
//    const uint32_t KICK_MS  = 60;     // kick duration (ms)
//
//    // ---- Integrators ----
//    float I_d = 0.0f, I_a = 0.0f;
//
//    // ---- Timers ----
//    uint32_t t0 = HAL_GetTick();
//    uint32_t last_ok = 0;
//    uint32_t last_tick = t0;
//
//    // Stop motors before starting
//    motor_set(0, true, 0);
//    motor_set(1, true, 0);
//
//    while (1) {
//        // --- dt ---
//        uint32_t now = HAL_GetTick();
//        float dt = (now - last_tick) * 0.001f;
//        if (dt < 0.0005f) dt = 0.0005f; // defensive lower bound
//        last_tick = now;
//
//        // --- sensors ---
//        update_sensors();
//        const int FL = (int)sensors.front_left;
//        const int FR = (int)sensors.front_right;
//
//        // --- errors (raw counts; positive means too close / left closer) ---
//        const int eL = FL - (int)target_align;
//        const int eR = FR - (int)target_align;
//        const float e_dist = 0.5f * (eL + eR);   // average error
//        const float e_ang  = (float)(eL - eR);   // left-right difference
//
//        // --- controller (pre-saturation) ---
//        const float v_unsat = Kp_d * e_dist + Ki_d * I_d;  // forward/back command
//        const float w_unsat = Kp_a * e_ang  + Ki_a * I_a;  // yaw command
//
//        // --- independent saturation ---
//        float v = v_unsat;
//        float w = w_unsat;
//        if (v >  V_MAX) v =  V_MAX;
//        if (v < -V_MAX) v = -V_MAX;
//        if (w >  W_MAX) w =  W_MAX;
//        if (w < -W_MAX) w = -W_MAX;
//
//        // --- conditional integration (anti-windup) ---
//        if ((fabsf(v_unsat) < V_MAX) || (v_unsat * e_dist <= 0.0f)) {
//            I_d += e_dist * dt;
//            const float I_d_max = (Ki_d > 0.0f) ? (0.6f * (float)V_MAX / Ki_d) : 0.0f;
//            if (I_d_max > 0.0f) {
//                if (I_d >  I_d_max) I_d =  I_d_max;
//                if (I_d < -I_d_max) I_d = -I_d_max;
//            }
//        }
//        if ((fabsf(w_unsat) < W_MAX) || (w_unsat * e_ang <= 0.0f)) {
//            I_a += e_ang * dt;
//            const float I_a_max = (Ki_a > 0.0f) ? (0.6f * (float)W_MAX / Ki_a) : 0.0f;
//            if (I_a_max > 0.0f) {
//                if (I_a >  I_a_max) I_a =  I_a_max;
//                if (I_a < -I_a_max) I_a = -I_a_max;
//            }
//        }
//
//        // --- wheel mixing (keep your original signs) ---
//        // left = -v - w ; right = -v + w
//        int rawL = (int)lroundf(-v - w);
//        int rawR = (int)lroundf(-v + w);
//
//        // --- deadzone-compensated mapping to PWM duty ---
//        int dutyL = map_cmd_to_duty(rawL, base_pwm, U_FULL, dzL.min_fwd, dzL.min_rev);
//        int dutyR = map_cmd_to_duty(rawR, base_pwm, U_FULL, dzR.min_fwd, dzR.min_rev);
//
//        // one-shot kick on 0 -> nonzero transitions
//        if (rawL != 0 && dzL.was_zero) dzL.kick_until = now + KICK_MS;
//        if (rawR != 0 && dzR.was_zero) dzR.kick_until = now + KICK_MS;
//        dzL.was_zero = (rawL == 0);
//        dzR.was_zero = (rawR == 0);
//
//        if (now < dzL.kick_until && dutyL != 0) {
//            int s = (dutyL >= 0) ? 1 : -1;
//            int d = (int)abs(dutyL) + KICK;
//            if (d > base_pwm) d = base_pwm;
//            dutyL = s * d;
//        }
//        if (now < dzR.kick_until && dutyR != 0) {
//            int s = (dutyR >= 0) ? 1 : -1;
//            int d = (int)abs(dutyR) + KICK;
//            if (d > base_pwm) d = base_pwm;
//            dutyR = s * d;
//        }
//
//        // safety clamp
//        dutyL = clampi_local(dutyL, -base_pwm, base_pwm);
//        dutyR = clampi_local(dutyR, -base_pwm, base_pwm);
//
//        // --- drive motors ---
//        motor_set(0, (dutyL >= 0), (uint16_t)abs(dutyL));
//        motor_set(1, (dutyR >= 0), (uint16_t)abs(dutyR));
//
//        // --- convergence check with dwell ---
//        const int e_dist_i = (int)lroundf(e_dist);
//        const int e_ang_i  = (int)lroundf(e_ang);
//        const uint8_t dist_ok = (abs(e_dist_i) <= DIST_TOL);
//        const uint8_t ang_ok  = (abs(e_ang_i)  <= ANG_TOL);
//
//        if (dist_ok && ang_ok) {
//            if (last_ok == 0) last_ok = now;
//            if ((now - last_ok) >= STABLE_DWELL_MS) {
//                break_motors();
//                return true;
//            }
//        } else {
//            last_ok = 0;
//        }
//
//        // --- timeout ---
//        if ((now - t0) > timeout_ms) {
//            break_motors();
//            return false;
//        }
//    }
//}

static inline int clampi_local(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

#define target_align 80

bool align_front_to_wall(int base_pwm, uint32_t timeout_ms)
{
    // ---- Your gains (unchanged) ----
    const float Kp_d = 30.0f, Ki_d = 0.3f;   // distance PI
    const float Kp_a = 10.5f, Ki_a = 0.1f;   // angle PI

    // ---- Small bias to kill steady left drift (counts). Try 0, then -1 or -2 if it still nudges left. ----
    const int   ANG_BIAS = 0;    // negative -> adds a tiny right-turn tendency

    // ---- Finish criteria (unchanged) ----
    const int   DIST_TOL = 10;         // counts
    const int   ANG_TOL  = 12;         // counts
    const uint32_t STABLE_DWELL_MS = 150;

    // ---- Output constraints (unchanged idea) ----
    const int PWM_MAX = base_pwm;      // clamp final wheel cmds
    const int PWM_MIN_MOVE = 500;      // measured deadzone threshold

    // Integrators
    float I_d = 0.0f, I_a = 0.0f;

    uint32_t t0 = HAL_GetTick();
    uint32_t last_ok = 0;
    uint32_t last_tick = HAL_GetTick();

    // reset motors
    motor_set(0, true, 0);
    motor_set(1, true, 0);

    while (1) {
        // --- timing / dt ---
        uint32_t now = HAL_GetTick();
        float dt = (now - last_tick) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        last_tick = now;

        // --- sensors ---
        update_sensors();
        int FL = (int)sensors.front_left;
        int FR = (int)sensors.front_right;

        // --- errors (raw counts) ---
        int eL = FL - (int)target_align;
        int eR = FR - (int)target_align;

        // distance = average; angle = left-right diff (+ means left closer). Add tiny bias to cancel drift.
        float e_dist = 0.5f * (eL + eR);
        float e_ang  = (float)(eL - eR + ANG_BIAS);

        // --- PI controllers ---
        I_d += e_dist * dt;
        I_a += e_ang  * dt;

        // simple clamps to keep integrators sane
        if (I_d > 100.0f) I_d = 100.0f;
        if (I_d < -100.0f) I_d = -100.0f;
        if (I_a > 100.0f) I_a = 100.0f;
        if (I_a < -100.0f) I_a = -100.0f;

        float v = Kp_d * e_dist + Ki_d * I_d;  // forward/back command  (- = back, + = forward)
        float w = Kp_a * e_ang  + Ki_a * I_a;  // turn command          (- = turn right, + = left)

        // per-wheel raw commands (signed) — keep your mixing/signs
        int cmd_left  = (int)lroundf(-v - w);
        int cmd_right = (int)lroundf(-v + w);

        // saturate
        cmd_left  = clampi_local(cmd_left,  -PWM_MAX, PWM_MAX);
        cmd_right = clampi_local(cmd_right, -PWM_MAX, PWM_MAX);

        // --- convergence check *before* applying min-move ---
        bool dist_ok = (abs((int)lroundf(e_dist)) <= DIST_TOL);
        bool ang_ok  = (abs((int)lroundf(e_ang))  <= ANG_TOL);
        bool nearly_done = (dist_ok && ang_ok);

        // --- stiction handling ---
        // If we're NOT nearly done, enforce a minimum to break deadzone.
        // If we ARE nearly done, DON'T enforce min move — brake instead to avoid creeping.
        if (!nearly_done) {
            if (cmd_left > 0  && cmd_left  < PWM_MIN_MOVE) cmd_left  = PWM_MIN_MOVE;
            if (cmd_left < 0  && -cmd_left < PWM_MIN_MOVE) cmd_left  = -PWM_MIN_MOVE;
            if (cmd_right > 0 && cmd_right < PWM_MIN_MOVE) cmd_right = PWM_MIN_MOVE;
            if (cmd_right < 0 && -cmd_right < PWM_MIN_MOVE) cmd_right = -PWM_MIN_MOVE;
        } else {
            // close enough: stop and actively brake so it doesn't coast/creep left
            cmd_left = 0;
            cmd_right = 0;
        }

        // --- drive / brake ---
        if (cmd_left == 0 && cmd_right == 0) {

            break_motors();  // actively short the motors to kill drift
        } else {
            bool lfwd = (cmd_left  >= 0);
            bool rfwd = (cmd_right >= 0);
            uint16_t lduty = (uint16_t)abs(cmd_left);
            uint16_t rduty = (uint16_t)abs(cmd_right);
            motor_set(0, lfwd, lduty);
            motor_set(1, rfwd, rduty);

        }

        // --- dwell-based success ---
        if (nearly_done) {
            if (last_ok == 0) last_ok = now;
            if ((now - last_ok) >= STABLE_DWELL_MS) {
                break_motors();
                return true;
            }
        } else {
            last_ok = 0;
        }

        // --- timeout ---
        if ((now - t0) > timeout_ms) {
            break_motors();
            return false;
        }
    }
}

bool alignment(int base_pwm, uint32_t timeout_ms)
{
    // ---- Your gains (unchanged) ----

    const float Kp_a = 1.0f, Ki_a = 0.1f;   // angle PI

    // ---- Small bias to kill steady left drift (counts). Try 0, then -1 or -2 if it still nudges left. ----


    // ---- Finish criteria (unchanged) ----
    const int   DIST_TOL = 10;         // counts
    const int   ANG_TOL  = 12;         // counts
    const uint32_t STABLE_DWELL_MS = 150;

    // ---- Output constraints (unchanged idea) ----
    const int PWM_MAX = base_pwm;      // clamp final wheel cmds
    const int PWM_MIN_MOVE = 500;      // measured deadzone threshold

    // Integrators
    float I_d = 0.0f, I_a = 0.0f;

    uint32_t t0 = HAL_GetTick();
    uint32_t last_ok = 0;
    uint32_t last_tick = HAL_GetTick();

    // reset motors
    motor_set(0, true, 0);
    motor_set(1, true, 0);

    while (1) {
        // --- timing / dt ---
        uint32_t now = HAL_GetTick();
        float dt = (now - last_tick) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        last_tick = now;

        // --- sensors ---
        update_sensors();
        int FL = (int)sensors.front_left;
        int FR = (int)sensors.front_right;

        // --- errors (raw counts) ---
        int eL = FL - (int)target_align;
        int eR = FR - (int)target_align;

        // distance = average; angle = left-right diff (+ means left closer). Add tiny bias to cancel drift.
        float e_ang  = (float)(eL - eR)*100;

        // --- PI controllers ---

        I_a += e_ang  * dt;

        // simple clamps to keep integrators sane
        if (I_a > 100.0f) I_a = 100.0f;
        if (I_a < -100.0f) I_a = -100.0f;

        float w = Kp_a * e_ang  + Ki_a * I_a;  // turn command          (- = turn right, + = left)

        // per-wheel raw commands (signed) — keep your mixing/signs
        int cmd_left  = (int)lroundf(-w);
        int cmd_right = (int)lroundf(w);

        // saturate
        cmd_left  = clampi_local(cmd_left,  -PWM_MAX, PWM_MAX);
        cmd_right = clampi_local(cmd_right, -PWM_MAX, PWM_MAX);

        // --- convergence check *before* applying min-move ---
        bool ang_ok  = (abs((int)lroundf(e_ang))  <= ANG_TOL);
        bool nearly_done = (ang_ok);

        // --- stiction handling ---
        // If we're NOT nearly done, enforce a minimum to break deadzone.
        // If we ARE nearly done, DON'T enforce min move — brake instead to avoid creeping.
        if (!nearly_done) {
            if (cmd_left > 0  && cmd_left  < PWM_MIN_MOVE) cmd_left  = PWM_MIN_MOVE;
            if (cmd_left < 0  && -cmd_left < PWM_MIN_MOVE) cmd_left  = -PWM_MIN_MOVE;
            if (cmd_right > 0 && cmd_right < PWM_MIN_MOVE) cmd_right = PWM_MIN_MOVE;
            if (cmd_right < 0 && -cmd_right < PWM_MIN_MOVE) cmd_right = -PWM_MIN_MOVE;
        } else {
            // close enough: stop and actively brake so it doesn't coast/creep left
            cmd_left = 0;
            cmd_right = 0;
        }

        // --- drive / brake ---
        if (cmd_left == 0 && cmd_right == 0) {

            break_motors();  // actively short the motors to kill drift
        } else {
            bool lfwd = (cmd_left  >= 0);
            bool rfwd = (cmd_right >= 0);
            uint16_t lduty = (uint16_t)abs(cmd_left);
            uint16_t rduty = (uint16_t)abs(cmd_right);
            motor_set(0, lfwd, lduty);
            motor_set(1, rfwd, rduty);

        }

        // --- dwell-based success ---
        if (nearly_done) {
            if (last_ok == 0) last_ok = now;
            if ((now - last_ok) >= STABLE_DWELL_MS) {
                break_motors();
                return true;
            }
        } else {
            last_ok = 0;
        }

        // --- timeout ---
        if ((now - t0) > timeout_ms) {
            break_motors();
            return false;
        }
    }
}




//// ================== WALL FOLLOW PID (ADD) ===================
//
//// ---------- Tunables ----------
//static int   WF_BASE_PWM        = 500;     // cruise PWM
//static int   WF_PWM_MIN_MOVE    = 50;      // overcome stiction
//static int   WF_PWM_MAX         = 1000;    // clamp
//
//// PID gains (your tuned values)
//static float WF_KP = 0.25f;
//static float WF_KI = 0.0f;
//static float WF_KD = 0.004f;
//static float WF_DERIV_ALPHA     = 0.85f;   // derivative low-pass filter (0..1) //0.35 in Praveen's
//static float WF_INT_LIMIT       = 250.0f;  // anti-windup clamp
//static float WF_SINGLE_ALPHA    = 0.03f;   // EMA for single-wall target tracking
//static float WF_BOTH_SCALE      = 1.0f;   // overall aggressiveness when both walls seen
//static float WF_U_SCALE      = 100.0f;
//
//// Front-wall behaviour
//static bool  WF_BRAKE_ON_FRONT  = true;
//static int   WF_SLOW_PWM        = 380;     // slow when front wall seen
//static uint8_t WF_FRONT_HOLD_MS = 120;     // brief brake pulse before stop (if you enable braking)
//
//// ---------- Lookup Tables ----------
//// Right sensor LUT
//#define R_LUT_SIZE 33
//static const int   right_adc[R_LUT_SIZE]   = {43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11};
//static const float right_dist[R_LUT_SIZE]  = {1.17,1.23,1.29,1.36,1.43,1.51,1.59,1.67,1.76,1.85,1.95,2.05,2.16,2.28,2.40,2.53,2.67,2.82,2.98,3.16,3.34,3.55,3.76,4.00,4.26,4.54,4.85,5.19,5.57,5.98,6.45,6.96};
//
//// Left sensor LUT
//#define L_LUT_SIZE 32
//static const int   left_adc[L_LUT_SIZE]    = {42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11};
//static const float left_dist[L_LUT_SIZE]   = {1.17f,1.23f,1.29f,1.36f,1.43f,1.51f,1.59f,1.67f,1.76f,1.85f,1.95f,2.05f,2.16f,2.28f,2.40f,2.53f,2.67f,2.82f,2.98f,3.16f,3.34f,3.55f,3.76f,4.00f,4.26f,4.54f,4.85f,5.19f,5.57f,5.98f,6.45f,6.96f};
//
//// ---------- Helper: Linear interpolation lookup ----------
//static float lut_lookup(int raw, const int *adc_table, const float *dist_table, int size)
//{
//    // Clamp below/above table
//    if (raw >= adc_table[0]) return dist_table[0];
//    if (raw <= adc_table[size-1]) return dist_table[size-1];
//
//    // Find interval [i, i+1] where raw fits
//    for (int i=0; i<size-1; i++) {
//        if (raw <= adc_table[i] && raw >= adc_table[i+1]) {
//            float t = (float)(raw - adc_table[i+1]) / (float)(adc_table[i] - adc_table[i+1]);
//            return dist_table[i+1] + t * (dist_table[i] - dist_table[i+1]);
//        }
//    }
//    return (float)raw; // fallback (should not happen)
//}
//
//
//// ---------- Internal state ----------
//typedef enum { WF_AUTO=0, WF_LEFT, WF_RIGHT } wf_mode_t;
//static wf_mode_t wf_mode = WF_AUTO;
//
//static float e_int = 0.0f, e_prev = 0.0f, d_filt = 0.0f;
//static uint32_t wf_last_ms = 0;
//
//static float target_left  = 2.50f;   // desired left wall distance (cm)
//static float target_right = 2.50f;   // desired right wall distance (cm)
//
//static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
//static inline float clampf(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v); }
//
//// Expose simple C API (so main.c can call without needing the enum)
////void wall_follow_reset_int(int mode, int base_pwm);   // forward decl
////void wall_follow_step(void);                          // forward decl
//
//// Call once before starting wall-follow
//void wall_follow_reset_int(int mode, int base_pwm)
//{
//    wf_mode = (mode == 1) ? WF_LEFT : (mode == 2) ? WF_RIGHT : WF_AUTO;
//    WF_BASE_PWM = base_pwm;
//
//    e_int = 0.0f; e_prev = 0.0f; d_filt = 0.0f;
//    wf_last_ms = HAL_GetTick();
//    update_sensors();
//
//    // bootstrap targets from current readings (prevents initial jump)
//    //target_left  = (float)sensors.side_left;
//    //target_right = (float)sensors.side_right;
//}
//
//// One control step; call at ~200–500 Hz inside your loop
//void wall_follow_step(void)
//{
//    // Get fresh sensors (uses your emitter-sync diff scheme)
//    update_sensors();  // reads FL/FR/SL/SR and sets wall flags
//
//    // dt
//    uint32_t now = HAL_GetTick();
//    float dt = (now - wf_last_ms) / 1000.0f;
//    if (dt <= 0.0f) dt = 0.001f;
//    wf_last_ms = now;
//
//    // Determine mode automatically if requested
//    bool Lw = sensors.wall_left;
//    bool Rw = sensors.wall_right;
//    bool Fw = sensors.wall_front;
//
//    if (wf_mode == WF_AUTO) {
//        if (Lw && Rw)       wf_mode = WF_AUTO;   // center using both
//        else if (Lw)        wf_mode = WF_LEFT;
//        else if (Rw)        wf_mode = WF_RIGHT;
//        else                wf_mode = WF_AUTO;   // nothing: just go straight
//    }
//
//    // Log-ratio error; positive => closer to LEFT (so slow left / speed right)
//    // Add +1.0f to avoid log(0). Use both-wall centering if available, else single-wall track.
//    float e = 0.0f;
//
//    if (Lw && Rw) {
//    	// Both walls: balance distances
//    	float L = lut_lookup(sensors.side_left,  left_adc,  left_dist,  L_LUT_SIZE);
//    	e = target_left - L;
////    	float R = lut_lookup(sensors.side_right, right_adc, right_dist, R_LUT_SIZE);
////    	e = WF_BOTH_SCALE * (L - R);
//        // keep single-wall targets gently aligned to present gap
//        //target_left  = (1.0f - WF_SINGLE_ALPHA)*target_left  + WF_SINGLE_ALPHA*L;
//        //target_right = (1.0f - WF_SINGLE_ALPHA)*target_right + WF_SINGLE_ALPHA*R;
//
//    } else if (Lw) {
//    	// Left wall only: hold target distance
//    	float L = lut_lookup(sensors.side_left, left_adc, left_dist, L_LUT_SIZE);
//    	e = target_left - L;
//
//    } else if (Rw) {
//    	// Right wall only: hold target distance
//    	float R = lut_lookup(sensors.side_right, right_adc, right_dist, R_LUT_SIZE);
//    	e = R - target_right;
//
//    } else {
//        // No side walls -> no correction (let heading/gyro PID handle straightness if you run it)
//        e = 0.0f;
//    }
//
//    // PID on error
//    e_int += e * dt;
//    e_int  = clampf(e_int, -WF_INT_LIMIT, WF_INT_LIMIT);
//
//    float d_raw = (e - e_prev) / dt;
//    d_filt = WF_DERIV_ALPHA * d_filt + (1.0f - WF_DERIV_ALPHA) * d_raw;
//
//    float u_norm = WF_KP*e + WF_KI*e_int + WF_KD*d_filt;  // u > 0 => speed up right / slow left
//    float u = u_norm * WF_U_SCALE;
//    e_prev = e;
//
//    // Front wall policy
//    int base = WF_BASE_PWM;
//    if (Fw && WF_BRAKE_ON_FRONT) {
//        base = WF_SLOW_PWM;
//        // If you want a hard stop, uncomment:
//        // motor_set(0, true, 0); motor_set(1, true, 0); HAL_Delay(WF_FRONT_HOLD_MS); return;
//    }
//
//    // Map correction to wheel PWMs (right = base+u, left = base-u)
//    int pwm_right = clampi((int)lroundf((float)base - u), 0, WF_PWM_MAX);
//    int pwm_left  = clampi((int)lroundf((float)base + u), 0, WF_PWM_MAX);
//
//    if (pwm_right > 0 && pwm_right < WF_PWM_MIN_MOVE) pwm_right = WF_PWM_MIN_MOVE;
//    if (pwm_left  > 0 && pwm_left  < WF_PWM_MIN_MOVE) pwm_left  = WF_PWM_MIN_MOVE;
//
//    // Apply (both forward)
//    motor_set(0, true, (uint16_t)pwm_left);   // Left
//    motor_set(1, true, (uint16_t)pwm_right);  // Right
//
////    send_bluetooth_printf("# L:%d R:%d e=%.3f u_norm=%.3f u=%.1f\n",
////        sensors.side_left, sensors.side_right, e, u_norm, u);
//
//}




/*
 * // after init
 *
fusion_reset();
fusion_set_heading_ref_to_current();  // optional on straight
while (1) {
    fusion_step();  // ~200–400 Hz
}
*
 * */


// =================== SINGLE STRAIGHT CONTROLLER (FUSION) ===================
// Uses your existing wall PID state/gains (WF_*) and your gyro PID gains (Kp_g/Ki_g/Kd_g)

//static float fus_theta = 0.0f;          // integrated heading (deg)
//static float fus_theta_ref = 0.0f;      // heading lock for current straight
//static float fus_conf_s = 0.0f;         // smoothed wall confidence
//static uint32_t fus_last_ms = 0;
//static float fus_u_prev = 0.0f;         // rate limit state
//
//// small heading-PID locals (assist only)
//static float h_int = 0.0f, h_prev = 0.0f, h_df = 0.0f;
//
//// knobs (not “tuning” — just safety rails)
//static const float FUS_CONF_EMA        = 0.90f;  // confidence smoothing
//static const float FUS_HEAD_CAP_FRAC   = 0.25f;  // max heading authority (fraction of base)
//static const float FUS_U_RATE_LIM      = 120.0f; // max |Δu| per step (PWM units)
//
//extern float Kp_g, Ki_g, Kd_g;                 // your gyro PID gains
//
//void fusion_reset(void)
//{
//    // reset wall PID memory (reuse your existing state)
//    e_int = 0.0f; e_prev = 0.0f; d_filt = 0.0f;
//    wf_last_ms = HAL_GetTick();
//
//    // reset fusion/heading memory
//    fus_theta = 0.0f;
//    fus_theta_ref = 0.0f;
//    fus_conf_s = 0.0f;
//    fus_u_prev = 0.0f;
//    h_int = 0.0f; h_prev = 0.0f; h_df = 0.0f;
//    fus_last_ms = HAL_GetTick();
//
//    // capture initial targets to avoid a jump at start
//    // for the Target, values should be updated -------------------->
//    update_sensors();
//    //target_left  = (float)sensors.side_left;
//    //target_right = (float)sensors.side_right;
//}
//
//void fusion_set_heading_ref_to_current(void)
//{
//    fus_theta_ref = fus_theta;
//}
//
//// Call at ~200–500 Hz. Pass 0 to use WF_BASE_PWM.
//void fusion_step(int base_pwm)
//{
//    // --- timing ---
//    uint32_t now = HAL_GetTick();
//    float dt = (now - fus_last_ms) * 0.001f;
//    if (dt <= 0.0f) dt = 0.001f;
//    fus_last_ms = now;
//
//    // --- sensors + gyro ---
//    update_sensors();
//    bool Lw = sensors.wall_left;
//    bool Rw = sensors.wall_right;
//    bool Fw = sensors.wall_front;
//
//    int  L = sensors.side_left;
//    int  R = sensors.side_right;
//
//    mpu9250_read_gyro();
//    float gz = mpu9250_get_gyro_z_compensated();   // deg/s
//    fus_theta += gz * dt;
//
//    // -------- WALL PID (log-ratio + your WF_* state) --------
//    float e_wall = 0.0f;
//    if (Lw && Rw) {
//        e_wall = WF_BOTH_SCALE * (logf((float)L + 1.0f) - logf((float)R + 1.0f));
//        // keep single-wall targets gently aligned (same as wall_follow_step)
//        //target_left  = (1.0f - WF_SINGLE_ALPHA)*target_left  + WF_SINGLE_ALPHA*(float)L;
//        //target_right = (1.0f - WF_SINGLE_ALPHA)*target_right + WF_SINGLE_ALPHA*(float)R;
//    } else if (Lw) {
//        //target_left  = (1.0f - WF_SINGLE_ALPHA)*target_left  + WF_SINGLE_ALPHA*(float)L;
//        e_wall = logf((float)L + 1.0f) - logf(target_left + 1.0f);
//    } else if (Rw) {
//        //target_right = (1.0f - WF_SINGLE_ALPHA)*target_right + WF_SINGLE_ALPHA*(float)R;
//        e_wall = logf(target_right + 1.0f) - logf((float)R + 1.0f);
//    } else {
//        e_wall = 0.0f;
//    }
//
//    // step the SAME wall PID states/gains
//    e_int += e_wall * dt;
//    e_int  = clampf(e_int, -WF_INT_LIMIT, WF_INT_LIMIT);
//    float d_raw = (e_wall - e_prev) / dt;
//    d_filt = WF_DERIV_ALPHA * d_filt + (1.0f - WF_DERIV_ALPHA) * d_raw;
//    float u_wall = WF_KP*e_wall + WF_KI*e_int + WF_KD*d_filt;
//    e_prev = e_wall;
//
//    // -------- HEADING PID (assist; reuses your gyro PID gains) --------
//    float e_head = fus_theta_ref - fus_theta;
//
//    // allow heading integrator only when walls are NOT present
//    if (!(Lw || Rw)) {
//        h_int += e_head * dt;
//        if (h_int > 200.0f) h_int = 200.0f;
//        if (h_int < -200.0f) h_int = -200.0f;
//    }
//
//    const float H_ALPHA = 0.90f;                  // small derivative filter
//    float h_draw = (e_head - h_prev) / dt;
//    h_df = H_ALPHA*h_df + (1.0f - H_ALPHA)*h_draw;
//    h_prev = e_head;
//
//    float u_head = Kp_g*e_head + Ki_g*h_int + Kd_g*h_df;
//
//    // limit heading authority so it never fights good wall info
//    int base_unclamped = (base_pwm > 0) ? base_pwm : WF_BASE_PWM;
//    float head_cap = FUS_HEAD_CAP_FRAC * (float)base_unclamped;   // e.g., 25% of base
//    if (u_head >  head_cap) u_head =  head_cap;
//    if (u_head < -head_cap) u_head = -head_cap;
//
//    // -------- BLEND (confidence from side walls) --------
//    float conf = 0.0f; if (Lw) conf += 0.5f; if (Rw) conf += 0.5f;
//    fus_conf_s = FUS_CONF_EMA*fus_conf_s + (1.0f - FUS_CONF_EMA)*conf;  // smooth handoffs
//    float u = fus_conf_s*u_wall + (1.0f - fus_conf_s)*u_head;
//
//    // -------- OUTPUT SHAPING --------
//    // optional rate limit on correction to avoid jerk
//    float du = u - fus_u_prev;
//    if (du >  FUS_U_RATE_LIM) du =  FUS_U_RATE_LIM;
//    if (du < -FUS_U_RATE_LIM) du = -FUS_U_RATE_LIM;
//    u = fus_u_prev + du;
//    fus_u_prev = u;
//
//    int base = (base_pwm > 0) ? base_pwm : WF_BASE_PWM;
//    if (Fw && WF_BRAKE_ON_FRONT) base = WF_SLOW_PWM;
//
//    // right = base + u, left = base - u
//    int pwm_right = clampi((int)lroundf((float)base + u), 0, WF_PWM_MAX);
//    int pwm_left  = clampi((int)lroundf((float)base - u), 0, WF_PWM_MAX);
//
//    if (pwm_right > 0 && pwm_right < WF_PWM_MIN_MOVE) pwm_right = WF_PWM_MIN_MOVE;
//    if (pwm_left  > 0 && pwm_left  < WF_PWM_MIN_MOVE) pwm_left  = WF_PWM_MIN_MOVE;
//
//    motor_set(0, true, (uint16_t)pwm_left);
//    motor_set(1, true, (uint16_t)pwm_right);
//}


// ================== WALL FOLLOW PID (ADD) ===================

// ---------- Tunables ----------
static int   WF_BASE_PWM        = 500;     // cruise PWM
static int   WF_PWM_MIN_MOVE    = 450;      // overcome stiction
static int   WF_PWM_MAX         = 1000;    // clamp

// PID gains (your tuned values)
static float WF_KP = 0.8f;
static float WF_KI = 0.0f;
static float WF_KD = 0.0f;
static float WF_DERIV_ALPHA     = 0.85f;   // derivative low-pass filter (0..1) //0.35 in Praveen's
static float WF_INT_LIMIT       = 250.0f;  // anti-windup clamp
static float WF_SINGLE_ALPHA    = 0.03f;   // EMA for single-wall target tracking
static float WF_BOTH_SCALE      = 1.0f;   // overall aggressiveness when both walls seen
static float WF_U_SCALE      = 100.0f;

// Front-wall behaviour
static bool  WF_BRAKE_ON_FRONT  = true;
static int   WF_SLOW_PWM        = 380;     // slow when front wall seen
static uint8_t WF_FRONT_HOLD_MS = 120;     // brief brake pulse before stop (if you enable braking)

// ---------- Lookup Tables ----------
//// Right sensor LUT (SR)
//#define R_LUT_SIZE 53
//// Right (RS) Sensor Fit Parameters: q0=-2.0390, q1=236.4218, q2=-605.3979
//// Range: ADC 71 (1.17cm) down to ADC 19 (8.73cm)
//static const uint8_t right_adc[R_LUT_SIZE] = {71,70,69,68,67,66,65,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19};
//static const float right_dist[R_LUT_SIZE]  = {1.1707f,1.2149f,1.2602f,1.3068f,1.3548f,1.4041f,1.4549f,1.5072f,1.5611f,1.6167f,1.6740f,1.7332f,1.7942f,1.8572f,1.9224f,1.9897f,2.0594f,2.1315f,2.2062f,2.2836f,2.3639f,2.4472f,2.5337f,2.6236f,2.7171f,2.8145f,2.9158f,3.0215f,3.1317f,3.2468f,3.3672f,3.4931f,3.6250f,3.7633f,3.9085f,4.0611f,4.2217f,4.3908f,4.5693f,4.7579f,4.9575f,5.1690f,5.3936f,5.6324f,5.8869f,6.1585f,6.4492f,6.7608f,7.0957f,7.4566f,7.8464f,8.2685f,8.7272f};
//
//// Left sensor LUT (LR)
//#define L_LUT_SIZE 55
//// Left (LS) Sensor Fit Parameters: q0=-2.0390, q1=236.4218, q2=-605.3979
//// Range: ADC 69 (1.07cm) down to ADC 15 (8.69cm)
//static const uint8_t left_adc[L_LUT_SIZE]  = {69,68,67,66,65,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15};
//static const float left_dist[L_LUT_SIZE]   = {1.0716f,1.1088f,1.1470f,1.1864f,1.2269f,1.2686f,1.3116f,1.3558f,1.4015f,1.4486f,1.4972f,1.5474f,1.5992f,1.6528f,1.7083f,1.7657f,1.8251f,1.8866f,1.9504f,2.0166f,2.0853f,2.1567f,2.2309f,2.3081f,2.3885f,2.4723f,2.5597f,2.6509f,2.7461f,2.8458f,2.9501f,3.0595f,3.1742f,3.2946f,3.4213f,3.5547f,3.6954f,3.8438f,4.0008f,4.1670f,4.3433f,4.5305f,4.7298f,4.9423f,5.1692f,5.4122f,5.6728f,5.9531f,6.2551f,6.5814f,6.9348f,7.3186f,7.7365f,8.1927f,8.6918f};


// ---------- Lookup Tables ----------
// Right sensor LUT
#define R_LUT_SIZE 33
static const int   right_adc[R_LUT_SIZE]   = {43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11};
static const float right_dist[R_LUT_SIZE]  = {1.17,1.23,1.29,1.36,1.43,1.51,1.59,1.67,1.76,1.85,1.95,2.05,2.16,2.28,2.40,2.53,2.67,2.82,2.98,3.16,3.34,3.55,3.76,4.00,4.26,4.54,4.85,5.19,5.57,5.98,6.45,6.96};

// Left sensor LUT
#define L_LUT_SIZE 32
static const int   left_adc[L_LUT_SIZE]    = {42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11};
static const float left_dist[L_LUT_SIZE]   = {1.17,1.23,1.29,1.36,1.43,1.51,1.59,1.67,1.76,1.85,1.95,2.05,2.16,2.28,2.40f,2.53f,2.67f,2.82f,2.98f,3.16f,3.34f,3.55f,3.76f,4.00f,4.26f,4.54f,4.85f,5.19f,5.57f,5.98f,6.45f,6.96f};

//// ---------- Lookup Tables ----------
//// Right sensor LUT
//#define R_LUT_SIZE 33
//static const int   right_adc[R_LUT_SIZE]   = {43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11};
//static const float right_dist[R_LUT_SIZE]  = {2.3885f,2.4723f,2.5597f,2.6509f,2.7461f,2.8458f,2.9501f,3.0595f,3.1742f,3.2946f,3.4213f,3.5547f,3.6954f,3.8438f,4.0008f,4.1670f,4.3433f,4.5305f,4.7298f,4.9423f,5.1692f,5.4122f,5.6728f,5.9531f,6.2551f,6.5814f,6.9348f,7.3186f,7.7365f,8.1927f,8.6918f,8.7918f,8.8918f,8.9918f,9.1918f,9.2918f};
//
//// Left sensor LUT
//#define L_LUT_SIZE 33
//static const int   left_adc[L_LUT_SIZE]    = {43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11};
//static const float left_dist[L_LUT_SIZE]   = {2.3885f,2.4723f,2.5597f,2.6509f,2.7461f,2.8458f,2.9501f,3.0595f,3.1742f,3.2946f,3.4213f,3.5547f,3.6954f,3.8438f,4.0008f,4.1670f,4.3433f,4.5305f,4.7298f,4.9423f,5.1692f,5.4122f,5.6728f,5.9531f,6.2551f,6.5814f,6.9348f,7.3186f,7.7365f,8.1927f,8.6918f,8.7918f,8.8918f,8.9918f,9.1918f,9.2918f};


// ---------- Helper: Linear interpolation lookup ----------
// ---------- Helper: Linear interpolation lookup ----------
static float lut_lookup(int raw, const int *adc_table, const float *dist_table, int size)
{
    if (raw >= adc_table[0]){
    	return dist_table[0];
    }
    if (raw <= adc_table[size-1]){
    	return dist_table[size-1];
    }

    for (int i = 0; i < size - 1; ++i) {
        int a_hi = adc_table[i], a_lo = adc_table[i+1];
        if (raw <= a_hi && raw >= a_lo) {
            float t   = (float)(raw - a_lo) / (float)(a_hi - a_lo); // 0..1
            float dhi = dist_table[i], dlo = dist_table[i+1];
            return dlo + t * (dhi - dlo);
        }
    }
    return dist_table[size-1];
}

// ---------- Internal state ----------
typedef enum { WF_AUTO=0, WF_LEFT, WF_RIGHT } wf_mode_t;
static wf_mode_t wf_mode = WF_AUTO;

static float e_int = 0.0f, e_prev = 0.0f, d_filt = 0.0f;
static uint32_t wf_last_ms = 0;

static float target_left  = 2.9f;   // desired left wall distance (cm)
static float target_right = 2.9f;// desired right wall distance (cm)

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float clampf(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v); }
float e = 0.0f;
int error_count;

// Expose simple C API (so main.c can call without needing the enum)
//void wall_follow_reset_int(int mode, int base_pwm);   // forward decl
//void wall_follow_step(void);                          // forward decl

// Call once before starting wall-follow
void wall_follow_reset_int(int mode, int base_pwm)
{
    wf_mode = (mode == 1) ? WF_LEFT : (mode == 2) ? WF_RIGHT : WF_AUTO;
    WF_BASE_PWM = base_pwm;

    e_int = 0.0f; e_prev = 0.0f; d_filt = 0.0f;
    wf_last_ms = HAL_GetTick();
    update_sensors();
    error_count=0;


    // bootstrap targets from current readings (prevents initial jump)
    //target_left  = (float)sensors.side_left;
    //target_right = (float)sensors.side_right;
}


// One control step; call at ~200–500 Hz inside your loop
void wall_follow_step(void)
{
    // Get fresh sensors (uses your emitter-sync diff scheme)
    update_sensors();  // reads FL/FR/SL/SR and sets wall flags

    // dt
    uint32_t now = HAL_GetTick();
    float dt = (now - wf_last_ms) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    wf_last_ms = now;

    // Determine mode automatically if requested
    bool Lw = sensors.wall_left;
    bool Rw = sensors.wall_right;
    bool Fw = sensors.wall_front;


    if (wf_mode == WF_AUTO) {
        if (Lw && Rw)       wf_mode = WF_AUTO;   // center using both
        else if (Lw)        wf_mode = WF_LEFT;
        else if (Rw)        wf_mode = WF_RIGHT;
        else                wf_mode = WF_AUTO;   // nothing: just go straight
    }

    // Log-ratio error; positive => closer to LEFT (so slow left / speed right)
    // Add +1.0f to avoid log(0). Use both-wall centering if available, else single-wall track.

    //float L=3.4f;
    float L = lut_lookup(sensors.side_left,  left_adc,  left_dist,  L_LUT_SIZE);
    float R = lut_lookup(sensors.side_right, right_adc, right_dist, R_LUT_SIZE);

    int isRight=0; // if Left=1 Right = -1


    if (Lw && Rw ){
    	// Both walls: balance distances
    	//L = lut_lookup(sensors.side_left,  left_adc,  left_dist,  L_LUT_SIZE);
    	//float R = lut_lookup(sensors.side_right, right_adc, right_dist, R_LUT_SIZE);
    	//e = (R - L)*0.5f;
    	//e = target_right-L;
//    	e = target_right-R;
//    	isRight=1;

    	e = (R - L)*0.5f;
//  	  if (e>=1.0f) e = 0.0f;
//  	  else if (e <= -1.0f)  e = 0.0f;




    } else if (Lw) {
    	// Left wall only: hold target distance
    	//L = lut_lookup(sensors.side_left, left_adc, left_dist, L_LUT_SIZE);


    	e = target_left - L;
//    	if (e>=1.0f) e = 0.0f;
//
//    	else if (e <= -1.0f)  e = 0.0f;
    	//e = R - target_right;
    	//e = 0.0f;

    } else if (Rw) {
    	// Right wall only: hold target distance
    	//float R = lut_lookup(sensors.side_right, right_adc, right_dist, R_LUT_SIZE);
    	e = target_right-R;
//    	if (e>=1.0f) e = 0.0f;
//
//    	else if (e <= -1.0f)  e = 0.0f;
    	isRight=1;
    	//e = target_left - L;
    	//e = 0.0f;

      } else {

        // No side walls -> no correction (let heading/gyro PID handle straightness if you run it)
    	  e=0.0f;

      }

    // PID on error
	if (isRight) e=-e;

    e_int += e * dt;
    e_int  = clampf(e_int, -WF_INT_LIMIT, WF_INT_LIMIT);

    float d_raw = (e - e_prev) / dt;
    d_filt = WF_DERIV_ALPHA * d_filt + (1.0f - WF_DERIV_ALPHA) * d_raw;

    float u_norm = WF_KP*e + WF_KI*e_int + WF_KD*d_filt;  // u > 0 => speed up right / slow left
    float u = u_norm * WF_U_SCALE;
    e_prev = e;

    // Front wall policy
    int base = WF_BASE_PWM;
//    if (Fw && WF_BRAKE_ON_FRONT) {
//        base = WF_SLOW_PWM;
//        // If you want a hard stop, uncomment:
//        // motor_set(0, true, 0); motor_set(1, true, 0); HAL_Delay(WF_FRONT_HOLD_MS); return;
//    }

    // Map correction to wheel PWMs (right = base+u, left = base-u)
    int pwm_right = clampi((int)lroundf((float)base - u), 0, WF_PWM_MAX);
    int pwm_left  = clampi((int)lroundf((float)base + u), 0, WF_PWM_MAX);

    if (pwm_right > 0 && pwm_right < WF_PWM_MIN_MOVE) pwm_right = WF_PWM_MIN_MOVE;
    if (pwm_left  > 0 && pwm_left  < WF_PWM_MIN_MOVE) pwm_left  = WF_PWM_MIN_MOVE;

    // Apply (both forward)
    motor_set(0, true, (uint16_t)pwm_left);   // Left
    motor_set(1, true, (uint16_t)pwm_right);  // Right

//    send_bluetooth_printf("# L:%d R:%d e=%.3f u_norm=%.3f u=%.1f\n",
//        sensors.side_left, sensors.side_right, e, u_norm, u);

    //send_bluetooth_printf("L:%f    e:%f      pwm_right: %d     pwm_left:%d  \r\n", L, e,pwm_right,pwm_left);
	//send_bluetooth_printf("R:%f    e:%f      pwm_right: %d     pwm_left:%d  \r\n", R, e,pwm_right,pwm_left);

}



// =================== SINGLE STRAIGHT CONTROLLER (FUSION) ===================
// (Left as you had it; no structural changes beyond safety clamps/rate limit above)

//static float fus_theta = 0.0f;          // integrated heading (deg)
//static float fus_theta_ref = 0.0f;      // heading lock for current straight
//static float fus_conf_s = 0.0f;         // smoothed wall confidence
//static uint32_t fus_last_ms = 0;
//static float fus_u_prev = 0.0f;         // rate limit state
//
//// small heading-PID locals (assist only)
//static float h_int = 0.0f, h_prev = 0.0f, h_df = 0.0f;
//
//// knobs (safety rails)
//static const float FUS_CONF_EMA        = 0.90f;  // confidence smoothing
//static const float FUS_HEAD_CAP_FRAC   = 0.25f;  // max heading authority (fraction of base)
//static const float FUS_U_RATE_LIM      = 120.0f; // max |Δu| per step (PWM units)
//
//extern float Kp_g, Ki_g, Kd_g;                 // your gyro PID gains
//
//void fusion_reset(void)
//{
//    e_int = 0.0f; e_prev = 0.0f; d_filt = 0.0f;
//    wf_last_ms = HAL_GetTick();
//
//    fus_theta = 0.0f;
//    fus_theta_ref = 0.0f;
//    fus_conf_s = 0.0f;
//    fus_u_prev = 0.0f;
//    h_int = 0.0f; h_prev = 0.0f; h_df = 0.0f;
//    fus_last_ms = HAL_GetTick();
//
//    update_sensors();
//}
//
//void fusion_set_heading_ref_to_current(void)
//{
//    fus_theta_ref = fus_theta;
//}
//
//// Call at ~200–500 Hz. Pass 0 to use WF_BASE_PWM.
//void fusion_step(int base_pwm)
//{
//    uint32_t now = HAL_GetTick();
//    float dt = (now - fus_last_ms) * 0.001f;
//    if (dt <= 0.0f) dt = 0.001f;
//    fus_last_ms = now;
//
//    update_sensors();
//    bool Lw = sensors.wall_left;
//    bool Rw = sensors.wall_right;
//    bool Fw = sensors.wall_front;
//
//    int  L = sensors.side_left;
//    int  R = sensors.side_right;
//
//    mpu9250_read_gyro();
//    float gz = mpu9250_get_gyro_z_compensated();   // deg/s
//    fus_theta += gz * dt;
//
//    // Wall PID error (log-ratio style kept as-is)
//    float e_wall = 0.0f;
//    if (Lw && Rw) {
//        e_wall = WF_BOTH_SCALE * (logf((float)L + 1.0f) - logf((float)R + 1.0f));
//    } else if (Lw) {
//        e_wall = logf((float)L + 1.0f) - logf(target_left + 1.0f);
//    } else if (Rw) {
//        e_wall = logf(target_right + 1.0f) - logf((float)R + 1.0f);
//    } else {
//        e_wall = 0.0f;
//    }
//
//    // Reuse WF_* states for wall correction
//    e_int += e_wall * dt;
//    e_int  = clampf(e_int, -WF_INT_LIMIT, WF_INT_LIMIT);
//    float d_raw = (e_wall - e_prev) / dt;
//    d_filt = WF_DERIV_ALPHA * d_filt + (1.0f - WF_DERIV_ALPHA) * d_raw;
//    float u_wall = WF_KP*e_wall + WF_KI*e_int + WF_KD*d_filt;
//    e_prev = e_wall;
//
//    // Heading PID (assist)
//    float e_head = fus_theta_ref - fus_theta;
//    if (!(Lw || Rw)) { // integrate heading only when no walls
//        h_int += e_head * dt;
//        if (h_int > 200.0f) h_int = 200.0f;
//        if (h_int < -200.0f) h_int = -200.0f;
//    }
//
//    const float H_ALPHA = 0.90f;
//    float h_draw = (e_head - h_prev) / dt;
//    h_df = H_ALPHA*h_df + (1.0f - H_ALPHA)*h_draw;
//    h_prev = e_head;
//
//    float u_head = Kp_g*e_head + Ki_g*h_int + Kd_g*h_df;
//
//    int base = (base_pwm > 0) ? base_pwm : WF_BASE_PWM;
//    float head_cap = FUS_HEAD_CAP_FRAC * (float)base;
//    if (u_head >  head_cap) u_head =  head_cap;
//    if (u_head < -head_cap) u_head = -head_cap;
//
//    // Blend (confidence from side walls)
//    float conf = 0.0f; if (Lw) conf += 0.5f; if (Rw) conf += 0.5f;
//    fus_conf_s = FUS_CONF_EMA*fus_conf_s + (1.0f - FUS_CONF_EMA)*conf;
//    float u = fus_conf_s*u_wall + (1.0f - fus_conf_s)*u_head;
//
//    // Rate limit
//    float du = u - fus_u_prev;
//    if (du >  FUS_U_RATE_LIM) du =  FUS_U_RATE_LIM;
//    if (du < -FUS_U_RATE_LIM) du = -FUS_U_RATE_LIM;
//    u = fus_u_prev + du;
//    fus_u_prev = u;
//
//    if (Fw && WF_BRAKE_ON_FRONT) base = WF_SLOW_PWM;
//
//    // right = base + u, left = base - u (kept)
//    int pwm_right = clampi((int)lroundf((float)base + u), 0, WF_PWM_MAX);
//    int pwm_left  = clampi((int)lroundf((float)base - u), 0, WF_PWM_MAX);
//
//    if (pwm_right > 0 && pwm_right < WF_PWM_MIN_MOVE) pwm_right = WF_PWM_MIN_MOVE;
//    if (pwm_left  > 0 && pwm_left  < WF_PWM_MIN_MOVE) pwm_left  = WF_PWM_MIN_MOVE;
//
//    motor_set(0, true, (uint16_t)pwm_left);
//    motor_set(1, true, (uint16_t)pwm_right);
//}



// ===== FUSION: wall + gyro =====
// Put this next to your wall-follow code (same file/scope as WF_* vars & state).

// --- Forward decls from your codebase (already exist) ---
extern float mpu9250_get_gyro_z_compensated(void);
extern void  mpu9250_read_gyro(void);



// --- Reuse your tuned parameters/state (defined with wall code) ---
extern int   WF_BASE_PWM, WF_PWM_MIN_MOVE, WF_PWM_MAX;
extern float WF_KP, WF_KI, WF_KD, WF_DERIV_ALPHA, WF_INT_LIMIT, WF_SINGLE_ALPHA, WF_BOTH_SCALE, WF_U_SCALE;
extern bool  WF_BRAKE_ON_FRONT;
extern int   WF_SLOW_PWM;
extern uint8_t WF_FRONT_HOLD_MS;

// wall PID state you already use in wall_follow_step()
static float wf_e_int = 0.0f, wf_e_prev = 0.0f, wf_d_filt = 0.0f;
static uint32_t wf_last_ms_fus = 0;

// single-wall “targets” if you use them
static float fus_target_left  = 2.50f;
static float fus_target_right = 2.50f;

// heading fusion state
static float fus_theta = 0.0f;        // integrated heading (deg)
static float fus_theta_ref = 0.0f;    // desired heading lock
static float fus_conf_s = 0.0f;       // smoothed wall confidence [0..1]
static float fus_u_prev = 0.0f;       // rate-limiter on correction
static uint32_t fus_last_ms = 0;

// knobs (not PID gains)
static const float FUS_CONF_EMA        = 0.97f;  // blend smoothing
static const float FUS_HEAD_CAP_FRAC = 0.50f;   // was 0.25 → gyro can steer
static const float FUS_WALL_CAP_FRAC = 0.15f;   // new: clamp |u_wall|
static const float FUS_DU_RATE_LIMIT   = 120.0f; // PWM units per step

// --- Gyro rate PID step you already calibrated (we just call it) ---


// ---------- Lookup helpers (reuse your LUTs if you have them) ----------
static float lut_lookup_lin(int raw, const int *adc_table, const float *dist_table, int size)
{
    if (raw >= adc_table[0]) return dist_table[0];
    if (raw <= adc_table[size-1]) return dist_table[size-1];
    for (int i = 0; i < size-1; i++) {
        if (raw <= adc_table[i] && raw >= adc_table[i+1]) {
            float t = (float)(raw - adc_table[i+1]) / (float)(adc_table[i] - adc_table[i+1]);
            return dist_table[i+1] + t * (dist_table[i] - dist_table[i+1]);
        }
    }
    return (float)raw;
}

// ------- NON-ACTUATING wall correction: compute u_wall only -------
static float wall_compute_u(float dt, int *p_has_left, int *p_has_right, int *p_has_front)
{
    update_sensors();
    const int Lw = sensors.wall_left  ? 1 : 0;
    const int Rw = sensors.wall_right ? 1 : 0;
    const int Fw = sensors.wall_front ? 1 : 0;
    if (p_has_left)  *p_has_left  = Lw;
    if (p_has_right) *p_has_right = Rw;
    if (p_has_front) *p_has_front = Fw;

    // read raw side ADCs
    const int Lraw = sensors.side_left;
    const int Rraw = sensors.side_right;

    // pick your distance model: LUT (preferred) or raw log
    float e = 0.0f;
    if (Lw && Rw) {
        // both walls → center
        const float L = lut_lookup_lin(Lraw, left_adc,  left_dist,  L_LUT_SIZE);
        const float R = lut_lookup_lin(Rraw, right_adc, right_dist, R_LUT_SIZE);
        e = WF_BOTH_SCALE * (L - R);  // +e means left closer → slow left / speed right
        // gently align targets for when a wall disappears
        fus_target_left  = (1.0f - WF_SINGLE_ALPHA)*fus_target_left  + WF_SINGLE_ALPHA*L;
        fus_target_right = (1.0f - WF_SINGLE_ALPHA)*fus_target_right + WF_SINGLE_ALPHA*R;
    } else if (Lw) {
        const float L = lut_lookup_lin(Lraw, left_adc, left_dist, L_LUT_SIZE);
        e = L- target_left;      // hold distance to left
    } else if (Rw) {
        const float R = lut_lookup_lin(Rraw, right_adc, right_dist, R_LUT_SIZE);
        e = target_right-R;     // hold distance to right
    } else {
        e = 0.0f; // no walls → let heading handle it
    }

    // PID on e (reuse your wall PID state/gains)
    wf_e_int += e * dt;
    if (wf_e_int >  WF_INT_LIMIT) wf_e_int =  WF_INT_LIMIT;
    if (wf_e_int < -WF_INT_LIMIT) wf_e_int = -WF_INT_LIMIT;

    const float d_raw = (e - wf_e_prev) / dt;
    wf_d_filt = WF_DERIV_ALPHA*wf_d_filt + (1.0f - WF_DERIV_ALPHA)*d_raw;
    wf_e_prev = e;

    const float u_norm = WF_KP*e + WF_KI*wf_e_int + WF_KD*wf_d_filt;
    return (WF_U_SCALE * u_norm);   // map to PWM units like your wall_follow_step()
}

// ---------- Public API ----------
void fusion_reset(void)
{
    // reset wall PID memory
    wf_e_int = 0.0f; wf_e_prev = 0.0f; wf_d_filt = 0.0f;
    wf_last_ms_fus = HAL_GetTick();

    // heading & blending
    fus_theta = 0.0f;
    fus_theta_ref = 0.0f;
    fus_conf_s = 0.0f;
    fus_u_prev = 0.0f;
    fus_last_ms = HAL_GetTick();

    // init targets from current reading to avoid jumps
    update_sensors();
    fus_target_left  = lut_lookup_lin(sensors.side_left,  left_adc,  left_dist,  L_LUT_SIZE);
    fus_target_right = lut_lookup_lin(sensors.side_right, right_adc, right_dist, R_LUT_SIZE);
}

void fusion_set_heading_ref_to_current(void)
{
    // capture current integrated heading as the straight-line lock
    fus_theta_ref = fus_theta;
}

// Call at ~200–500 Hz. Pass 0 to use WF_BASE_PWM.
void fusion_step(int base_pwm)
{
    // timing
    uint32_t now = HAL_GetTick();
    float dt = (now - fus_last_ms) * 0.001f;
    if (dt <= 0.0f) dt = 0.001f;
    fus_last_ms = now;

    // gyro
    mpu9250_read_gyro();
    const float gz = mpu9250_get_gyro_z_compensated(); // deg/s
    fus_theta += gz * dt;

    // wall correction
    int hasL=0, hasR=0, hasF=0;
    float u_wall = wall_compute_u(dt, &hasL, &hasR, &hasF);

    // heading assist using your rate PID: command 0 deg/s + proportional bias from heading error
    // Map heading error → desired rate (light touch so it won’t fight walls)
    const float e_head = fus_theta_ref - fus_theta;              // deg
    const float k_head2rate = 60.0f;                             // deg/s per deg (small)
    float sp_rate = k_head2rate * e_head;                        // desired deg/s
    // soft cap desired rate
    if (sp_rate >  300.0f) sp_rate =  300.0f;
    if (sp_rate < -300.0f) sp_rate = -300.0f;

    float dummy_dt = dt;
    float u_head = gyro_rate_pid_step(sp_rate, gz, &dummy_dt);  // returns ΔPWM using your tuned K’s

    // blend by wall confidence (0.5 for each side seen)
    float conf = 0.0f; if (hasL) conf += 0.20f; if (hasR) conf += 0.20f; ////////////////////////
    fus_conf_s = FUS_CONF_EMA*fus_conf_s + (1.0f - FUS_CONF_EMA)*conf;

    // cap heading authority to a fraction of base
    const int base_unclamped = (base_pwm > 0) ? base_pwm : WF_BASE_PWM;
    //float u_head_capped = u_head;
    float head_cap = FUS_HEAD_CAP_FRAC * (float)base_unclamped;
    if (u_head >  head_cap) u_head =  head_cap;
    if (u_head < -head_cap) u_head = -head_cap;

//    float wall_cap = FUS_WALL_CAP_FRAC * base_unclamped;
//    if (u_wall >  wall_cap) u_wall =  wall_cap;
//    if (u_wall < -wall_cap) u_wall = -wall_cap;

    // final correction
    if (sensors.side_left>40 || sensors.side_right>40) fus_conf_s=1.0f;
    float u = fus_conf_s * u_wall + (1.0f - fus_conf_s) * u_head;

    // optional rate limiting on correction to avoid jerk
    float du = u - fus_u_prev;
    if (du >  FUS_DU_RATE_LIMIT) du =  FUS_DU_RATE_LIMIT;
    if (du < -FUS_DU_RATE_LIMIT) du = -FUS_DU_RATE_LIMIT;
    u = fus_u_prev + du;
    fus_u_prev = u;

    // front-wall policy same as your wall code
    int base = (base_pwm > 0) ? base_pwm : WF_BASE_PWM;
//    if (hasF && WF_BRAKE_ON_FRONT) base = WF_SLOW_PWM;

    // right = base + u ; left = base - u   (same sign convention as your code)
    int pwm_right = base + (int)lroundf(u);
    int pwm_left  = base - (int)lroundf(u);

    if (pwm_right < 0) pwm_right = 0; if (pwm_right > WF_PWM_MAX) pwm_right = WF_PWM_MAX;
    if (pwm_left  < 0) pwm_left  = 0; if (pwm_left  > WF_PWM_MAX) pwm_left  = WF_PWM_MAX;

    if (pwm_right > 0 && pwm_right < WF_PWM_MIN_MOVE) pwm_right = WF_PWM_MIN_MOVE;
    if (pwm_left  > 0 && pwm_left  < WF_PWM_MIN_MOVE) pwm_left  = WF_PWM_MIN_MOVE;

    motor_set(0, true, (uint16_t)pwm_left);
    motor_set(1, true, (uint16_t)pwm_right);
}

static float fus_theta_local = 0.0f;

static float lut_lookup_lin_local(int raw, const int *adc_table, const float *dist_table, int size)
{
    if (!adc_table || !dist_table || size <= 1) return (float)raw;
    if (raw >= adc_table[0])      return dist_table[0];
    if (raw <= adc_table[size-1]) return dist_table[size-1];
    for (int i = 0; i < size-1; i++) {
        if (raw <= adc_table[i] && raw >= adc_table[i+1]) {
            float t = (float)(raw - adc_table[i+1]) / (float)(adc_table[i] - adc_table[i+1]);
            return dist_table[i+1] + t * (dist_table[i] - dist_table[i+1]);
        }
    }
    return dist_table[size-1];
}

// Optional wheel balance scalers (1.0 = no correction); tune if it creeps
#ifndef SPIN_BAL_L
#define SPIN_BAL_L 1.00f
#endif
#ifndef SPIN_BAL_R
#define SPIN_BAL_R 1.00f
#endif

bool fusion_align_entry2(int base_pwm, uint32_t timeout_ms)
{

// --- PD gains in PWM units ---
	const float KP_PWM_PER_CM    = 600.0f;   // ↑ from 500 → stronger shove
	const float KD_PWM_PER_CMPS  = 140.0f;   // a bit more damping
	const float D_ALPHA          = 0.92f;    // smoother derivative


	// Stiction kick & limits
	const int   SPIN_PWM_MAX     = clampi_local(WF_PWM_MAX/2, WF_PWM_MIN_MOVE, WF_PWM_MAX);
	const int   SPIN_PWM_MIN     = WF_PWM_MIN_MOVE;
	const int   KICK_PWM         = SPIN_PWM_MIN + 120; // initial shove
	const uint32_t KICK_MS       = 120;                // duration for kick after start
	const int   DU_RATE_LIMIT    = 180;                // allow faster rise (or set 0 to disable)
	const int PWM_MIN_MOVE = 500;

	const int PWM_MAX = base_pwm;
	// Stop conditions (loosen slightly so it doesn’t immediately “done→brake”)
	const float DIFF_TOL_CM      = 0.20f;   // both walls
	const float DIST_TOL_CM      = 0.25f;   // single wall
	const uint32_t STABLE_DWELL_MS = 150;
	const int   ANG_TOL  = 12;

	// Seed targets from current readings (single-wall)
	update_sensors();
	//float target_left_cm  = lut_lookup_lin_local(sensors.side_left,  left_adc,  left_dist,  L_LUT_SIZE);
	//float target_right_cm = lut_lookup_lin_local(sensors.side_right, right_adc, right_dist, R_LUT_SIZE);

	float e_prev = 0.0f, d_filt = 0.0f, u_prev = 0.0f;
	uint32_t t0 = HAL_GetTick(), last = t0, last_ok = 0;
	bool Lw = sensors.wall_left;
	bool Rw = sensors.wall_right;
	bool FL= sensors.wall_frontL;
	bool FR= sensors.wall_frontR;


	if (!Lw && !Rw) { break_motors(); return false; }

	while ((HAL_GetTick() - t0) < timeout_ms) {
		uint32_t now = HAL_GetTick();
		float dt = (now - last) * 0.001f; if (dt <= 0.0f) dt = 0.001f;
		last = now;

		update_sensors();
		bool Lw = sensors.wall_left;
		bool Rw = sensors.wall_right;
		bool FL= sensors.wall_frontL;
		bool FR= sensors.wall_frontR;

		// Maintain requirement: at least one side wall; if both drop, stop
		if (!Lw && !Rw) { break_motors(); return false; }

		float Lcm = lut_lookup_lin_local(sensors.side_left,  left_adc,  left_dist,  L_LUT_SIZE);
		float Rcm = lut_lookup_lin_local(sensors.side_right, right_adc, right_dist, R_LUT_SIZE);


		// Error
		float e_cm, tol_cm;
		if (Lw && Rw) {
			e_cm  = (Rcm - Lcm);      // +e → need to turn right

		} else if (Lw) {
			e_cm  = (Lcm-target_left);   // +e if too near left wall


		} else { // Rw
			e_cm  = (Rcm - target_right);  // +e if too far from right wall

		}

		// PD
		float d_raw = (e_cm - e_prev) / dt;
		d_filt = D_ALPHA*d_filt + (1.0f - D_ALPHA)*d_raw;
		e_prev = e_cm;

		float u = KP_PWM_PER_CM*e_cm + KD_PWM_PER_CMPS*d_filt; // signed PWM

		u=clampi_local(u,  -PWM_MAX, PWM_MAX);

		bool ang_ok  = (abs((int)lroundf(e_cm*100))  <= ANG_TOL);

		if (!ang_ok) {
			if (u > 0  && u  < PWM_MIN_MOVE) u  = PWM_MIN_MOVE;
			if (u < 0  && -u < PWM_MIN_MOVE) u  = -PWM_MIN_MOVE;

		} else {
			// close enough: stop and actively brake so it doesn't coast/creep left
			u = 0;
		}

		if (u=0) break_motors();
		else{
			bool lfwd =(u>=0);
			uint16_t duty = (uint16_t)abs(u);
			motor_set(0, lfwd, duty);
			motor_set(1, !lfwd, duty);
		}


		// --- dwell-based success ---
		if (ang_ok) {
			if (last_ok == 0) last_ok = now;
			if ((now - last_ok) >= STABLE_DWELL_MS) {
				break_motors();
				return true;
			}
		} else {
			last_ok = 0;
		}


	}

	// timeout
	break_motors();
	return false;

}


bool fusion_align_entry(int base_pwm, uint32_t timeout_ms)
{
    // ---- Your gains (unchanged) ----

    const float Kp_a = 10.0f, Ki_a = 0.1f;   // angle PI

    // ---- Small bias to kill steady left drift (counts). Try 0, then -1 or -2 if it still nudges left. ----


    // ---- Finish criteria (unchanged) ----
    const int   DIST_TOL = 10;         // counts
    const int   ANG_TOL  = 12;         // counts
    const uint32_t STABLE_DWELL_MS = 150;

    // ---- Output constraints (unchanged idea) ----
    const int PWM_MAX = base_pwm;      // clamp final wheel cmds
    const int PWM_MIN_MOVE = 500;      // measured deadzone threshold

    // Integrators
    float I_d = 0.0f, I_a = 0.0f;

    uint32_t t0 = HAL_GetTick();
    uint32_t last_ok = 0;
    uint32_t last_tick = HAL_GetTick();

    update_sensors();
            //float target_left_cm  = lut_lookup_lin_local(sensors.side_left,  left_adc,  left_dist,  L_LUT_SIZE);
            //float target_right_cm = lut_lookup_lin_local(sensors.side_right, right_adc, right_dist, R_LUT_SIZE);

	float e_prev = 0.0f, d_filt = 0.0f, u_prev = 0.0f;
	bool Lw = sensors.wall_left;
	bool Rw = sensors.wall_right;
	bool FL= sensors.wall_frontL;
	bool FR= sensors.wall_frontR;

	if (!Lw && !Rw) { break_motors(); return false; }

    // reset motors
    motor_set(0, true, 0);
    motor_set(1, true, 0);

    while (1) {
        // --- timing / dt ---
        uint32_t now = HAL_GetTick();
        float dt = (now - last_tick) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        last_tick = now;

        // --- sensors ---
        update_sensors();

		bool Lw = sensors.wall_left;
		bool Rw = sensors.wall_right;
		bool FL= sensors.wall_frontL;
		bool FR= sensors.wall_frontR;

		// Maintain requirement: at least one side wall; if both drop, stop
//		if (!Lw && !Rw) { break_motors(); return false; }

		float Lcm = lut_lookup_lin_local(sensors.side_left,  left_adc,  left_dist,  L_LUT_SIZE);
		float Rcm = lut_lookup_lin_local(sensors.side_right, right_adc, right_dist, R_LUT_SIZE);


		// Error
		float e_ang;
		if (Lw && Rw) {
			//e_ang  = (Rcm - Lcm)*100;      // +e → need to turn right
			e_ang  = (Lcm-2.7)*100;

		} else if (Lw) {
			//e_ang  = (Lcm-target_left)*100;   // +e if too near left wall
			e_ang  = (Lcm-2.7)*100;


		} else { // Rw
			//e_ang  = (Rcm - target_right)*100;  // +e if too far from right wall
			e_ang  = (Rcm - 2.7)*100;

		}


        // --- PI controllers ---

        I_a += e_ang  * dt;

        // simple clamps to keep integrators sane
        if (I_a > 100.0f) I_a = 100.0f;
        if (I_a < -100.0f) I_a = -100.0f;

        float w = Kp_a * e_ang  + Ki_a * I_a;  // turn command          (- = turn right, + = left)
        int cmd_left;
        int cmd_right;

        // per-wheel raw commands (signed) — keep your mixing/signs
        if (Lw){
            cmd_left  = (int)lroundf(-w);
            cmd_right = (int)lroundf(w);
        }else if (Rw){
            cmd_left  = (int)lroundf(w);
            cmd_right = (int)lroundf(-w);
        }


        // saturate
        cmd_left  = clampi_local(cmd_left,  -PWM_MAX, PWM_MAX);
        cmd_right = clampi_local(cmd_right, -PWM_MAX, PWM_MAX);

        // --- convergence check *before* applying min-move ---
        bool ang_ok  = (abs((int)lroundf(e_ang))  <= ANG_TOL);
        bool nearly_done = (ang_ok);

        // --- stiction handling ---
        // If we're NOT nearly done, enforce a minimum to break deadzone.
        // If we ARE nearly done, DON'T enforce min move — brake instead to avoid creeping.
        if (!nearly_done) {
            if (cmd_left > 0  && cmd_left  < PWM_MIN_MOVE) cmd_left  = PWM_MIN_MOVE;
            if (cmd_left < 0  && -cmd_left < PWM_MIN_MOVE) cmd_left  = -PWM_MIN_MOVE;
            if (cmd_right > 0 && cmd_right < PWM_MIN_MOVE) cmd_right = PWM_MIN_MOVE;
            if (cmd_right < 0 && -cmd_right < PWM_MIN_MOVE) cmd_right = -PWM_MIN_MOVE;
        } else {
            // close enough: stop and actively brake so it doesn't coast/creep left
            cmd_left = 0;
            cmd_right = 0;
        }

        // --- drive / brake ---
        if (cmd_left == 0 && cmd_right == 0) {

            break_motors();  // actively short the motors to kill drift
        } else {
            bool lfwd = (cmd_left  >= 0);
            bool rfwd = (cmd_right >= 0);
            uint16_t lduty = (uint16_t)abs(cmd_left);
            uint16_t rduty = (uint16_t)abs(cmd_right);
            motor_set(0, lfwd, lduty);
            motor_set(1, rfwd, rduty);

        }

        // --- dwell-based success ---
        if (nearly_done) {
            if (last_ok == 0) last_ok = now;
            if ((now - last_ok) >= STABLE_DWELL_MS) {
                break_motors();
                return true;
            }
        } else {
            last_ok = 0;
        }

        // --- timeout ---
        if ((now - t0) > timeout_ms) {
            break_motors();
            return false;
        }
    }
}
