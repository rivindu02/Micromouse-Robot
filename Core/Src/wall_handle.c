/*
 * turns.c
 *
 *  Created on: Oct 3, 2025
 *      Author: ASUS
 */


#include "micromouse.h"
#include "movement.h"


// --- Front-wall alignment (PI + PI)---
// Helper: clamp
static inline int clampi_local(int v, int lo, int hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

#define target_align 72



/**
 * Align normal to the front wall using FL/FR sensors.
 *
 * target_FL/target_FR : your desired "normal to wall" readings for FL/FR
 * base_pwm            : nominal wheel headroom (used as limit; commands saturate to +/-base_pwm)
 * timeout_ms          : safety timeout
 *
 * returns true on success (converged), false on timeout
 */
bool align_front_to_wall(int base_pwm, uint32_t timeout_ms)
{
    // --- Gains (start conservative; tune on-floor) ---
    // Distance loop (forward/back) acts on average error
    const float Kp_d = 0.50f, Ki_d = 0.02f;    // no D: IR noise/quantization
    // Angle loop (turn) acts on left-right difference
    const float Kp_a = 1.10f, Ki_a = 0.03f;    // slightly higher P than distance

    // --- Finish criteria ---
    const int   DIST_TOL = 10;     // adjust to your sensor units
    const int   ANG_TOL  = 12;     // "
    const uint32_t STABLE_DWELL_MS = 150;

    // --- Output constraints ---
    const int PWM_MAX = base_pwm;     // map control output into [-base_pwm .. +base_pwm]
    const int PWM_MIN_MOVE = 500;      // overcome stiction when non-zero

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

        // --- update sensors ---
        update_sensors();  // provides sensors.front_left/right etc. :contentReference[oaicite:6]{index=6}

        int FL = (int)sensors.front_left;   // :contentReference[oaicite:7]{index=7}
        int FR = (int)sensors.front_right;  // :contentReference[oaicite:8]{index=8}

        // --- compute errors ---
        int eL = FL - (int)target_align;
        int eR = FR - (int)target_align;

        // distance = average error (drive until correct range)
        float e_dist = 0.5f * (eL + eR);
        // angle    = difference (turn until FL≈FR at that distance)
        float e_ang  = (float)(eL - eR);

        // --- PI controllers ---
        I_d += e_dist * dt;
        I_a += e_ang  * dt;

        // anti-windup clamp on the integrators (keep reasonable)
        if (I_d > 1500.0f) I_d = 1500.0f;
        if (I_d < -1500.0f) I_d = -1500.0f;

        if (I_a > 1500.0f) I_a = 1500.0f;
        if (I_a < -1500.0f) I_a = -1500.0f;

        float v = Kp_d * e_dist + Ki_d * I_d;  // forward/back command  (- = back, + = forward)
        float w = Kp_a * e_ang  + Ki_a * I_a;  // turn command          (- = turn right, + = left)

        // per-wheel raw commands (signed)
        int cmd_left  = (int)lroundf(-v - w);
        int cmd_right = (int)lroundf(-v + w);

        // saturate
        cmd_left  = clampi_local(cmd_left,  -PWM_MAX, PWM_MAX);
        cmd_right = clampi_local(cmd_right, -PWM_MAX, PWM_MAX);

        // apply a small minimum when non-zero to overcome stiction
        if (cmd_left > 0  && cmd_left  < PWM_MIN_MOVE) cmd_left  = PWM_MIN_MOVE;
        if (cmd_left < 0  && -cmd_left < PWM_MIN_MOVE) cmd_left  = -PWM_MIN_MOVE;
        if (cmd_right > 0 && cmd_right < PWM_MIN_MOVE) cmd_right = PWM_MIN_MOVE;
        if (cmd_right < 0 && -cmd_right < PWM_MIN_MOVE) cmd_right = -PWM_MIN_MOVE;

        // --- drive motors via your API ---
        // motor_set(motor_index, forward, duty) — you already have this. :contentReference[oaicite:9]{index=9}
        bool lfwd = (cmd_left  >= 0);
        bool rfwd = (cmd_right >= 0);
        uint16_t lduty = (uint16_t)abs(cmd_left);
        uint16_t rduty = (uint16_t)abs(cmd_right);

        motor_set(0, lfwd, lduty);   // Left  :contentReference[oaicite:10]{index=10}
        motor_set(1, rfwd, rduty);   // Right :contentReference[oaicite:11]{index=11}

        // --- convergence check with dwell ---
        bool dist_ok = (abs((int)e_dist) <= DIST_TOL);
        bool ang_ok  = (abs((int)e_ang)  <= ANG_TOL);

        if (dist_ok && ang_ok) {
            if (last_ok == 0) last_ok = now;
            if ((now - last_ok) >= STABLE_DWELL_MS) {
                // stop cleanly
                motor_set(0, true, 0);
                motor_set(1, true, 0);
                return true;
            }
        } else {
            last_ok = 0;
        }

        // --- timeout ---
        if ((now - t0) > timeout_ms) {
            motor_set(0, true, 0);
            motor_set(1, true, 0);
            return false;
        }

        // small loop delay to keep CPU sane
        // (your system already runs ~1kHz loops elsewhere; adjust if needed)
        // HAL_Delay(1); // optional
    }
}






// ================== WALL FOLLOW PID (ADD) ===================

// ---------- Tunables ----------
static int   WF_BASE_PWM        = 500;     // cruise PWM
static int   WF_PWM_MIN_MOVE    = 50;      // overcome stiction
static int   WF_PWM_MAX         = 1000;    // clamp

// PID gains (your tuned values)
static float WF_KP = 0.25f;
static float WF_KI = 0.0f;
static float WF_KD = 0.004f;
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
// Right sensor LUT
#define R_LUT_SIZE 33
static const int   right_adc[R_LUT_SIZE]   = {43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11};
static const float right_dist[R_LUT_SIZE]  = {1.17,1.23,1.29,1.36,1.43,1.51,1.59,1.67,1.76,1.85,1.95,2.05,2.16,2.28,2.40,2.53,2.67,2.82,2.98,3.16,3.34,3.55,3.76,4.00,4.26,4.54,4.85,5.19,5.57,5.98,6.45,6.96};

// Left sensor LUT
#define L_LUT_SIZE 32
static const int   left_adc[L_LUT_SIZE]    = {42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11};
static const float left_dist[L_LUT_SIZE]   = {1.17f,1.23f,1.29f,1.36f,1.43f,1.51f,1.59f,1.67f,1.76f,1.85f,1.95f,2.05f,2.16f,2.28f,2.40f,2.53f,2.67f,2.82f,2.98f,3.16f,3.34f,3.55f,3.76f,4.00f,4.26f,4.54f,4.85f,5.19f,5.57f,5.98f,6.45f,6.96f};

// ---------- Helper: Linear interpolation lookup ----------
static float lut_lookup(int raw, const int *adc_table, const float *dist_table, int size)
{
    // Clamp below/above table
    if (raw >= adc_table[0]) return dist_table[0];
    if (raw <= adc_table[size-1]) return dist_table[size-1];

    // Find interval [i, i+1] where raw fits
    for (int i=0; i<size-1; i++) {
        if (raw <= adc_table[i] && raw >= adc_table[i+1]) {
            float t = (float)(raw - adc_table[i+1]) / (float)(adc_table[i] - adc_table[i+1]);
            return dist_table[i+1] + t * (dist_table[i] - dist_table[i+1]);
        }
    }
    return (float)raw; // fallback (should not happen)
}


// ---------- Internal state ----------
typedef enum { WF_AUTO=0, WF_LEFT, WF_RIGHT } wf_mode_t;
static wf_mode_t wf_mode = WF_AUTO;

static float e_int = 0.0f, e_prev = 0.0f, d_filt = 0.0f;
static uint32_t wf_last_ms = 0;

static float target_left  = 2.50f;   // desired left wall distance (cm)
static float target_right = 2.50f;   // desired right wall distance (cm)

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float clampf(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v); }

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
    float e = 0.0f;

    if (Lw && Rw) {
    	// Both walls: balance distances
    	float L = lut_lookup(sensors.side_left,  left_adc,  left_dist,  L_LUT_SIZE);
    	e = target_left - L;
//    	float R = lut_lookup(sensors.side_right, right_adc, right_dist, R_LUT_SIZE);
//    	e = WF_BOTH_SCALE * (L - R);
        // keep single-wall targets gently aligned to present gap
        //target_left  = (1.0f - WF_SINGLE_ALPHA)*target_left  + WF_SINGLE_ALPHA*L;
        //target_right = (1.0f - WF_SINGLE_ALPHA)*target_right + WF_SINGLE_ALPHA*R;

    } else if (Lw) {
    	// Left wall only: hold target distance
    	float L = lut_lookup(sensors.side_left, left_adc, left_dist, L_LUT_SIZE);
    	e = target_left - L;

    } else if (Rw) {
    	// Right wall only: hold target distance
    	float R = lut_lookup(sensors.side_right, right_adc, right_dist, R_LUT_SIZE);
    	e = R - target_right;

    } else {
        // No side walls -> no correction (let heading/gyro PID handle straightness if you run it)
        e = 0.0f;
    }

    // PID on error
    e_int += e * dt;
    e_int  = clampf(e_int, -WF_INT_LIMIT, WF_INT_LIMIT);

    float d_raw = (e - e_prev) / dt;
    d_filt = WF_DERIV_ALPHA * d_filt + (1.0f - WF_DERIV_ALPHA) * d_raw;

    float u_norm = WF_KP*e + WF_KI*e_int + WF_KD*d_filt;  // u > 0 => speed up right / slow left
    float u = u_norm * WF_U_SCALE;
    e_prev = e;

    // Front wall policy
    int base = WF_BASE_PWM;
    if (Fw && WF_BRAKE_ON_FRONT) {
        base = WF_SLOW_PWM;
        // If you want a hard stop, uncomment:
        // motor_set(0, true, 0); motor_set(1, true, 0); HAL_Delay(WF_FRONT_HOLD_MS); return;
    }

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

}




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

static float fus_theta = 0.0f;          // integrated heading (deg)
static float fus_theta_ref = 0.0f;      // heading lock for current straight
static float fus_conf_s = 0.0f;         // smoothed wall confidence
static uint32_t fus_last_ms = 0;
static float fus_u_prev = 0.0f;         // rate limit state

// small heading-PID locals (assist only)
static float h_int = 0.0f, h_prev = 0.0f, h_df = 0.0f;

// knobs (not “tuning” — just safety rails)
static const float FUS_CONF_EMA        = 0.90f;  // confidence smoothing
static const float FUS_HEAD_CAP_FRAC   = 0.25f;  // max heading authority (fraction of base)
static const float FUS_U_RATE_LIM      = 120.0f; // max |Δu| per step (PWM units)

extern float Kp_g, Ki_g, Kd_g;                 // your gyro PID gains

void fusion_reset(void)
{
    // reset wall PID memory (reuse your existing state)
    e_int = 0.0f; e_prev = 0.0f; d_filt = 0.0f;
    wf_last_ms = HAL_GetTick();

    // reset fusion/heading memory
    fus_theta = 0.0f;
    fus_theta_ref = 0.0f;
    fus_conf_s = 0.0f;
    fus_u_prev = 0.0f;
    h_int = 0.0f; h_prev = 0.0f; h_df = 0.0f;
    fus_last_ms = HAL_GetTick();

    // capture initial targets to avoid a jump at start
    // for the Target, values should be updated -------------------->
    update_sensors();
    //target_left  = (float)sensors.side_left;
    //target_right = (float)sensors.side_right;
}

void fusion_set_heading_ref_to_current(void)
{
    fus_theta_ref = fus_theta;
}

// Call at ~200–500 Hz. Pass 0 to use WF_BASE_PWM.
void fusion_step(int base_pwm)
{
    // --- timing ---
    uint32_t now = HAL_GetTick();
    float dt = (now - fus_last_ms) * 0.001f;
    if (dt <= 0.0f) dt = 0.001f;
    fus_last_ms = now;

    // --- sensors + gyro ---
    update_sensors();
    bool Lw = sensors.wall_left;
    bool Rw = sensors.wall_right;
    bool Fw = sensors.wall_front;

    int  L = sensors.side_left;
    int  R = sensors.side_right;

    mpu9250_read_gyro();
    float gz = mpu9250_get_gyro_z_compensated();   // deg/s
    fus_theta += gz * dt;

    // -------- WALL PID (log-ratio + your WF_* state) --------
    float e_wall = 0.0f;
    if (Lw && Rw) {
        e_wall = WF_BOTH_SCALE * (logf((float)L + 1.0f) - logf((float)R + 1.0f));
        // keep single-wall targets gently aligned (same as wall_follow_step)
        //target_left  = (1.0f - WF_SINGLE_ALPHA)*target_left  + WF_SINGLE_ALPHA*(float)L;
        //target_right = (1.0f - WF_SINGLE_ALPHA)*target_right + WF_SINGLE_ALPHA*(float)R;
    } else if (Lw) {
        //target_left  = (1.0f - WF_SINGLE_ALPHA)*target_left  + WF_SINGLE_ALPHA*(float)L;
        e_wall = logf((float)L + 1.0f) - logf(target_left + 1.0f);
    } else if (Rw) {
        //target_right = (1.0f - WF_SINGLE_ALPHA)*target_right + WF_SINGLE_ALPHA*(float)R;
        e_wall = logf(target_right + 1.0f) - logf((float)R + 1.0f);
    } else {
        e_wall = 0.0f;
    }

    // step the SAME wall PID states/gains
    e_int += e_wall * dt;
    e_int  = clampf(e_int, -WF_INT_LIMIT, WF_INT_LIMIT);
    float d_raw = (e_wall - e_prev) / dt;
    d_filt = WF_DERIV_ALPHA * d_filt + (1.0f - WF_DERIV_ALPHA) * d_raw;
    float u_wall = WF_KP*e_wall + WF_KI*e_int + WF_KD*d_filt;
    e_prev = e_wall;

    // -------- HEADING PID (assist; reuses your gyro PID gains) --------
    float e_head = fus_theta_ref - fus_theta;

    // allow heading integrator only when walls are NOT present
    if (!(Lw || Rw)) {
        h_int += e_head * dt;
        if (h_int > 200.0f) h_int = 200.0f;
        if (h_int < -200.0f) h_int = -200.0f;
    }

    const float H_ALPHA = 0.90f;                  // small derivative filter
    float h_draw = (e_head - h_prev) / dt;
    h_df = H_ALPHA*h_df + (1.0f - H_ALPHA)*h_draw;
    h_prev = e_head;

    float u_head = Kp_g*e_head + Ki_g*h_int + Kd_g*h_df;

    // limit heading authority so it never fights good wall info
    int base_unclamped = (base_pwm > 0) ? base_pwm : WF_BASE_PWM;
    float head_cap = FUS_HEAD_CAP_FRAC * (float)base_unclamped;   // e.g., 25% of base
    if (u_head >  head_cap) u_head =  head_cap;
    if (u_head < -head_cap) u_head = -head_cap;

    // -------- BLEND (confidence from side walls) --------
    float conf = 0.0f; if (Lw) conf += 0.5f; if (Rw) conf += 0.5f;
    fus_conf_s = FUS_CONF_EMA*fus_conf_s + (1.0f - FUS_CONF_EMA)*conf;  // smooth handoffs
    float u = fus_conf_s*u_wall + (1.0f - fus_conf_s)*u_head;

    // -------- OUTPUT SHAPING --------
    // optional rate limit on correction to avoid jerk
    float du = u - fus_u_prev;
    if (du >  FUS_U_RATE_LIM) du =  FUS_U_RATE_LIM;
    if (du < -FUS_U_RATE_LIM) du = -FUS_U_RATE_LIM;
    u = fus_u_prev + du;
    fus_u_prev = u;

    int base = (base_pwm > 0) ? base_pwm : WF_BASE_PWM;
    if (Fw && WF_BRAKE_ON_FRONT) base = WF_SLOW_PWM;

    // right = base + u, left = base - u
    int pwm_right = clampi((int)lroundf((float)base + u), 0, WF_PWM_MAX);
    int pwm_left  = clampi((int)lroundf((float)base - u), 0, WF_PWM_MAX);

    if (pwm_right > 0 && pwm_right < WF_PWM_MIN_MOVE) pwm_right = WF_PWM_MIN_MOVE;
    if (pwm_left  > 0 && pwm_left  < WF_PWM_MIN_MOVE) pwm_left  = WF_PWM_MIN_MOVE;

    motor_set(0, true, (uint16_t)pwm_left);
    motor_set(1, true, (uint16_t)pwm_right);
}


