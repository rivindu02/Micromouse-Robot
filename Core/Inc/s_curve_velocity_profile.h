#ifndef S_CURVE_VELOCITY_PROFILE_H
#define S_CURVE_VELOCITY_PROFILE_H

#include "micromouse.h" // brings SCurveProfile definition

#ifdef __cplusplus
extern "C" {
#endif

void scurve_profile_init(SCurveProfile* p, float distance_mm, float vmax, float amax, float jmax);
void scurve_profile_reset_time(SCurveProfile* p);
void scurve_profile_update(SCurveProfile* p);            // integrates by real dt
float scurve_profile_get_velocity(SCurveProfile* p);      // target feedforward speed
bool  scurve_profile_is_complete(SCurveProfile* p);
float scurve_profile_get_position(SCurveProfile* p);
void scurve_profile_reset(SCurveProfile* p);
void scurve_profile_send_status(SCurveProfile* p);

void scurve_profile_init_cell_movement(SCurveProfile* p, float speed_multiplier);
void scurve_profile_init_turn_movement(SCurveProfile* p, float angle_degrees);

#ifdef __cplusplus
}
#endif

#endif // S_CURVE_VELOCITY_PROFILE_H
