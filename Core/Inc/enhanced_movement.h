/*
 * enhanced_movement.h
 *
 *  Created on: Aug 22, 2025
 *      Author: ASUS
 */

#ifndef INC_ENHANCED_MOVEMENT_H_
#define INC_ENHANCED_MOVEMENT_H_

#include "micromouse.h"
void move_forward_scurve(float distance_mm, float speed_multiplier);
void turn_scurve(int turn_direction);
void move_forward_cell_scurve(void);
void turn_left_scurve(void);
void turn_right_scurve(void);
void turn_around_scurve(void);
void move_forward_adaptive_scurve(float speed_multiplier);
void send_scurve_movement_status(void);


#endif /* INC_ENHANCED_MOVEMENT_H_ */
