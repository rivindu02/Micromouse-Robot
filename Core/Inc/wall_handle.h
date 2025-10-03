/*
 * wall_handle.h
 *
 *  Created on: Oct 3, 2025
 *      Author: ASUS
 */

#ifndef INC_WALL_HANDLE_H_
#define INC_WALL_HANDLE_H_


bool align_front_to_wall(int base_pwm, uint32_t timeout_ms);
void wall_follow_reset_int(int mode, int base_pwm);
void wall_follow_step(void);
void fusion_reset(void);
void fusion_set_heading_ref_to_current(void);
void fusion_step(int base_pwm);



#endif /* INC_WALL_HANDLE_H_ */
