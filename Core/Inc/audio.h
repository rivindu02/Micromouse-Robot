
#ifndef AUDIO_H
#define AUDIO_H
#include "micromouse.h"
void play_tone(uint16_t frequency, uint16_t duration_ms);
void speaker_off(void);
void play_startup_tone(void);
void play_confirmation_tone(void);
void play_success_tone(void);
void play_error_tone(void);
void play_wall_beep(void);
void play_turn_beep(void);
void play_battery_warning(void);
#endif
