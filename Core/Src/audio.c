/*
 * audio.c - Audio feedback and speaker control
 *
 * Provides audio feedback for different micromouse states
 * Using TIM1_CH3 PWM on PA10 with prescaler=20, period=200
 */

#include "micromouse.h"

/**
 * @brief Play a tone of specific frequency and duration
 */
void play_tone(uint16_t frequency, uint16_t duration_ms)
{
    if (frequency == 0) {
        speaker_off();
        HAL_Delay(duration_ms);
        return;
    }

    // Calculate period for desired frequency
    // Timer freq = 84MHz / (prescaler + 1) = 84MHz / 21 = 4MHz
    // Period = Timer_freq / desired_freq = 4000000 / frequency
    uint32_t period = 4000000 / frequency;
    if (period > 65535) period = 65535; // Clamp to 16-bit
    if (period < 20) period = 20;       // Minimum period

    // Update timer period
    __HAL_TIM_SET_AUTORELOAD(&htim1, period - 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, period / 2); // 50% duty cycle

    // Start PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // Play for specified duration
    HAL_Delay(duration_ms);

    // Stop PWM
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief Turn off speaker
 */
void speaker_off(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief Play startup tone sequence
 */
void play_startup_tone(void)
{
    play_tone(523, 200);  // C5
    play_tone(0, 50);     // Pause
    play_tone(659, 200);  // E5
    play_tone(0, 50);     // Pause
    play_tone(784, 300);  // G5
}

/**
 * @brief Play confirmation tone
 */
void play_confirmation_tone(void)
{
    play_tone(784, 150);  // G5
    play_tone(0, 50);     // Pause
    play_tone(1047, 200); // C6
}

/**
 * @brief Play success tone sequence
 */
void play_success_tone(void)
{
    play_tone(523, 100);  // C5
    play_tone(659, 100);  // E5
    play_tone(784, 100);  // G5
    play_tone(1047, 200); // C6
    play_tone(0, 100);    // Pause
    play_tone(1047, 100); // C6
    play_tone(784, 100);  // G5
    play_tone(1047, 300); // C6
}

/**
 * @brief Play error tone sequence
 */
void play_error_tone(void)
{
    for (int i = 0; i < 3; i++) {
        play_tone(220, 200);  // A3
        play_tone(0, 100);    // Pause
    }
}

/**
 * @brief Play wall detection beep
 */
void play_wall_beep(void)
{
    play_tone(1000, 50);
}

/**
 * @brief Play turn signal beep
 */
void play_turn_beep(void)
{
    play_tone(800, 30);
}

/**
 * @brief Play battery low warning
 */
void play_battery_warning(void)
{
    for (int i = 0; i < 5; i++) {
        play_tone(440, 100);  // A4
        play_tone(0, 100);    // Pause
    }
}
