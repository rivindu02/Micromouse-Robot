/*
 * stm32f4xx_hal.h - host stub used only by the maze simulator (tools/sim)
 *
 * Provides just enough of the HAL types and macros for the firmware's
 * maze logic, telemetry, audio and utility files to compile on a PC.
 * Timing calls advance a simulated clock instead of waiting.
 */

#ifndef SIM_STM32F4XX_HAL_H
#define SIM_STM32F4XX_HAL_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef enum { EXTI1_IRQn = 7, EXTI15_10_IRQn = 40 } IRQn_Type;

typedef struct { uint32_t dummy; } GPIO_TypeDef;
typedef struct { uint32_t State; } ADC_HandleTypeDef;
typedef struct { uint32_t dummy; } SPI_HandleTypeDef;
typedef struct { uint32_t dummy; } TIM_HandleTypeDef;
typedef struct { uint32_t dummy; } UART_HandleTypeDef;

extern GPIO_TypeDef sim_gpioa, sim_gpiob, sim_gpioc;
#define GPIOA (&sim_gpioa)
#define GPIOB (&sim_gpiob)
#define GPIOC (&sim_gpioc)

#define GPIO_PIN_0  0x0001U
#define GPIO_PIN_1  0x0002U
#define GPIO_PIN_2  0x0004U
#define GPIO_PIN_3  0x0008U
#define GPIO_PIN_4  0x0010U
#define GPIO_PIN_5  0x0020U
#define GPIO_PIN_6  0x0040U
#define GPIO_PIN_7  0x0080U
#define GPIO_PIN_8  0x0100U
#define GPIO_PIN_9  0x0200U
#define GPIO_PIN_10 0x0400U
#define GPIO_PIN_11 0x0800U
#define GPIO_PIN_12 0x1000U
#define GPIO_PIN_13 0x2000U
#define GPIO_PIN_14 0x4000U
#define GPIO_PIN_15 0x8000U

#define TIM_CHANNEL_1 0x0U
#define TIM_CHANNEL_2 0x4U
#define TIM_CHANNEL_3 0x8U
#define TIM_CHANNEL_4 0xCU

/* Cortex-M debug cycle counter used by dwt_delay_us() */
typedef struct { volatile uint32_t CTRL; volatile uint32_t CYCCNT; } DWT_Type;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
extern DWT_Type sim_dwt;
extern CoreDebug_Type sim_coredebug;
#define DWT (&sim_dwt)
#define CoreDebug (&sim_coredebug)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define CoreDebug_DEMCR_TRCENA_Msk (1U << 24)
#define __NOP() (sim_dwt.CYCCNT += 1000U)

void HAL_Delay(uint32_t ms);
uint32_t HAL_GetTick(void);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *h, uint32_t channel);

#define __HAL_TIM_SET_AUTORELOAD(h, v) ((void)(h), (void)(v))
#define __HAL_TIM_SET_COMPARE(h, ch, v) ((void)(h), (void)(ch), (void)(v))
#define __HAL_TIM_GET_COUNTER(h) (0U)

#endif /* SIM_STM32F4XX_HAL_H */
