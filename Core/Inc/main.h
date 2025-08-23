/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_STBY_Pin GPIO_PIN_13
#define MOTOR_STBY_GPIO_Port GPIOC
#define Battery_Voltage_Pin GPIO_PIN_0
#define Battery_Voltage_GPIO_Port GPIOA
#define BTN_LEFT_Pin GPIO_PIN_1
#define BTN_LEFT_GPIO_Port GPIOA
#define BTN_LEFT_EXTI_IRQn EXTI1_IRQn
#define Front_Right_Receiver_Pin GPIO_PIN_2
#define Front_Right_Receiver_GPIO_Port GPIOA
#define Side_Right_Receiver_Pin GPIO_PIN_3
#define Side_Right_Receiver_GPIO_Port GPIOA
#define Side_Left_Receiver_Pin GPIO_PIN_4
#define Side_Left_Receiver_GPIO_Port GPIOA
#define Front_Left_Receiver_Pin GPIO_PIN_5
#define Front_Left_Receiver_GPIO_Port GPIOA
#define MOTOR_IN1_Pin GPIO_PIN_6
#define MOTOR_IN1_GPIO_Port GPIOA
#define MOTOR_IN2_Pin GPIO_PIN_7
#define MOTOR_IN2_GPIO_Port GPIOA
#define MOTOR_IN3_Pin GPIO_PIN_0
#define MOTOR_IN3_GPIO_Port GPIOB
#define MOTOR_IN4_Pin GPIO_PIN_1
#define MOTOR_IN4_GPIO_Port GPIOB
#define BTN_RIGHT_Pin GPIO_PIN_10
#define BTN_RIGHT_GPIO_Port GPIOB
#define BTN_RIGHT_EXTI_IRQn EXTI15_10_IRQn
#define Chip_Select_Pin GPIO_PIN_12
#define Chip_Select_GPIO_Port GPIOB
#define Gyro_SCL_Pin GPIO_PIN_13
#define Gyro_SCL_GPIO_Port GPIOB
#define Gyro_ADO_Pin GPIO_PIN_14
#define Gyro_ADO_GPIO_Port GPIOB
#define Gyro_SDA_Pin GPIO_PIN_15
#define Gyro_SDA_GPIO_Port GPIOB
#define EMIT_FRONT_RIGHT_Pin GPIO_PIN_8
#define EMIT_FRONT_RIGHT_GPIO_Port GPIOA
#define EMIT_SIDE_LEFT_Pin GPIO_PIN_9
#define EMIT_SIDE_LEFT_GPIO_Port GPIOA
#define speaker_PWM_Pin GPIO_PIN_10
#define speaker_PWM_GPIO_Port GPIOA
#define Bluetooth_TX_Pin GPIO_PIN_11
#define Bluetooth_TX_GPIO_Port GPIOA
#define Bluetooth_RX_Pin GPIO_PIN_12
#define Bluetooth_RX_GPIO_Port GPIOA
#define Left_EncoderA_Pin GPIO_PIN_15
#define Left_EncoderA_GPIO_Port GPIOA
#define Left_EncoderB_Pin GPIO_PIN_3
#define Left_EncoderB_GPIO_Port GPIOB
#define LED_LEFT_Pin GPIO_PIN_4
#define LED_LEFT_GPIO_Port GPIOB
#define LED_RIGHT_Pin GPIO_PIN_5
#define LED_RIGHT_GPIO_Port GPIOB
#define Right_EncoderA_Pin GPIO_PIN_6
#define Right_EncoderA_GPIO_Port GPIOB
#define Right_EncoderB_Pin GPIO_PIN_7
#define Right_EncoderB_GPIO_Port GPIOB
#define EMIT_SIDE_RIGHT_Pin GPIO_PIN_8
#define EMIT_SIDE_RIGHT_GPIO_Port GPIOB
#define EMIT_FRONT_LEFT_Pin GPIO_PIN_9
#define EMIT_FRONT_LEFT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
