/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DRDY_Pin GPIO_PIN_2
#define DRDY_GPIO_Port GPIOE
#define DRDY_EXTI_IRQn EXTI2_TSC_IRQn
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define MEMS_INT3_Pin GPIO_PIN_4
#define MEMS_INT3_GPIO_Port GPIOE
#define MEMS_INT4_Pin GPIO_PIN_5
#define MEMS_INT4_GPIO_Port GPIOE
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MISOA7_Pin GPIO_PIN_7
#define SPI1_MISOA7_GPIO_Port GPIOA
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOE
#define LD5_Pin GPIO_PIN_10
#define LD5_GPIO_Port GPIOE
#define LD7_Pin GPIO_PIN_11
#define LD7_GPIO_Port GPIOE
#define LD9_Pin GPIO_PIN_12
#define LD9_GPIO_Port GPIOE
#define LD10_Pin GPIO_PIN_13
#define LD10_GPIO_Port GPIOE
#define LD8_Pin GPIO_PIN_14
#define LD8_GPIO_Port GPIOE
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOE
#define DM_Pin GPIO_PIN_11
#define DM_GPIO_Port GPIOA
#define DP_Pin GPIO_PIN_12
#define DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define MEMS_INT1_Pin GPIO_PIN_0
#define MEMS_INT1_GPIO_Port GPIOE
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* Accelerometer defines */
#define I2C_ADDR_READ_ACCELEROMETER (0x32)

#define LSM303DLHC_WHO_AM_I_ADDR 0x0F

#define I2C_ADDR_OUT_X_L_A 0x28
#define I2C_ADDR_OUT_X_H_A 0x29
#define I2C_ADDR_OUT_Y_L_A 0x2A
#define I2C_ADDR_OUT_Y_H_A 0x2B
#define I2C_ADDR_OUT_Z_L_A 0x2C
#define I2C_ADDR_OUT_Z_H_A 0x2D

/* Magnetometer defines */
#define I2C_ADDR_READ_MAGNETOMETER 0x3C

#define I2C_ADDR_WHOAMI            0x4F

#define I2C_ADDR_STATUS_REG_M      0x67
#define I2C_ADDR_OUTX_L_REG_M      0x68
#define I2C_ADDR_OUTX_H_REG_M      0x69
#define I2C_ADDR_OUTY_L_REG_M      0x6A
#define I2C_ADDR_OUTY_H_REG_M      0x6B
#define I2C_ADDR_OUTZ_L_REG_M      0x6C
#define I2C_ADDR_OUTZ_H_REG_M      0x6D


#define I2C_ADDR_OFFSET_X_REG_L_M  0x45
#define I2C_ADDR_OFFSET_X_REG_H_M  0x46
#define I2C_ADDR_OFFSET_Y_REG_L_M  0x47
#define I2C_ADDR_OFFSET_Y_REG_H_M  0x48
#define I2C_ADDR_OFFSET_Z_REG_L_M  0x49
#define I2C_ADDR_OFFSET_Z_REG_H_M  0x4a

#define I2C_ADDR_CFG_REG_A_M       0x60
#define I2C_ADDR_CFG_REG_C_M       0x62
#define I2C_ADDR_INT_CTRL_REG_M    0x63


#define I2C_ADDR_CTRL_REG1_A       0x20

#define I2C_RESULT_WHOAMI  0x40


#define PWM_PERIODS 100
#define LED_CNT     8

void data_ready_interrupt(void);

typedef struct led_pwm_desc_s
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin;
  GPIO_PinState PinState;
  uint8_t duty_cycle_percent;
} led_pwm_desc_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
