#include "main.h"
#include "compass.h"

#include <stdbool.h>
#include <math.h>


#define CONCAT_8BIT_INTO_16BIT(high_byte, low_byte)  (((high_byte) << 8U) | low_byte)
#define CONVERSE_16BIT_TO_32BIT(value)               ((value) - (0xffff) / 2)
#define MAX(a, b)                                    ((a) = (b) > (a) ? (a):(a))
#define MIN(a, b)                                    ((a) = (b) < (a) ? (b):(a))

#define MAGN_VALUE_TO_PERSENT(val) ((0xffff - (val)) / 10)



static led_pwm_desc_t g_led_states[] =
{
  {
    .GPIOx = LD3_GPIO_Port,
    .GPIO_Pin = LD3_Pin,
    .PinState = GPIO_PIN_RESET,
    .duty_cycle_percent = 0
  },
  {
    .GPIOx = LD4_GPIO_Port,
    .GPIO_Pin = LD4_Pin,
    .PinState = GPIO_PIN_RESET,
    .duty_cycle_percent = 0
  },
  {
    .GPIOx = LD5_GPIO_Port,
    .GPIO_Pin = LD5_Pin,
    .PinState = GPIO_PIN_RESET,
    .duty_cycle_percent = 0
  },
  {
    .GPIOx = LD6_GPIO_Port,
    .GPIO_Pin = LD6_Pin,
    .PinState = GPIO_PIN_RESET,
    .duty_cycle_percent = 0
  },
  {
    .GPIOx = LD7_GPIO_Port,
    .GPIO_Pin = LD7_Pin,
    .PinState = GPIO_PIN_RESET,
    .duty_cycle_percent = 0
  },
  {
    .GPIOx = LD8_GPIO_Port,
    .GPIO_Pin = LD8_Pin,
    .PinState = GPIO_PIN_RESET,
    .duty_cycle_percent = 0
  },
  {
    .GPIOx = LD9_GPIO_Port,
    .GPIO_Pin = LD9_Pin,
    .PinState = GPIO_PIN_RESET,
    .duty_cycle_percent = 0
  },
  {
    .GPIOx = LD10_GPIO_Port,
    .GPIO_Pin = LD10_Pin,
    .PinState = GPIO_PIN_RESET,
    .duty_cycle_percent = 0
  }
};

static uint8_t g_reading_steps[] =
{
  I2C_ADDR_OUTX_L_REG_M,
  I2C_ADDR_OUTX_H_REG_M,
  I2C_ADDR_OUTY_L_REG_M,
  I2C_ADDR_OUTY_H_REG_M,
  I2C_ADDR_OUTZ_L_REG_M,
  I2C_ADDR_OUTZ_H_REG_M
};

void compass_init(void)
{
  HAL_StatusTypeDef status;
  uint8_t value = 0;

  status = HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_READ_MAGNETOMETER, I2C_ADDR_WHOAMI, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);

  if (status != HAL_OK || value != I2C_RESULT_WHOAMI)
  {
    Error_Handler();
  }

  HAL_Delay(100);

  value = 0x8CU;
  status = HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              I2C_ADDR_CFG_REG_A_M, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);

  if (status != HAL_OK)
  {
    Error_Handler();
  }

  value = 0x02;
  status = HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              I2C_ADDR_CFG_REG_B_M, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);

  if (status != HAL_OK)
  {
    Error_Handler();
  }

  value = 0x10;
  status = HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              I2C_ADDR_CFG_REG_C_M, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);

  if (status != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(20);


//   HAL_GPIO_WritePin(g_led_states[7].GPIOx, g_led_states[7].GPIO_Pin, GPIO_PIN_SET);
}


void data_ready_interrupt(TimerHandle_t xTimer)
{

  HAL_StatusTypeDef status;
  static uint8_t result[sizeof(g_reading_steps) / sizeof(*g_reading_steps)] = {0};
  int32_t result_32[3];
  uint8_t status_reg = 0;
  uint8_t axis_idx;
  double angle;

  static uint8_t curr_poll = 0;

  uint8_t led_idx;

  status = HAL_I2C_Mem_Read(&hi2c1, 0x3C,
                            I2C_ADDR_STATUS_REG_M, I2C_MEMADD_SIZE_8BIT,
                            &status_reg, 1, HAL_MAX_DELAY);

  if (!(status_reg & 0b1000))
  {
    return;
  }

  /* Read all registers */
  for (axis_idx = 0; axis_idx < sizeof(g_reading_steps)/sizeof(*g_reading_steps); axis_idx++)
  {
    status = HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              g_reading_steps[curr_poll], I2C_MEMADD_SIZE_8BIT,
                              result + axis_idx, 1, HAL_MAX_DELAY);

    if (status != HAL_OK)
    {
      break;
    }

    curr_poll++;
  }

  if (status != HAL_OK)
  {
    Error_Handler();
  }

  /* Check that this is an error and pack registers into values. */
  if (curr_poll == sizeof(g_reading_steps) / sizeof(*g_reading_steps))
  {
    curr_poll = 0;

    result_32[0] = CONVERSE_16BIT_TO_32BIT(CONCAT_8BIT_INTO_16BIT(result[1], result[0]));
    result_32[1] = CONVERSE_16BIT_TO_32BIT(CONCAT_8BIT_INTO_16BIT(result[3], result[2]));
    result_32[2] = CONVERSE_16BIT_TO_32BIT(CONCAT_8BIT_INTO_16BIT(result[5], result[4]));

    for(led_idx = 0; led_idx < LED_CNT; led_idx++)
    {
      g_led_states[led_idx].duty_cycle_percent = 0U;
    }

    /* Get result. */
    angle = atan2(result_32[0], result_32[1]) + M_PI;

    for(led_idx = 0; led_idx < LED_CNT; led_idx++)
    {
      if (angle >= M_PI_4 * led_idx && angle <= M_PI_4 * (led_idx + 1))
      {
        g_led_states[led_idx].duty_cycle_percent = 100;
      }
    }
  }

}

