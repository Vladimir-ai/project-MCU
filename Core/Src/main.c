/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONCAT_8BIT_INTO_16BIT(high_byte, low_byte)  (((high_byte) << 8U) | low_byte)
#define MAX(a, b)                                   ((a) = (b) > (a) ? (a):(a))
#define MIN(a, b)                                   ((a) = (b) < (a) ? (b):(a))

#define MAGN_VALUE_TO_PERSENT(val) ((0xffff - (val)) / 10)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

uint8_t buf[20];
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

static uint8_t g_offsets[] =
{
  I2C_ADDR_OFFSET_X_REG_L_M,
  I2C_ADDR_OFFSET_X_REG_H_M,
  I2C_ADDR_OFFSET_Y_REG_L_M,
  I2C_ADDR_OFFSET_Y_REG_H_M,
  I2C_ADDR_OFFSET_Z_REG_L_M,
  I2C_ADDR_OFFSET_Z_REG_H_M
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// bool tx_completed = false;
// bool rx_complete = false;
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  buf[10] = 0;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint16_t current_ctr = 0;
  static uint8_t led_idx;
  static bool reset_state = false;

  if (htim == &htim16)
  {
    reset_state = true;
    current_ctr = 0;

    data_ready_interrupt();

    for (led_idx = 0; reset_state && led_idx < LED_CNT; led_idx++)
    {
      if (g_led_states[led_idx].duty_cycle_percent != 0U)
      {
        HAL_GPIO_WritePin(g_led_states[led_idx].GPIOx, g_led_states[led_idx].GPIO_Pin, GPIO_PIN_SET);
      }
    }

    for (led_idx = 0; led_idx < LED_CNT; led_idx++)
    {
      if (g_led_states[led_idx].duty_cycle_percent == 0)
      {
        HAL_GPIO_WritePin(g_led_states[led_idx].GPIOx, g_led_states[led_idx].GPIO_Pin, GPIO_PIN_RESET);
      }
    }
  }
}

void mb_init(void)
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
                              0x60U, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);

  if (status != HAL_OK)
  {
    Error_Handler();
  }

  value = 0x02;
  status = HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              0x61U, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);

  if (status != HAL_OK)
  {
    Error_Handler();
  }

  value = 0x10;
  status = HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              0x62U, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);

  if (status != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(20);


  HAL_GPIO_WritePin(g_led_states[7].GPIOx, g_led_states[7].GPIO_Pin, GPIO_PIN_SET);


  // mb_toggle_pin(LD4_GPIO_Port, LD4_Pin, value);

  // /* IRA_REG_M */
  // HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_READ_ACCELEROMETER, I2C_ADDR_OUT_X_H_A, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);

  // mb_toggle_pin(LD3_GPIO_Port, LD3_Pin, value);

  // /* IRA_REG_M */
  // HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_READ_ACCELEROMETER, I2C_ADDR_OUT_Y_L_A, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);

  // mb_toggle_pin(LD5_GPIO_Port, LD5_Pin, value);

  // /* IRA_REG_M */
  // HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_READ_ACCELEROMETER, I2C_ADDR_OUT_Y_H_A, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);

  // mb_toggle_pin(LD6_GPIO_Port, LD6_Pin, value);

  // /* IRA_REG_M */
  // HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_READ_ACCELEROMETER, I2C_ADDR_OUT_Z_L_A, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);

  // mb_toggle_pin(LD7_GPIO_Port, LD7_Pin, value);

  // /* IRA_REG_M */
  // HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_READ_ACCELEROMETER, I2C_ADDR_OUT_Z_H_A, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);

  // mb_toggle_pin(LD8_GPIO_Port, LD8_Pin, value);
}


#ifdef SELF_TEST_MODE

static uint8_t axis_values[6];

void read_all_regs(void)
{
  uint8_t axis_idx;
  HAL_StatusTypeDef status;
  uint8_t value;

  for (axis_idx = 0; axis_idx < sizeof(g_reading_steps)/sizeof(*g_reading_steps); axis_idx++)
  {
    status = HAL_I2C_Mem_Read(&hi2c1, 0x3C,
                              g_reading_steps[axis_idx], I2C_MEMADD_SIZE_8BIT,
                              &axis_values[axis_idx], 1, HAL_MAX_DELAY);
  }
}


void self_test(void)
{
  uint8_t value;
  HAL_StatusTypeDef status;
  uint8_t idx;
  uint8_t axis_idx;
  uint16_t x_axis[50], y_axis[50], z_axis[50];
  uint16_t st_x_axis[50], st_y_axis[50], st_z_axis[50];
  uint32_t no_st[3] = {0, 0, 0}, st[3] = {0, 0, 0};
  uint16_t st_min[3] = {0xFFFF, 0xFFFF, 0xFFFF}, st_max[3] = {0, 0, 0};

  memset(axis_values, 0, sizeof(axis_values));

  value = 0x8CU;
  status = HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              0x60U, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);

  if (status != HAL_OK)
  {
    Error_Handler();
  }

  value = 0x02;
  status = HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              0x61U, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);

  if (status != HAL_OK)
  {
    Error_Handler();
  }

  value = 0x10;
  status = HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              0x62U, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);

  if (status != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(20);

  // Second block

  while(1)
  {
    status = HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              0x67U, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);
    if (value & 0b1000)
    {
      read_all_regs();
      break;
    }
  }

  for (idx = 0; idx < 50; idx++)
  {
    status = HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                            0x67U, I2C_MEMADD_SIZE_8BIT,
                            &value, 1, HAL_MAX_DELAY);

    if (value & 0b1000)
    {
      read_all_regs();

      x_axis[axis_idx] = CONCAT_8BIT_INTO_16BIT(axis_values[1], axis_values[0]);
      y_axis[axis_idx] = CONCAT_8BIT_INTO_16BIT(axis_values[3], axis_values[2]);
      z_axis[axis_idx] = CONCAT_8BIT_INTO_16BIT(axis_values[5], axis_values[4]);

      axis_idx++;
    }

    // HAL_Delay(5);
  }

  value = 0x12;
  status = HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              0x62U, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);

  HAL_Delay(60);

  while(1)
  {
    status = HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              0x67U, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);
    if (value & 0b1000)
    {
      read_all_regs();
      break;
    }
  }

  for (idx = 0; idx < 50; idx++)
  {
    status = HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                            0x67U, I2C_MEMADD_SIZE_8BIT,
                            &value, 1, HAL_MAX_DELAY);

    if (value & 0b1000)
    {
      read_all_regs();

      st_x_axis[axis_idx] = CONCAT_8BIT_INTO_16BIT(axis_values[1], axis_values[0]);
      st_y_axis[axis_idx] = CONCAT_8BIT_INTO_16BIT(axis_values[3], axis_values[2]);
      st_z_axis[axis_idx] = CONCAT_8BIT_INTO_16BIT(axis_values[5], axis_values[4]);

      axis_idx++;
    }

    // HAL_Delay(5);
  }

  for (idx = 0; idx < 50; idx++)
  {
    st_max[0] = fmaxf(st_max[0], st_x_axis[idx]);
    st_min[0] = fminf(st_min[0], st_x_axis[idx]);

    st_max[1] = fmaxf(st_max[1], st_y_axis[idx]);
    st_min[1] = fminf(st_min[1], st_y_axis[idx]);

    st_max[2] = fmaxf(st_max[2], st_z_axis[idx]);
    st_min[2] = fminf(st_min[2], st_z_axis[idx]);

    st[0] += st_x_axis[idx];
    st[1] += st_y_axis[idx];
    st[2] += st_z_axis[idx];

    no_st[0] += x_axis[idx];
    no_st[1] += y_axis[idx];
    no_st[2] += z_axis[idx];
  }



  status = HAL_OK;
  for (axis_idx = 0; axis_idx < 3; axis_idx++)
  {
    st[axis_idx] /= 50;
    no_st[axis_idx] /= 50;

    if (st_min[axis_idx] <= abs(st[axis_idx] - no_st[axis_idx])
        && abs(st[axis_idx] - no_st[axis_idx]) <= st_max[axis_idx])
    {
      HAL_GPIO_WritePin(g_led_states[2].GPIOx, g_led_states[2].GPIO_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(g_led_states[3].GPIOx, g_led_states[3].GPIO_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(g_led_states[4].GPIOx, g_led_states[4].GPIO_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(g_led_states[5].GPIOx, g_led_states[5].GPIO_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(g_led_states[6].GPIOx, g_led_states[6].GPIO_Pin, GPIO_PIN_SET);
    }
  }

  value = 0x10;
  status = HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              0x62U, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);

  value = 0x83;
  status = HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_READ_MAGNETOMETER,
                              0x62U, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, HAL_MAX_DELAY);
}

#endif /* SELF_TEST_MODE */

void data_ready_interrupt(void)
{
  HAL_StatusTypeDef status;
  static uint8_t result[sizeof(g_reading_steps) / sizeof(*g_reading_steps)] = {0};
  static uint16_t min[3] = {0xffff, 0xffff, 0xffff};
  static uint16_t max[3] = {0};
  int32_t result_32[3];
  uint8_t status_reg = 0;
  static uint16_t res_16_bit[3]; // x,y,z
  uint8_t axis_idx;
  // uint16_t x_percent,y_percent;
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

  for (axis_idx = 0; axis_idx < sizeof(g_reading_steps)/sizeof(*g_reading_steps); axis_idx++)
  {
    status = HAL_I2C_Mem_Read(&hi2c1, 0x3C,
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

  // g_led_states[1].duty_cycle_percent = 100;
  // g_led_states[2].duty_cycle_percent = 100;
  // g_led_states[3].duty_cycle_percent = 100;


  if (curr_poll == sizeof(g_reading_steps) / sizeof(*g_reading_steps))
  {
    curr_poll = 0;

    res_16_bit[0] = CONCAT_8BIT_INTO_16BIT(result[1], result[0]);
    res_16_bit[1] = CONCAT_8BIT_INTO_16BIT(result[3], result[2]);
    res_16_bit[2] = CONCAT_8BIT_INTO_16BIT(result[5], result[4]);

    for (axis_idx = 0; axis_idx < 3; axis_idx++)
    {
      min[axis_idx] = fminf(res_16_bit[axis_idx], min[axis_idx]);
      max[axis_idx] = fmaxf(res_16_bit[axis_idx], max[axis_idx]);

      result_32[axis_idx] = res_16_bit[axis_idx] - (0xffff) / 2;
    }

    // x_percent = MAGN_VALUE_TO_PERSENT(res_16_bit[0]);
    // y_percent = MAGN_VALUE_TO_PERSENT(res_16_bit[1]);

    for(led_idx = 0; led_idx < LED_CNT; led_idx++)
    {
      g_led_states[led_idx].duty_cycle_percent = 0U;
    }

    angle = (int) (atan2(result_32[0], result_32[1]) * (180 / M_PI) + 180);

    static float mn = 1000, mx = 0;

    mn = fminf(mn, angle);
    mx = fmaxf(mx, angle);

    if (angle >= 315 || (angle >= 0 && angle < 45))
    {
      g_led_states[0].duty_cycle_percent = 100;
    }
// 0 2 4 6 7 5 3 1
    if (angle >= 0 && angle < 90)
    {
      g_led_states[2].duty_cycle_percent = 100;
    }

    if (angle >= 45 && angle < 135)
    {
      g_led_states[4].duty_cycle_percent = 100;
    }

    if (angle >= 90 && angle < 180)
    {
      g_led_states[6].duty_cycle_percent = 100;
    }

    if (angle >= 135 && angle < 225)
    {
      g_led_states[7].duty_cycle_percent = 100;
    }

    if (angle >= 180 && angle < 270)
    {
      g_led_states[5].duty_cycle_percent = 100;
    }

    if (angle >= 225 && angle < 315)
    {
      g_led_states[3].duty_cycle_percent = 100;
    }

    if (angle >= 270 && angle < 360)
    {
      g_led_states[1].duty_cycle_percent = 100;
    }
  }

};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // uint8_t led_idx = 0;
  // bool reset_state = true;
  // uint8_t ret;

  if (GPIO_Pin == DRDY_Pin)
  {
    // data_ready_interrupt();

    // for (led_idx = 0; reset_state && led_idx < LED_CNT; led_idx++)
    // {
    //   if (g_led_states[led_idx].duty_cycle_percent != 0U)
    //   {
    //     HAL_GPIO_WritePin(g_led_states[led_idx].GPIOx, g_led_states[led_idx].GPIO_Pin, GPIO_PIN_SET);
    //   }
    // }

    // for (led_idx = 0; led_idx < LED_CNT; led_idx++)
    // {
    //   if (g_led_states[led_idx].duty_cycle_percent == 0)
    //   {
    //     HAL_GPIO_WritePin(g_led_states[led_idx].GPIOx, g_led_states[led_idx].GPIO_Pin, GPIO_PIN_RESET);
    //   }
    // }

    // HAL_I2C_Mem_Read(&hi2c1, 0x3C,
    //                 0x64, I2C_MEMADD_SIZE_8BIT,
    //                 ret, 1, HAL_MAX_DELAY);
  }

  // HAL_NVIC_ClearPendingIRQ(EXTI2_TSC_IRQn);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  mb_init();

  /* Enable interrupt for DRDY */
  // HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 0x00, 0x00);
  // HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

  HAL_TIM_Base_Start_IT(&htim16);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // mb_init();

    // HAL_Delay(100);
    // if (HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin) == GPIO_PIN_SET)
    // {
    //   HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    //   HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
    // }
    // else
    // {
    //   HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
    //   HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    // }

      // data_ready_interrupt();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1000;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DRDY_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
