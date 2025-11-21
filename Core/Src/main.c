/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include"spi_slave.h"
#include "icx.h"
#include "changecaculate.h"
#include "pwm6.h"
#include "motor.h"
#include "chassis.h"
#include "robot_config.h"
#include "rc_process.h"
#include "gimbal.h"
uint8_t d0 = 0, d1 = 0, d2 = 0, d3 = 0; // 由 SPI1 更新
uint8_t d4 = 0, d5 = 0;  // 由 ICX_GetDuty(&ic3/4) 更新
uint8_t k0 = 0, k1, k2, k3;//SPI2收到的数据
ICX_Handle  ic3, ic4;
SPI_SlaveCtx spi1_ctx, spi2_ctx;
//int16_t AngleSpeedOfReel_Origianl0, AngleSpeedOfReel_Origianl1, AngleSpeedOfReel_Origianl2, AngleSpeedOfReel_Origianl3;
//int16_t RealReelSpeed0, RealReelSpeed1, RealReelSpeed2, RealReelSpeed3;
const uint32_t TICK_HZ = 1000000UL;

extern int wheel_target[4];
extern int wheel_speed[4];
extern RC_Ctrl_t rc;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  PWM6_Init();
  PWM6_StartAll();
  SPI_Slave_InitAndStart_IT(&spi1_ctx, &hspi1, SPI_Slave_FrameLen6);
  SPI_Slave_InitAndStart_IT(&spi2_ctx, &hspi2, SPI_Slave_FrameLen6);

  ICX_Attach(&ic3, &htim3, TIM_CHANNEL_1, TICK_HZ);
  ICX_Start(&ic3);
  ICX_Attach(&ic4, &htim4, TIM_CHANNEL_1, TICK_HZ);
  ICX_Start(&ic4);

  Chassis_PID_Init();

  Gimbal_Init();
Motor_SetPWM(5, 10);

   Motor_SetPWM(6, 9);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    

    // 更新SPI1遥控器4通道
    SPI_Slave_TryGet4U8(&spi1_ctx, &d0, &d1, &d2, &d3);
    
    // 更新开关量 d4, d5
    d4 = ICX_GetDuty(&ic3);
    d5 = ICX_GetDuty(&ic4);

    // 运行你的底盘控制逻辑
    Chassis_ControlLoop();
    // 运行云台控制逻辑
    // Gimbal_ControlLoop();

    //  OLED 调试显示  !!!OLED显示频繁会影响控制周期，若调试出现问题请注释掉!!!
    
    // OLED_ShowString(1,1,"Tar:",OLED_6X8);
    // OLED_ShowSignedNum(1, 9, (int)wheel_target[0], 20, OLED_6X8);
    // OLED_ShowSignedNum(1, 17, (int)wheel_target[1], 20, OLED_6X8);
    // OLED_ShowSignedNum(1, 25, (int)wheel_target[2], 20, OLED_6X8);
    // OLED_ShowSignedNum(1, 33, (int)wheel_target[3], 20, OLED_6X8);
    
    // OLED_ShowString(35,1,"Spe:",OLED_6X8);
    // OLED_ShowSignedNum(35, 9, wheel_speed[0], 4, OLED_6X8);
    // OLED_ShowSignedNum(35, 17, wheel_speed[1], 4, OLED_6X8);
    // OLED_ShowSignedNum(35, 25, wheel_speed[2], 4, OLED_6X8);
    // OLED_ShowSignedNum(35, 33, wheel_speed[3], 4, OLED_6X8);
    // OLED_Update();
    // OLED_ShowString(70,1,"vxvy:",OLED_6X8);
    // OLED_ShowSignedNum(70, 9, (int)rc.vx, 4, OLED_6X8);
    // OLED_ShowSignedNum(70, 17, (int)rc.vy, 4, OLED_6X8);
    // OLED_ShowSignedNum(70, 25, (int)rc.omega, 4, OLED_6X8);

    
    
    HAL_Delay(5); // 5ms 控制周期（200Hz）
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
#ifdef USE_FULL_ASSERT
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
