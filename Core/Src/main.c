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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g4xx_hal.h"
#include "video.h"
#include <stdlib.h>
#include "stdio.h"

void show(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define DelayMs(ms)  HAL_Delay(ms)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1;
DMA_HandleTypeDef hdma_tim3_ch3;
DMA_HandleTypeDef hdma_tim3_ch4;

/* USER CODE BEGIN PV */
__IO uint32_t Paused;
__IO uint32_t TimingDelay;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2S2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DelayMs(uint32_t nTime) // delay function
{
  TimingDelay = nTime;
  while((TimingDelay != 0));
  while(Paused);
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_I2S2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //do know if it is necessary


  HAL_TIM_Base_Start(&htim2); // start the timer for the video sync
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2); // this the same
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // i am not sure if i need to start this PWM

//  // Now hook TIM3 → DMA on CH1 and CH3:
//  HAL_TIM_OC_Start_DMA(&htim3,
//                       TIM_CHANNEL_1,
//                       (uint32_t*)lineptrs,       // buffer of pointers per line
//                       VID_VSIZE);                // total number of lines per field
//
//  HAL_TIM_OC_Start_DMA(&htim3,
//                       TIM_CHANNEL_3,
//                       (uint32_t*)SyncTable,       // sync-pulse timing table
//                       VID_VSIZE);


  HAL_I2S_Transmit_DMA(&hi2s2, Vwhite, VID_HSIZE);


//#define PATTERN_LEN 64
//uint16_t testBuffer[PATTERN_LEN];
//
//void preparePattern(void) {
//    // Fill a sawtooth or other visible pattern
//    for (int i = 0; i < PATTERN_LEN; i++) {
//        testBuffer[i] = (uint16_t)(i * 0x1000) | (i & 0x0FFF);
//    }
//}

//void startDmaLoop(void) {
//    // 1) First fill the buffer
//    preparePattern();
//
//    // 2) Start the I2S‐DMA in circular mode so it never stops:
//    //    – hi2s2 was set up in MX_I2S2_Init()
//    //    – hdma_spi2_tx was configured in MX_DMA_Init()
//    //
//    if (HAL_I2S_Transmit_DMA(&hi2s2, testBuffer, PATTERN_LEN) != HAL_OK) {
//        Error_Handler();
//    }
//
//    // 3) Optionally adjust the DMA mode to circular, if CubeMX left it as normal:
//    __HAL_DMA_DISABLE(&hdma_spi2_tx);
//    hdma_spi2_tx.Instance->CCR |= DMA_CCR_CIRC;  // set circular bit
//    __HAL_DMA_ENABLE(&hdma_spi2_tx);
//
//    // Now the buffer will be replayed over and over at the I2S bitrate.
//}
  vidClearScreen();
  gdiDrawTextEx(40, 40, "HELLO VIDEO");

  // also i added a function for handleing
  srand(SysTick->VAL);
  //preparePattern();
  //startDmaLoop();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //show();
	  //DelayMs(20);
	  //HAL_I2S_Transmit_DMA(&hi2s2, testBuffer, PATTERN_LEN);
	 //HAL_I2S_Transmit_DMA(&hi2s2, Vwhite, XFERS_PERLINE+HPORCH);
	  //show();
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
    RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit    = {0};

    /* Voltage scaling for max performance */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* 1) Enable HSE and configure PLL (HSE=8 MHz → SYSCLK=16 MHz) */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM            = 1;                // /1
    RCC_OscInitStruct.PLL.PLLN            = 2;                // ×2 → 16 MHz
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV7;    // unused
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;    // USB if needed
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;    // SYSCLK = VCO/2 = 16 MHz
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* 2) Select PLL as SYSCLK source, AHB/APB @ no prescale */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_HCLK
                                     | RCC_CLOCKTYPE_PCLK1
                                     | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }

    /* 3) Tell the I2S peripheral to take its clock from SYSCLK */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    PeriphClkInit.I2sClockSelection    = RCC_I2SCLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}


/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{
  /* … HAL setup as before … */
  hi2s2.Instance          = SPI2;
  hi2s2.Init.Mode         = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard     = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat   = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput   = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq    = I2S_AUDIOFREQ_8K;     // dummy, we’ll override directly
  hi2s2.Init.CPOL         = I2S_CPOL_HIGH;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
    Error_Handler();

  /* —————————————— OVERRIDE PRESCALER —————————————— */
  __HAL_I2S_DISABLE(&hi2s2);
  // On STM32G4, writing 1 to I2SPR gives I2SDIV=1, ODD=0, MCKOE=0
  SPI2->I2SPR = 1;
  __HAL_I2S_ENABLE(&hi2s2);
}


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */

static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  //TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */



  //i should change :
  //htim2.Init.Prescaler         = VID_HSIZE/4 - 1;
  //htim2.Init.Period            = 2*VID_VSIZE - 1;
  //sConfigOC.Pulse      = VID_VSIZE - 1;

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = VID_HSIZE/4 - 1; // equal to
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2*VID_VSIZE - 1; // 2*
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // was DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  // chat told me to add - this is external then the ioc aoutumatics
  sClockSourceConfig.ClockSource    = TIM_CLOCKSOURCE_ETRMODE1;  // CHANGE it from INTERNAL to ETRMODE1
  sClockSourceConfig.ClockPolarity  = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter    = 0;


  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
//  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
//  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
//  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
//  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
//  sSlaveConfig.TriggerFilter = 0;
//  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = VID_VSIZE - 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */


  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  //need to change
  //htim3.Init.Period = TIMERCOUNTS - 1;  // e.g. 1023 for a 1024-count line

  //FOR CHANNEL 1 :
  //sConfigOC.Pulse = NO_TOG;

  //FOR CHANNEL 2:
  //sConfigOC.Pulse = HSYNCCOUNTS;

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIMERCOUNTS - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = NO_TOG;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  // HSYNC on CH2:
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = HSYNCCOUNTS;
  //sConfigOC.OC2Preaload = TIM_AUTORELOAD_PRELOAD_ENABLE; // was ENABLE
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  // Back-porch start on CH3:
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 208;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  // Active-video off on CH4:
  sConfigOC.Pulse = 880;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  //TIM_DMACmd(TIM3, TIM_DMA_CC1|TIM_DMA_CC3, ENABLE);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);


}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
