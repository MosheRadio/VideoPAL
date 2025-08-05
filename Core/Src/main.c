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
#include <string.h>
#include "stdio.h"

void show(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//uint16_t fullFrame[TOTAL_LINES * VID_HSIZE];

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
void Video_AdjustI2SFor8MHz(void);
void Video_SetupTiming(void);

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
void Gpio_deinit(GPIO_TypeDef  *GPIOx){
		for(int i =0; i<15;i++){
			HAL_GPIO_DeInit(GPIOA, i);
		}
	}


void TIMER_SET(uint32_t sys){


	if (sys == 16000000){
		  __HAL_I2S_DISABLE(&hi2s2);
		   SPI2->I2SPR = 1;
		   __HAL_I2S_ENABLE(&hi2s2);
	}

	else if(sys == 48000000){
		  __HAL_I2S_DISABLE(&hi2s2);
		  SPI2->I2SPR = (3 << SPI_I2SPR_I2SDIV_Pos)
		              | (0 << SPI_I2SPR_ODD_Pos);
		   __HAL_I2S_ENABLE(&hi2s2);
	}

	else if (sys == 96000000){
		  __HAL_I2S_DISABLE(&hi2s2);
		  SPI2->I2SPR = (6 << SPI_I2SPR_I2SDIV_Pos)
		              | (0 << SPI_I2SPR_ODD_Pos);
		   __HAL_I2S_ENABLE(&hi2s2);
		   HAL_Delay(1);
	}

	else if(sys == 120000000){
		__HAL_I2S_DISABLE(&hi2s2);
		SPI2->I2SPR = (7 << SPI_I2SPR_I2SDIV_Pos) | (1 << SPI_I2SPR_ODD_Pos);
		__HAL_I2S_ENABLE(&hi2s2);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	HAL_I2S_DeInit(&hi2s2);
	HAL_TIM_OC_DeInit(&htim2);
	HAL_TIM_OC_DeInit(&htim3);
	Gpio_deinit(GPIOA);
	Gpio_deinit(GPIOB);
	Gpio_deinit(GPIOC);
	HAL_DMA_DeInit(&hdma_tim3_ch1);
	HAL_DMA_DeInit(&hdma_tim3_ch3);
	HAL_DMA_DeInit(&hdma_spi2_tx);
	HAL_DMA_DeInit(&hdma_tim3_ch4);




  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* --- Enable Flash prefetch and caches before changing SYSCLK --- */
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
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
  TIMER_SET(HAL_RCC_GetSysClockFreq());
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  //Video_AdjustI2SFor8MHz();
  //Video_SetupTiming();
  // 1) How many timer ticks per half-word at your clock?


  //do know if it is necessary
  HAL_TIM_Base_Start(&htim2); // start the timer for the video sync

  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_4);  // OC4Ref → TRGO


  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // this the same
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_4);



  HAL_DMA_Start(
    &hdma_tim3_ch1,
    (uint32_t)SyncTable,                // memory: array of CCR1 timings
    (uint32_t)&TIM3->CCR1,              // peripheral: CCR1 register
    325                           // one entry per visible line
  );
  __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_CC1);  // also enable CC1DE for VSync

  // 2) LINE BUFFERS → I2S DMA CMAR at back porch (CC3)
  HAL_DMA_Start(
    &hdma_tim3_ch3,
    (uint32_t)lineptrs,                 // memory: array of line-buffer addresses
    (uint32_t)&hdma_spi2_tx.Instance->CMAR,
	313//VID_VSIZE
  );

  // 3) BLACK-PORCH → I2S DMA CMAR at front porch (CC4)
  HAL_DMA_Start(
    &hdma_tim3_ch4,
    (uint32_t)borders,                  // memory: single-entry blank-line buffer
    (uint32_t)&hdma_spi2_tx.Instance->CMAR,
    313// one entry per line
  );

  // 4) Kick off the I2S DMA stream once
  HAL_I2S_Transmit_DMA(
    &hi2s2,
    (uint16_t*)Vblack,
    VID_HSIZE// 21 + 11 = 32 half-words per line
  );


  // also i added a function for handleing
  //srand(SysTick->VAL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  vidClearScreen();
//  gdiDrawTextEx(60, 10, "Hello World!");
//  gdiDrawTextEx(110, 60, "hey");
//  gdiDrawTextEx(120, 70, "MOSHE");
//  gdiDrawTextEx(130, 55, "REUT");
  //gdiDrawTextEx(140, 90, "IDAN");
//  gdiDrawTextEx(150, 80, "RUBEN");
//  gdiDrawTextEx(160, 100, "a");


  while (1)
  {
		for (int i=50, j=50; i < 160 && j < 150; i++ && j++) {
            gdiDrawSmallTextEx(i, j, "IDAN");
//			if (i == 160 && j == 150) {
//				i = 50;
//				j = 50;
//			}
        	HAL_Delay(50);
        	vidClearScreen();
		}
		vidClearScreen();

		for (int k=0; k < 100; k++) {
			gdiDrawTextEx((100+k), 50, "I");
			gdiDrawTextEx((105+k), 50, "D");
			gdiDrawTextEx((110+k), 50, "A");
			gdiDrawTextEx((115+k), 50, "N");
			HAL_Delay(50);
		}
		vidClearScreen();

		for (int i=50, j=50; i < 160 && j < 150; i++ && j++) {
            gdiDrawBigTextEx(i, j, "IDAN");
//			if (i == 160 && j == 150) {
//				i = 50;
//				j = 50;
//			}
        	HAL_Delay(50);
        	vidClearScreen();
		}
		vidClearScreen();




	  //introScreen("Hey whats up");

	  //HAL_Delay(1000);


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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV10;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // 8mhz *12 / 2

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;



  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
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

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */
	//hi2s2.Init.CPOL = I2S_CPOL_HIGH;

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s2.Init.CPOL = I2S_CPOL_HIGH;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */
//  __HAL_I2S_DISABLE(&hi2s2);
////  SPI2->I2SPR = (3 << SPI_I2SPR_I2SDIV_Pos)
////              | (0 << SPI_I2SPR_ODD_Pos);
//
//   SPI2->I2SPR = 1;   // I2SDIV = 1, ODD = 0
//   __HAL_I2S_ENABLE(&hi2s2);

  /* USER CODE END I2S2_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */



  //i should change :
  //htim2.Init.Prescaler         = VID_HSIZE/4 - 1; // 32/4 -1 = 7
  //htim2.Init.Period            = 2*VID_VSIZE - 1; // 2*625-1 = 1249
  //sConfigOC.Pulse      = VID_VSIZE - 1;

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = VID_HSIZE/4 - 1; // 32/4 -1 = 7
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2*VID_VSIZE - 1; // 2*625-1 = 1249
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE1;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;//NEEDS TO BE CHECK WITH EXTERNAL1
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF; // OC4Ref → TRGO;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse =  VID_VSIZE - 1;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = HSYNCCOUNTS;//HSYNCCOUNTS;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 2000;//208;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 8000;//(672+208);//(672+208);
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  //TIM_DMACmd(TIM3, TIM_DMA_CC1|TIM_DMA_CC3, ENABLE);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
  /* Allow TIM3 Compare-3 (CC3) and Compare-4 (CC4) events to generate DMA requests */
  //__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_CC2);
  __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_CC3);   // CC3DE bit → DMA request on CC3
  __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_CC4);   // CC4DE bit → DMA request on CC4
  //__HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_3);
  //__HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_4);


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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */


  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Video_AdjustI2SFor8MHz(void) {
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();      // e.g. 96000000
    uint32_t div    = sysclk / (2 * 8000000UL);       // floor(96e6/16e6) = 6
    uint32_t rem    = sysclk % (2 * 8000000UL);
    uint32_t odd    = (rem != 0);                     // only if you need sub‑pixel accuracy
    SPI2->I2SPR = (div << SPI_I2SPR_I2SDIV_Pos)
                | (odd << SPI_I2SPR_ODD_Pos);
}
//
//void Video_SetupTiming(void) {
//    uint32_t sys = HAL_RCC_GetSysClockFreq();     // e.g. 96000000
//    uint32_t hf = 1000000UL / PAL_Hsyncinterval;  // 15625
//    uint32_t cnt = sys / hf;                      // 6144 ticks/line
//    uint32_t sync = cnt * PAL_HsyncPulsewidth / PAL_Hsyncinterval; // ~451
//    uint32_t perHW = cnt / VID_HSIZE;             // ticks per half‑word
//
//    __HAL_TIM_SET_AUTORELOAD(&htim3, cnt - 1);
//    __HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_2, sync);
//    __HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_3,
//                           sync + HPORCH*perHW);
//    __HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_4,
//                           sync + (HPORCH+XFERS_PERLINE)*perHW);
//
//    uint32_t div = sys / (2 * 8000000UL);         // e.g. 6
//    SPI2->I2SPR = (div << SPI_I2SPR_I2SDIV_Pos)
//                | (0 << SPI_I2SPR_ODD_Pos);
//}


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
