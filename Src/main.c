/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
//#include <stm32f103xb.h>
#include <stm32f1xx_ll_tim.h>
#include <stm32f1xx_hal_tim.h>
#include "string.h"
#include "log.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim3_ch4_up;
DMA_HandleTypeDef hdma_tim4_ch1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t KKK[200];

uint16_t OOO[] = {
524, 0, 352,
45,0, 22,
44,0, 22,
45,0, 23,
45,0, 22,
45,0, 22,
44,0, 22,
45,0, 23,
86,0, 22,
87,0, 23,
86,0, 23,
86,0, 22,
86,0, 22,
86,0, 22,
87,0, 23,
86,0, 22,
44,0, 22,
45,0, 23,
86,0, 22,
45,0, 23,
45,0, 22,
45,0, 22,
44,0, 23,
45,0, 23,
45,0, 22,
86,0, 22,
45,0, 22,
86,0, 22,
86,0, 22,
86,0, 22,
87,0, 23,
86,0, 22,
86,0, 22,
140,0, 22,
436,0, 351,
3714,0, 22,
436,0, 350,
138,0, 23,
138, 0, 0
};
//uint16_t OOO[]={
//    3,  3, 1, 1,
//    4, 4, 2, 2,
//    5, 6, 3, 3,
//    2, 12, 1, 4,
//    10, 3, 0, 5
//};
//uint16_t OOO[]={
//        3,  0, 1,
//        4, 0, 2,
//        5, 0, 3,
//        2, 0, 1,
//        10, 0, 0
//};
//uint16_t OOO[]={
//  1,2,3,4,5,6,7,8,9,10,11,12
//};
uint16_t PPP[]={
        6,7,8,9
};

void startdma(){
  TIM_HandleTypeDef *htim = &htim3;
  /* configure the DMA Burst Mode */
  htim->Instance->DCR = TIM_DMABASE_CCR3 | TIM_DMABURSTLENGTH_2TRANSFERS;

  HAL_DMA_Start_IT(&hdma_tim3_ch4_up, (uint32_t)&htim->Instance->DMAR, (uint32_t)KKK, 200);

  htim->State = HAL_TIM_STATE_READY;

  /* Enable the TIM DMA Request */
  __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC4);

}
static void TIM_DMASendIRDMACplt(DMA_HandleTypeDef *hdma)
{
  TIM_HandleTypeDef* htim = ( TIM_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  htim->State= HAL_TIM_STATE_READY;
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
}

void SendIRData(uint16_t *pData, uint32_t cnt){
  if (cnt < 6){
    LogE("Invalid IRData len = %d", cnt);
    return;
  }
  LogE("Send IRData: len = %d", cnt);
  TIM_HandleTypeDef *htim = &htim4;

  HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
  HAL_DMA_Abort_IT(&hdma_tim4_ch1);
  __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
  __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);

  /* configure the DMA Burst Mode */
  htim->Instance->DCR = TIM_DMABASE_ARR | TIM_DMABURSTLENGTH_3TRANSFERS;

  //Write first pulse param
  __HAL_TIM_SET_AUTORELOAD(htim, pData[0]);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pData[2]);
  HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);

  hdma_tim4_ch1.XferCpltCallback = TIM_DMASendIRDMACplt;
  HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t)&pData[3], (uint32_t)&htim->Instance->DMAR, cnt-3);

  htim->State = HAL_TIM_STATE_READY;

  /* Enable the TIM Capture/Compare 1 DMA request */
  __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC1);

  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);

}
//void sendIRdma(){
//  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
//
//  HAL_DMA_Abort_IT(&hdma_tim4_ch1);
////  HAL_DMA_Abort_IT(&hdma_tim4_up);
//
//  TIM_HandleTypeDef *htim = &htim4;
//  /* Enable the TIM Capture/Compare 1 DMA request */
//  __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
//
//  /* Enable the TIM DMA Request */
//  __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);
//
//  /* configure the DMA Burst Mode */
//  htim->Instance->DCR = TIM_DMABASE_ARR | TIM_DMABURSTLENGTH_4TRANSFERS;
////  htim->Instance->DCR = TIM_DMABASE_CCR2 | TIM_DMABURSTLENGTH_2TRANSFERS;
//  __HAL_TIM_SET_AUTORELOAD(htim, 10);
//  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 3);
//  HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);
////  HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_CC1);
//
////  hdma_tim4_up.XferCpltCallback = TIM_DMASendIRDMACplt;
//  hdma_tim4_ch1.XferCpltCallback = TIM_DMASendIRDMACplt;
//
////  HAL_DMA_Start_IT(&hdma_tim4_up, (uint32_t)IROut, (uint32_t)&htim->Instance->DMAR, sizeof(IROut)/ sizeof(uint16_t));
//  int cnt =   sizeof(OOO)/ sizeof(uint16_t);
////  HAL_DMA_Start_IT(&hdma_tim4_up, (uint32_t)OOO, (uint32_t)&htim->Instance->DMAR, cnt);
//  HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t)OOO, (uint32_t)&htim->Instance->DMAR, cnt);
//
//  htim->State = HAL_TIM_STATE_READY;
//
//
////  cnt =   sizeof(PPP)/ sizeof(uint16_t);
////  HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t)PPP, (uint32_t)&htim->Instance->CCR4, cnt);
//    /* Enable the TIM Capture/Compare 1 DMA request */
//    __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC1);
//
//  /* Enable the TIM DMA Request */
////  __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_UPDATE);
//
////  HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);
////  HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_CC1);
//  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
//}

void logRecord(){
  int i = 0;
  for (; i<200; i++){
    if (KKK[i] == 0){
      break;
    }
  }
  int cnt = i;
  LogI("record cnt=%d, DMA left=%d", cnt, __HAL_DMA_GET_COUNTER(&hdma_tim3_ch4_up));
  uint16_t *p = (uint16_t *) KKK;
  for (int j = 1; j < cnt*2; ++j) {
     LogI("%d --[%08x]--%d,  %d,  %d", j, KKK[j/2],  p[j], p[j-1], (int)(13.2*2*(p[j]-p[j-1])));
  }

//  LogI(" ++++++++ ");
//    int j = 2;
//  for (; j < cnt*2; j+=2) {
//    int hi = p[j-1] - p[j-2];
//    int low = p[j] - p[j-1];
//    LogI("%d, %d,", hi+low, hi);
//  }
//  if (cnt > 0){
//    int hi = p[j-1] - p[j-2];
//    int low = hi*5;
//    LogI("%d, %d,", hi+low, hi);
//  }

  LogI(" ========== ");
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  HAL_TIM_Base_Start(&htim2);
//  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_4);
//  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_DMABurst_ReadStop(&htim3, TIM_DMA_CC4);
//    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_3, KKK, 200);
//  sendIRdma();
  SendIRData(OOO, sizeof(OOO)/ sizeof(uint16_t));
  while (1)
  {
//    LogI("adfasdfsadf");
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    memset(KKK, 0, sizeof(KKK));
//    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_3, KKK, 200);
    startdma();
    if (!LL_TIM_IsEnabledCounter(htim4.Instance)){
      HAL_Delay(1000);
      for (int i = 0; i <10 ; ++i) {
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
      }
      SendIRData(OOO, sizeof(OOO)/ sizeof(uint16_t));
    }
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
//    sendIRdma();
    HAL_Delay(5000);
//			if (HAL_GetTick() - tToggle > 1000){
//    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
//    HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_3);
    HAL_TIM_DMABurst_ReadStop(&htim3, TIM_DMA_CC4);
//    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
//    HAL_TIM_DMABurst_WriteStop(&htim4, TIM_DMA_UPDATE);
    logRecord();
    HAL_Delay(1000);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 157;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
