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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "log.h"
#include "defines.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004;
volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000;
volatile unsigned int *DWT_LAR      = (volatile unsigned int *)0xE0001FB0;
volatile unsigned int *SCB_DHCSR    = (volatile unsigned int *)0xE000EDF0;
volatile unsigned int *SCB_DEMCR    = (volatile unsigned int *)0xE000EDFC;
volatile unsigned int *ITM_TER      = (volatile unsigned int *)0xE0000E00;
volatile unsigned int *ITM_TCR      = (volatile unsigned int *)0xE0000E80;
static int Debug_ITMDebug = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

static void SmartPortPhaseIRQ();
static void SmartPortDeviceEnableIRQ();
static void SmartPortWpAckIRQ();
static void SmartPortWrReqIRQ();

static void SmartPortReceiveWRDataIRQ();
static void SmartPortReceiveRDDataIRQ();

static volatile uint8_t rdData=0;
static volatile uint8_t prevRdData=0;
static volatile uint8_t xorRdData=0;
static volatile int rdStartOffset=0;

static volatile uint8_t wrData=0;
static volatile uint8_t prevWrData=0;
static volatile uint8_t xorWrData=0;
static volatile int wrStartOffset=0;

static volatile unsigned int rdBitCounter=0;
static volatile unsigned int rdBytes=0;
static volatile unsigned int rdBytesReceived=0;

static volatile unsigned int wrBitCounter=0;
static volatile unsigned int wrBytes=0;
static volatile unsigned int wrBytesReceived=0;

static volatile unsigned char rdByteWindow=0x0;
static volatile unsigned char wrByteWindow=0x0;

static volatile uint16_t wrCycleWithNoBytes=0;
static volatile uint8_t flgPacket=2;

static volatile unsigned int WR_REQ_PHASE=0;

static volatile unsigned char phase=0x0;

unsigned char packet_bufferWR[SP_PKT_SIZE];   //smartport packet buffer
unsigned char packet_bufferRD[SP_PKT_SIZE];   //smartport packet buffer

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void EnableTiming(void){
  if ((*SCB_DHCSR & 1) && (*ITM_TER & 1)) // Enabled?
    Debug_ITMDebug = 1;

  *SCB_DEMCR |= 0x01000000;
  *DWT_LAR = 0xC5ACCE55;                                    // enable access
  *DWT_CYCCNT = 0;                                          // reset the counter
  *DWT_CONTROL |= 1 ;                                       // enable the counter
}



/**
  * @brief TIMER 1 IRQ Interrupt is handling the Reading from DEVICE to HOST
  * @param None
  * @retval None
  */

void TIM1_CC_IRQHandler(void){

  //  Data bit are sent every 4us
  //  For pulse a duration of 2us
  
  if (TIM1->SR & TIM_SR_UIF){
    TIM1->SR &= ~TIM_SR_UIF;                              // Clear the overflow interrupt 
    SmartPortReceiveRDDataIRQ();
  }else if (TIM1->SR & TIM_SR_CC1IF){                     // Pulse compare interrrupt on Channel 1
    TIM1->SR &= ~TIM_SR_CC1IF;                            // Clear the compare interrupt flag
    
  }else
    TIM1->SR = 0;

}
/**
  * @brief 
  * @param None
  * @retval None
  */
 void TIM1_UP_TIM10_IRQHandler(void){

}

/**
  * @brief TIMER 2 IRQ Interrupt is handling the Writing from HOST to DEVICE      
  * @param (void)
  * @retval None
  */
void TIM2_IRQHandler(void){
  
  if (TIM2->SR & TIM_SR_UIF){ 
    TIM2->SR &= ~TIM_SR_UIF;                                                  // Reset the Interrupt
  }else if (TIM2->SR & TIM_SR_CC2IF){                                         // The count & compare is on channel 2 to avoid issue with ETR1 
    TIM2->SR &= ~TIM_SR_CC2IF;                                                // clear the count & compare interrupt
    SmartPortReceiveWRDataIRQ();
  }else{
    TIM2->SR=0;
  }  
}

void EXTI0_IRQHandler(void){
  SmartPortPhaseIRQ();
}

void EXTI1_IRQHandler(void){
  SmartPortPhaseIRQ();
}

void EXTI2_IRQHandler(void){
  SmartPortPhaseIRQ();
}

void EXTI3_IRQHandler(void){
  SmartPortPhaseIRQ();
}

void EXTI15_10_IRQHandler(void){
  SmartPortWpAckIRQ();
}

void EXTI9_5_IRQHandler(void){
  SmartPortDeviceEnableIRQ();
  SmartPortWrReqIRQ();
}


static void SmartPortPhaseIRQ(){
    
  phase=(GPIOA->IDR&0b0000000000001111);
  return;
}

volatile uint8_t DEVICE_ENABLE=0x0;
static void SmartPortDeviceEnableIRQ(){
  DEVICE_ENABLE=(GPIOA->IDR&0b0000000000001111);
}

volatile uint8_t pWR_REQ_PHASE=0x1;
static void SmartPortWrReqIRQ(){

  /*if ((WR_REQ_GPIO_Port->IDR & WR_REQ_Pin)==0)
       WR_REQ_PHASE=0;
  else
       WR_REQ_PHASE=1;
  */
  WR_REQ_PHASE=WR_REQ_GPIO_Port->IDR & WR_REQ_Pin;

  if (WR_REQ_PHASE==0 && pWR_REQ_PHASE!=WR_REQ_PHASE){

    flgPacket=0;
    wrData=0;
    prevWrData=0;
    wrBytes=0;
    wrBitCounter=0;
    wrStartOffset=0;

    flgPacket=0;
    rdData=0;
    rdData=0;
    rdBytes=0;
    rdBitCounter=0;
    rdStartOffset=0;

    rdByteWindow=0x0;
    wrByteWindow=0x0;
    switch (phase){
      case 0x05:
        log_info("Ph:0x05 Reset message");                                                                         
        while (phase == 0x05);                                                      // Wait for phases to change 
        break;                                                                                      
      case 0x0a:                                                                            // Phase lines for smartport bus enable
      case 0x0b:                                                                            // Ph3=1 ph2=x ph1=1 ph0=x
      case 0x0e:
      case 0x0f:
        HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);
        HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_2);
        log_info("hereA");
      default:
        break;
    }
  }else if (WR_REQ_PHASE==1 && pWR_REQ_PHASE!=WR_REQ_PHASE){
    switch (phase){
      case 0x0a:
      case 0x0b:
      case 0x0e:
      case 0x0f:
        HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_1);
        packet_bufferRD[rdBytes++]=0x0;
        
        HAL_TIM_OC_Stop_IT(&htim2,TIM_CHANNEL_2);
        packet_bufferWR[wrBytes++]=0x0;
        log_info("hereB");
        flgPacket=1;
        break;
      default:
       break;
    }
  }
  pWR_REQ_PHASE=WR_REQ_PHASE;     
}


uint8_t WP_ACK=0;
static void SmartPortWpAckIRQ(){
  WP_ACK=WR_PROTECT_GPIO_Port->IDR & WR_PROTECT_Pin;
 /* if ((WR_PROTECT_GPIO_Port->IDR & WR_PROTECT_Pin)==0)
    WP_ACK=0;
  else
    WP_ACK=1;
    */
}

static void SmartPortReceiveWRDataIRQ(){
  // ADD WR_REQ IRQ TO MANAGE START & STOP OF THE TIMER
  
  /*if ((GPIOA->IDR & WR_DATA_Pin)==0)                                       // get WR_DATA DO NOT USE THE HAL function creating an overhead
      wrData=0;
  else
      wrData=1;
  */  
  
  wrData=WR_DATA_GPIO_Port->IDR & WR_DATA_Pin;
  
  wrData^= 0x01u;                                                           // get /WR_DATA
  xorWrData=wrData ^ prevWrData;                                            // Compute Magnetic polarity inversion
  prevWrData=wrData; 

  wrByteWindow<<=1;
  wrByteWindow|=xorWrData;
  
  if (wrByteWindow & 0x80){                                                 // Check if ByteWindow Bit 7 is 1 meaning we have a full bytes 0b1xxxxxxx 0x80
      
      packet_bufferWR[wrBytes]=wrByteWindow;
      
      if (wrStartOffset==0 && wrByteWindow==0xC3 && wrBitCounter>10)        // Identify when the message start
          wrStartOffset=0x01;
      
      wrByteWindow=0x0;

      if (wrStartOffset!=0)                                               // Start writing to packet_buffer only if offset is not 0 (after sync byte)
          wrBytes++;
  }

  wrBitCounter++;                                                           // Next bit please ;)
}

static void SmartPortReceiveRDDataIRQ(){
  /*if ((GPIOA->IDR & RD_DATA_Pin)==0)                                       // get WR_DATA DO NOT USE THE HAL function creating an overhead
    rdData=0;
  else
    rdData=1;
  */
  rdData=RD_DATA_GPIO_Port->IDR & RD_DATA_Pin;
  
  rdByteWindow<<=1;
  rdByteWindow|=rdData;

  if (rdByteWindow & 0x80){                                                 // Check if ByteWindow Bit 7 is 1 meaning we have a full bytes 0b1xxxxxxx 0x80
  
    packet_bufferRD[rdBytes]=rdByteWindow;
    
    if (rdStartOffset==0 && rdByteWindow==0xC3 && rdBitCounter>10)          // Identify when the message start
      rdStartOffset=0x01;
    
    rdByteWindow=0x0;

    if (rdStartOffset!=0)                                                     // Start writing to packet_buffer only if offset is not 0 (after sync byte)
      rdBytes++;
  }

  rdBitCounter++;                                                           // Next bit please ;)
}

void print_packet (unsigned char* data, int bytes){
  int count, row;
  char xx;

  //log_info("Dump packet src:%02X,dst:%02X,type:%02X,aux:%02x,cmd:%02X,paramcnt:%02X",data[SP_SRC],data[SP_DEST],data[SP_TYPE],data[SP_AUX],data[SP_COMMAND],data[SP_PARMCNT]);
  printf("\r\n");
  for (count = 0; count < bytes; count = count + 16) {
      
      printf("%04X: ", count);
      for (row = 0; row < 16; row++) {
          if (count + row >= bytes)
              printf("   ");
          else {
              printf("%02X ",data[count + row]);
          }
      }
      printf("- ");
      for (row = 0; row < 16; row++) {
          if ((data[count + row] > 31) && (count + row < bytes) && (data[count + row] < 129)){
              xx = data[count + row];
              printf("%c",xx);
          }
          else
              printf(".");
      }
      printf("\r\n");
  }
  
}

int packet_length (unsigned char * packet_buffer){
  int x = 0;

  while (packet_buffer[x++]);

  return x - 1; // point to last packet byte = C8
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  log_info("this is the sound of sea");
  int T2_DIER=0x0;
  T2_DIER|=TIM_DIER_CC2IE;
  T2_DIER|=TIM_DIER_UIE;
  TIM2->DIER|=T2_DIER;
  
  int T1_DIER=0x0;
  T1_DIER|=TIM_DIER_CC1IE;
  T1_DIER|=TIM_DIER_UIE;
  TIM1->DIER|=T1_DIER;

  //HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);   
  //HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_2);   
  uint8_t pflgPacket=0x0;
  while (1)
  {
    if (flgPacket==1 && pflgPacket!=flgPacket){
      printf("READ MSG:\n");
      print_packet(packet_bufferRD,packet_length(packet_bufferRD));

      printf("WRITE MSG:\n");
      print_packet(packet_bufferWR,packet_length(packet_bufferWR));
    }
    pflgPacket=flgPacket;

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 32*12;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 6*12;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 32*12-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 32*6;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PHASE0_Pin */
  GPIO_InitStruct.Pin = PHASE0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PHASE0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PHASE1_Pin PHASE2_Pin PHASE3_Pin PA4
                           RD_DATA_Pin WR_DATA_Pin */
  GPIO_InitStruct.Pin = PHASE1_Pin|PHASE2_Pin|PHASE3_Pin|GPIO_PIN_4
                          |RD_DATA_Pin|WR_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : WR_PROTECT_Pin WR_REQ_Pin */
  GPIO_InitStruct.Pin = WR_PROTECT_Pin|WR_REQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DEVICE_ENABLE_Pin */
  GPIO_InitStruct.Pin = DEVICE_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DEVICE_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
}


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);  
  return ch;
}
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
  log_error("Oups I lost my mind, and then I crashed with no inspiration\n");
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
