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
/*
static void SmartPortPhaseIRQ();
static void SmartPortDeviceEnableIRQ();
static void SmartPortWpAckIRQ();
static void SmartPortWrReqIRQ();
*/

static void SmartPortReceiveWRDataIRQ();
static void SmartPortReceiveRDDataIRQ();
static uint8_t verifyCmdpktChecksum(char * packet_buffer);


static void print_packet (unsigned char Emit,unsigned char* data, int bytes);
int packet_length (unsigned char * packet_buffer);

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


static void WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState){
 
  if(PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;
  }
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
    
  }else if (TIM1->SR & TIM_SR_CC1IF){                     // Pulse compare interrrupt on Channel 1
    TIM1->SR &= ~TIM_SR_CC1IF;                            // Clear the compare interrupt flag
    SmartPortReceiveRDDataIRQ();
  }else
    TIM1->SR = 0;
}

/**
  * @brief 
  * @param None
  * @retval None
  */
 void TIM1_UP_TIM10_IRQHandler(void){
 // HAL_GPIO_WritePin(DEBUG1_GPIO_Port,DEBUG1_Pin,GPIO_PIN_SET);
 // SmartPortReceiveRDDataIRQ();
 // HAL_GPIO_WritePin(DEBUG1_GPIO_Port,DEBUG1_Pin,GPIO_PIN_RESET);
 if (TIM1->SR & TIM_SR_UIF){
  TIM1->SR &= ~TIM_SR_UIF;
  return;
  
  
  //printf("%d",rdData);
//rdData=RD_DATA_GPIO_Port->IDR & RD_DATA_Pin;
return;
 }else{
  //SmartPortReceiveRDDataIRQ(); 
  TIM1->SR = 0;
 }
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

void SmartPortPhaseIRQ(){
    
  phase=(GPIOA->IDR&0b0000000000001111);
  return;
}

volatile uint8_t DEVICE_ENABLE=0x0;

void SmartPortDeviceEnableIRQ(){
  if ((DEVICE_ENABLE_GPIO_Port->IDR & DEVICE_ENABLE_Pin)==0)
    DEVICE_ENABLE=0;
  else
    DEVICE_ENABLE=1;
}

uint8_t WP_ACK=0;
uint8_t pWP_ACK=0;

volatile uint8_t pWR_REQ_PHASE=0x1;

void SmartPortWrReqIRQ(){

  if ((WR_REQ_GPIO_Port->IDR & WR_REQ_Pin)==0)
       WR_REQ_PHASE=0;
  else
       WR_REQ_PHASE=1;

  pWR_REQ_PHASE=WR_REQ_PHASE;     
}

void SmartPortWpAckIRQ(){

  if ((WR_PROTECT_GPIO_Port->IDR & WR_PROTECT_Pin)==0)
    WP_ACK=0;
  else
    WP_ACK=1;
  return;
}


static void SmartPortReceiveRDDataIRQ(){
  // ADD WR_REQ IRQ TO MANAGE START & STOP OF THE TIMER
  //WritePin(DEBUG1_GPIO_Port,DEBUG1_Pin,GPIO_PIN_SET);
  //  WritePin(DEBUG1_GPIO_Port,DEBUG1_Pin,GPIO_PIN_SET);
  if ((RD_DATA_GPIO_Port->IDR & RD_DATA_Pin)==0)                                       // get WR_DATA DO NOT USE THE HAL function creating an overhead
    rdData=0;
  else
    rdData=1;
  //  WritePin(DEBUG1_GPIO_Port,DEBUG1_Pin,GPIO_PIN_RESET);
    
    rdByteWindow<<=1;
    rdByteWindow|=rdData;

  if (rdByteWindow & 0x80){                                                 // Check if ByteWindow Bit 7 is 1 meaning we have a full bytes 0b1xxxxxxx 0x80
    
    packet_bufferRD[rdBytes]=rdByteWindow;
    
    if (rdStartOffset==0 && rdByteWindow==0xC3 && rdBitCounter>10)          // Identify when the message start
      rdStartOffset=0x01;
    
    //printf("%02X",rdByteWindow);
    rdByteWindow=0x0;

    if (rdStartOffset!=0)                                                     // Start writing to packet_buffer only if offset is not 0 (after sync byte)
      rdBytes++;
    
    //if (rdBytes==600)
    //  rdBytes=0;
    
  }
  rdBitCounter++;                                      // Next bit please ;)
}

static void SmartPortReceiveWRDataIRQ(){
  // ADD WR_REQ IRQ TO MANAGE START & STOP OF THE TIMER
  //WritePin(DEBUG1_GPIO_Port,DEBUG1_Pin,GPIO_PIN_SET);
  if ((WR_DATA_GPIO_Port->IDR & WR_DATA_Pin)==0)                                       // get WR_DATA DO NOT USE THE HAL function creating an overhead
      wrData=0;
  else
      wrData=1;
  //WritePin(DEBUG1_GPIO_Port,DEBUG1_Pin,GPIO_PIN_RESET); 
 //printf("za\n");
  //wrData=WR_DATA_GPIO_Port->IDR & WR_DATA_Pin;
  
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


static int decodeDataPacket (char * packet_buffer, int pl){

  int grpbyte, grpcount;
  unsigned char numgrps, numodd;
  unsigned char checksum = 0, bit0to6, bit7, oddbits, evenbits;
  unsigned char group_buffer[8];

  log_info("Data Packet Len:%d",pl);
  
  numodd = packet_buffer[6] & 0x7f;                                                               // Handle arbitrary length packets :) 
  numgrps = packet_buffer[7] & 0x7f;

  for (uint8_t count = 1;  count < 8;  count++)                                                   // First, checksum  packet header, because we're about to destroy it
      checksum = checksum ^ packet_buffer[count];                                                 // now xor the packet header bytes

  log_info("PL-3:%02X PL-2:%02X",packet_buffer[pl-3],packet_buffer[pl-2]);
  evenbits = packet_buffer[pl-3] & 0x55;
  oddbits = (packet_buffer[pl-2] & 0x55 ) << 1;

  log_info("evBits %02X, oddBit:%02X",evenbits,oddbits);
  for(int i = 0; i < numodd; i++){                                                                 //add oddbyte(s), 1 in a 512 data packet
      packet_buffer[i] = ((packet_buffer[8] << (i+1)) & 0x80) | (packet_buffer[9+i] & 0x7f);
  }

  for (grpcount = 0; grpcount < numgrps; grpcount++){                                             // 73 grps of 7 in a 512 byte packet
      memcpy(group_buffer, packet_buffer + 10 + (grpcount * 8), 8);
      for (grpbyte = 0; grpbyte < 7; grpbyte++) {
          bit7 = (group_buffer[0] << (grpbyte + 1)) & 0x80;
          bit0to6 = (group_buffer[grpbyte + 1]) & 0x7f;
          packet_buffer[1 + (grpcount * 7) + grpbyte] = bit7 | bit0to6;
      }
  }

  for (uint8_t count = 0;  count < (numgrps*7+1); count++)                                                   // Verify checksum
     checksum = checksum ^ packet_buffer[count];   

  log_info("decode Data checksum %02X<>%02X",checksum,(oddbits | evenbits));
  print_packet (0,(unsigned char*) packet_buffer,pl);
  
  if (checksum == (oddbits | evenbits))
      return 0;                                                                                   // NO error
  else
      return 6;                                                                                   // Smartport bus error code

}
static uint8_t verifyCmdpktChecksum2(char * packet_buffer,int length){
  int count = 0;
  unsigned char evenbits, oddbits, bit7, bit0to6, grpbyte;
  unsigned char calc_checksum = 0; //initial value is 0
  unsigned char pkt_checksum;


/*
C3              PBEGIN    MARKS BEGINNING OF PACKET             32 micro Sec.       6   0
81              DEST      DESTINATION UNIT NUMBER               32 micro Sec.       7   1
80              SRC       SOURCE UNIT NUMBER                    32 micro Sec.       8   2
80              TYPE      PACKET TYPE FIELD                     32 micro Sec.       9   3
80              AUX       PACKET AUXILLIARY TYPE FIELD          32 micro Sec.      10   4
80              STAT      DATA STATUS FIELD                     32 micro Sec.      11   5
82              ODDCNT    ODD BYTES COUNT                       32 micro Sec.      12   6
81              GRP7CNT   GROUP OF 7 BYTES COUNT                32 micro Sec.      13   7
80              ODDMSB    ODD BYTES MSB's                       32 micro Sec.      14   8
81              COMMAND   1ST ODD BYTE = Command Byte           32 micro Sec.      15   9
83              PARMCNT   2ND ODD BYTE = Parameter Count        32 micro Sec.      16  10
80              GRP7MSB   MSB's FOR 1ST GROUP OF 7              32 micro Sec.      17  11
80              G7BYTE1   BYTE 1 FOR 1ST GROUP OF 7             32 micro Sec.      18  12

0000: C3 81 80 80 80 80 82 81 80 81 83 82 80 88 80 80 - ..80808080..80...80.8080
0010: 80 FF 80 FF BB C8

*/

  unsigned char oddcnt=packet_buffer[SP_ODDCNT]  & 0x7f;
  unsigned char grpcnt=packet_buffer[SP_GRP7CNT] & 0x7f;
    
  for(u_int8_t i = 0; i < oddcnt; i++){                                                                 //add oddbyte(s), 1 in a 512 data packet
      calc_checksum ^= ((packet_buffer[SP_ODDMSB] << (i+1)) & 0x80) | (packet_buffer[SP_COMMAND+i] & 0x7f);
  }

  unsigned char group_buffer[8];

  for (unsigned char grp = 0; grp < grpcnt; grp++){                                             // 73 grps of 7 in a 512 byte packet
    
    memcpy(group_buffer, packet_buffer + SP_COMMAND+ oddcnt + (grp * 8), 8);
    
    for (grpbyte = 0; grpbyte < 7; grpbyte++) {
        bit7 = (group_buffer[0] << (grpbyte + 1)) & 0x80;
        bit0to6 = (group_buffer[grpbyte + 1]) & 0x7f;
        //packet_buffer[1 + (grpcount * 7) + grpbyte] = bit7 | bit0to6;
        calc_checksum ^= bit7 | bit0to6;
    }
  }

  // calculate checksum for overhead bytes
  for (count = 1; count < SP_ODDMSB; count++) // start from first id byte
    calc_checksum ^= packet_buffer[count];

  oddbits = (packet_buffer[length - 2] & 0x55 )<< 1 ;
  evenbits = packet_buffer[length - 3] & 0x55;
    
  pkt_checksum = oddbits | evenbits;

  if ( pkt_checksum == calc_checksum )
      return 1;
  else{
      //print_packet ((unsigned char*) packet_buffer,packet_length());
      log_info("packet_buffer[length - 2]:%02X",packet_buffer[length - 2]);
      log_info("packet_buffer[length - 3]:%02X",packet_buffer[length - 3]);
      log_warn("pkt_chksum:%02X!=calc_chksum:%02X",pkt_checksum,calc_checksum);
      return 0;
  }

}

static uint8_t verifyCmdpktChecksum(char * packet_buffer){
  int count = 0, length;
  unsigned char evenbits, oddbits, bit7, bit0to6, grpbyte;
  unsigned char calc_checksum = 0; //initial value is 0
  unsigned char pkt_checksum;

  length = packet_length(packet_buffer);
/*
C3              PBEGIN    MARKS BEGINNING OF PACKET             32 micro Sec.       6   0
81              DEST      DESTINATION UNIT NUMBER               32 micro Sec.       7   1
80              SRC       SOURCE UNIT NUMBER                    32 micro Sec.       8   2
80              TYPE      PACKET TYPE FIELD                     32 micro Sec.       9   3
80              AUX       PACKET AUXILLIARY TYPE FIELD          32 micro Sec.      10   4
80              STAT      DATA STATUS FIELD                     32 micro Sec.      11   5
82              ODDCNT    ODD BYTES COUNT                       32 micro Sec.      12   6
81              GRP7CNT   GROUP OF 7 BYTES COUNT                32 micro Sec.      13   7
80              ODDMSB    ODD BYTES MSB's                       32 micro Sec.      14   8
81              COMMAND   1ST ODD BYTE = Command Byte           32 micro Sec.      15   9
83              PARMCNT   2ND ODD BYTE = Parameter Count        32 micro Sec.      16  10
80              GRP7MSB   MSB's FOR 1ST GROUP OF 7              32 micro Sec.      17  11
80              G7BYTE1   BYTE 1 FOR 1ST GROUP OF 7             32 micro Sec.      18  12

0000: C3 81 80 80 80 80 82 81 80 81 83 82 80 88 80 80 - ..80808080..80...80.8080
0010: 80 FF 80 FF BB C8

*/

  //unsigned char oddcnt=packet_buffer[SP_ODDCNT] & 0x80;
  //unsigned char grpcnt=packet_buffer[SP_GRP7CNT] & 0x80;

  //2 oddbytes in cmd packet
  calc_checksum ^= ((packet_buffer[SP_ODDMSB] << 1) & 0x80) | (packet_buffer[SP_COMMAND] & 0x7f);
  calc_checksum ^= ((packet_buffer[SP_ODDMSB] << 2) & 0x80) | (packet_buffer[SP_PARMCNT] & 0x7f);

  // 1 group of 7 in a cmd packet
  
  for (grpbyte = 0; grpbyte < 7; grpbyte++) {
      bit7 = (packet_buffer[SP_GRP7MSB] << (grpbyte + 1)) & 0x80;
      bit0to6 = (packet_buffer[SP_G7BYTE1 + grpbyte]) & 0x7f;
      calc_checksum ^= bit7 | bit0to6;
  }

  // calculate checksum for overhead bytes
  for (count = 1; count < 8; count++) // start from first id byte
      calc_checksum ^= packet_buffer[count];

  oddbits = (packet_buffer[length - 2] & 0x55 )<< 1 ;
  evenbits = packet_buffer[length - 3] & 0x55;
  
  pkt_checksum = oddbits | evenbits;

  // calculate checksum for overhead bytes
  

  if ( pkt_checksum == calc_checksum )
      return 1;
  else{
      //print_packet ((unsigned char*) packet_buffer,packet_length());
      log_info("packet_buffer[length - 2]:%02X",packet_buffer[length - 2]);
      log_info("packet_buffer[length - 3]:%02X",packet_buffer[length - 3]);
      log_warn("pkt_chksum:%02X!=calc_chksum:%02X",pkt_checksum,calc_checksum);
      return 0;
  }

}

//*****************************************************************************
// Function: print_packet
// Parameters: pointer to data, number of bytes to be printed
// Returns: none
//
// Description: prints packet data for debug purposes to the serial port
//*****************************************************************************

static void print_packet (unsigned char Emit,unsigned char* data, int bytes){
  int count, row;
  char xx;
  if (Emit==1)
    log_info("HOST Packet size:%03d, src:%02X, dst:%02X, type:%02X, aux:%02x, cmd:%02X, paramcnt:%02X",bytes,data[SP_SRC],data[SP_DEST],data[SP_TYPE],data[SP_AUX],data[SP_COMMAND],data[SP_PARMCNT]);
  else 
    log_info("DEVICE Packet size:%03d, src:%02X, dst:%02X, type:%02X, aux:%02x, cmd:%02X, paramcnt:%02X",bytes,data[SP_SRC],data[SP_DEST],data[SP_TYPE],data[SP_AUX],data[SP_COMMAND],data[SP_PARMCNT]);
 

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
  printf(".\r\n");
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
  
  
  log_info("This is the sound of sea\n\r");
  int T2_DIER=0x0;
  T2_DIER|=TIM_DIER_CC2IE;
  T2_DIER|=TIM_DIER_UIE;
  TIM2->DIER|=T2_DIER;
  
  int T1_DIER=0x0;
  T1_DIER|=TIM_DIER_CC1IE;
  T1_DIER|=TIM_DIER_UIE;
  TIM1->DIER|=T1_DIER;
   
  int packetIdx=0x0;
  while (1){
    switch (phase) {
      // phase lines for smartport bus reset
      // ph3=0 ph2=1 ph1=0 ph0=1
      case 0x05:

      // Monitor phase lines for reset to clear
          while (phase == 0x05);                                                      // Wait for phases to change 
          log_info("Ph:0x05 Reset message");
          break;
      // Phase lines for smartport bus enable
      // Ph3=1 ph2=x ph1=1 ph0=x
      case 0x0a:
      case 0x0b:
      case 0x0e:
      case 0x0f:
            wrStartOffset=0;
            wrBitCounter=0;
            wrBytes=0;
            wrByteWindow=0;
            while (WR_REQ_PHASE==1);
            HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_2);
            WritePin(DEBUG1_GPIO_Port,DEBUG1_Pin,GPIO_PIN_SET); 

            while (WR_REQ_PHASE==0);
            
            HAL_TIM_OC_Stop_IT(&htim2,TIM_CHANNEL_2);
            while (WP_ACK==1);

            WritePin(DEBUG1_GPIO_Port,DEBUG1_Pin,GPIO_PIN_RESET);
            packet_bufferWR[wrBytes]=0x0; 
            //WritePin(DEBUG2_GPIO_Port,DEBUG2_Pin,GPIO_PIN_SET);
            //print_packet(1,packet_bufferWR,wrBytes);
            //verifyCmdpktChecksum2(packet_bufferWR,wrBytes);
            
            if (rdBytes!=0){
              if (rdBytes>24)
                rdBytes=24;
              //print_packet(2,packet_bufferRD,rdBytes);
            }
            //verifyCmdpktChecksum2(packet_bufferRD,rdBytes);
            //WritePin(DEBUG2_GPIO_Port,DEBUG2_Pin,GPIO_PIN_RESET);
            
            rdStartOffset=0;
            rdBitCounter=0;
            rdBytes=0;
            rdByteWindow=0;
            printf("idx:%04d\n",packetIdx);
            
            while (WP_ACK==0);

            // 

            HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);
            WritePin(DEBUG1_GPIO_Port,DEBUG1_Pin,GPIO_PIN_SET); 

            while(1){
              if (WR_REQ_PHASE==0){
                // We have an new imcomming data from Host;
                WritePin(DEBUG2_GPIO_Port,DEBUG2_Pin,GPIO_PIN_SET);
                HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_1);
                wrStartOffset=0;
                wrBitCounter=0;
                wrBytes=0;
                wrByteWindow=0;
                HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_2);
                while (WR_REQ_PHASE==0);
                HAL_TIM_OC_Stop_IT(&htim2,TIM_CHANNEL_2);

                while (WP_ACK==1);
                WritePin(DEBUG2_GPIO_Port,DEBUG2_Pin,GPIO_PIN_RESET); 
                packet_bufferWR[wrBytes]=0x0;
                while (WP_ACK==0);                                      // After That the reading answer from the Device
                rdStartOffset=0;
                rdBitCounter=0;
                rdBytes=0;
                rdByteWindow=0;
                HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);
                WritePin(DEBUG2_GPIO_Port,DEBUG2_Pin,GPIO_PIN_SET); 
                while (WP_ACK==1);
                HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_1);
                packet_bufferRD[rdBytes]=0x0;
                WritePin(DEBUG2_GPIO_Port,DEBUG2_Pin,GPIO_PIN_RESET);       // we have our second request
                break;
              
              }else if (WP_ACK==0){                                         // Normal case of Device Sending after 1 request
                  HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_1);
                  WritePin(DEBUG1_GPIO_Port,DEBUG1_Pin,GPIO_PIN_RESET);
                  packet_bufferRD[rdBytes]=0x0;
                  break;
                
              }
            }

            //while (WP_ACK==1);
            //HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_1);
            //WritePin(DEBUG1_GPIO_Port,DEBUG1_Pin,GPIO_PIN_RESET);

            //while(WP_ACK==0);
            //packet_bufferRD[rdBytes]=0x0;
            packetIdx++;
            

      break;
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  htim1.Init.Period = 32*12+5;
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
  sConfigOC.Pulse = 32*12-15;
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
  htim2.Init.Period = 32*12-1-2;
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
  huart1.Init.BaudRate = 1152000;// 921600; //460800;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG1_GPIO_Port, DEBUG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG2_GPIO_Port, DEBUG2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PHASE0_Pin PHASE1_Pin PHASE2_Pin PHASE3_Pin */
  GPIO_InitStruct.Pin = PHASE0_Pin|PHASE1_Pin|PHASE2_Pin|PHASE3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RD_DATA_Pin WR_DATA_Pin */
  GPIO_InitStruct.Pin = RD_DATA_Pin|WR_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : WR_PROTECT_Pin DEVICE_ENABLE_Pin WR_REQ_Pin */
  GPIO_InitStruct.Pin = WR_PROTECT_Pin|DEVICE_ENABLE_Pin|WR_REQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG1_Pin */
  GPIO_InitStruct.Pin = DEBUG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG2_Pin */
  GPIO_InitStruct.Pin = DEBUG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  //printf("FUCKH %d\n");
  switch (GPIO_Pin){
    
    case DEVICE_ENABLE_Pin:
    SmartPortDeviceEnableIRQ();
    break;
    
    case WR_REQ_Pin:
    SmartPortWrReqIRQ();
    break;
    case WR_PROTECT_Pin:
    SmartPortWpAckIRQ();
    break;
    case PHASE0_Pin:
    case PHASE1_Pin:
    case PHASE2_Pin:
    case PHASE3_Pin:
    SmartPortPhaseIRQ();
    break;
    default:break;


  }
  
  
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

 // int __io_putchar(int ch)
//{
 // Write character to ITM ch.0
 //ITM_SendChar(ch);
 //return(ch);
//}
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
