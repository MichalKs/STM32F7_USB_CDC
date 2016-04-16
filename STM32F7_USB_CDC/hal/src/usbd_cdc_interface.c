/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Src/usbd_cdc_interface.c
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    18-November-2015
  * @brief   Source file for USBD CDC interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#include "fifo.h"

#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

USBD_CDC_LineCodingTypeDef LineCoding =
{
  115200, /* baud rate*/
  0x00,   /* stop bits-1*/
  0x00,   /* parity - none*/
  0x08    /* nb. of bits 8*/
};

#define COMM_BUF_LEN_TX 64    ///< COMM buffer lengths
#define COMM_BUF_LEN_RX 64    ///< COMM buffer lengths

static uint8_t rxBuffer[COMM_BUF_LEN_RX]; ///< Buffer for received data.
static uint8_t txBuffer[COMM_BUF_LEN_TX]; ///< Buffer for transmitted data.

static FIFO_TypeDef rxFifo; ///< RX FIFO
static FIFO_TypeDef txFifo; ///< TX FIFO

#define COMM_TERMINATOR '\r'   ///< COMM frame terminator character
static uint8_t gotFrame;  ///< Nonzero signals a new frame (number of received frames)

uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */
//uint32_t BuffLength;
//uint32_t UserTxBufPtrIn = 0;/* Increment this pointer or roll it back to
//                               start address when data are received over USART */
//uint32_t UserTxBufPtrOut = 0; /* Increment this pointer or roll it back to
//                                 start address when data are sent over USB */

/* USB handler declaration */
extern USBD_HandleTypeDef  USBD_Device;

static volatile int vcpConfigured;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive(uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_CDC_fops = 
{
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive
};

int CDC_IsVcpConfigured(void) {
  return vcpConfigured;
}

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len) {
  uint8_t result = USBD_OK;

  USBD_CDC_SetTxBuffer(&USBD_Device, Buf, Len);
  result = USBD_CDC_TransmitPacket(&USBD_Device);
  return result;
}

/**
 * @brief Get a complete frame from USART2 (nonblocking)
 * @param buf Buffer for data (data will be null terminated for easier string manipulation)
 * @param len Length not including terminator character
 * @retval 0 Received frame
 * @retval 1 No frame in buffer
 * @retval 2 Frame error
 * TODO Add maximum length checking so as not to overflow
 */
int CDC_GetFrame(uint8_t* buf, uint8_t* len) {

  uint8_t c;
  *len = 0; // zero out length variable

  if (gotFrame) {
    while (1) {

      // no more data and terminator wasn't reached => error
      if (FIFO_IsEmpty(&rxFifo)) {
        *len = 0;
//        println("Invalid frame");
        return 2;
      }
      FIFO_Pop(&rxFifo, &c);
      buf[(*len)++] = c;

      // if end of frame
      if (c == COMM_TERMINATOR) {
        (*len)--; // length without terminator character
        buf[*len] = 0; // USART terminator character converted to NULL terminator
        break;
      }

    }
    gotFrame--;
    return 0;

  } else {

    return 1;
  }

}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void)
{

//
//  /*##-3- Configure the TIM Base generation  #################################*/
//  TIM_Config();
//
//  /*##-4- Start the TIM Base generation in interrupt mode ####################*/
//  /* Start Channel1 */
//  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
//  {
//    /* Starting Error */
//    Error_Handler();
//  }
  
  // Initialize RX FIFO for receiving data from PC
  rxFifo.buf = rxBuffer;
  rxFifo.len = COMM_BUF_LEN_RX;
  FIFO_Add(&rxFifo);

  // Initialize TX FIFO for transferring data to PC
  txFifo.buf = txBuffer;
  txFifo.len = COMM_BUF_LEN_TX;
  FIFO_Add(&txFifo);

  /*##-5- Set Application Buffers ############################################*/
  USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBuffer, 0);
  USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer);
  
  vcpConfigured = 1;

  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Receive(uint8_t* Buf, uint32_t *Len)
{
//  HAL_UART_Transmit_DMA(&UartHandle, Buf, *Len);

  for (uint32_t i = 0; i < *Len; i++) {
    char c = Buf[i];
    uint8_t res = FIFO_Push(&rxFifo, c); // Put data in RX buffer

    // Checking res to ensure no buffer overflow occurred
    if ((c == COMM_TERMINATOR) && (res == 0)) {
      gotFrame++;
    }
  }

  USBD_CDC_ReceivePacket(&USBD_Device);
  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_DeInit(void)
{
  vcpConfigured = 0;
  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    LineCoding.format     = pbuf[4];
    LineCoding.paritytype = pbuf[5];
    LineCoding.datatype   = pbuf[6];
    
    /* Set the new configuration */
//    ComPort_Config();
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(LineCoding.bitrate);
    pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
    pbuf[4] = LineCoding.format;
    pbuf[5] = LineCoding.paritytype;
    pbuf[6] = LineCoding.datatype;     
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */
    break;

  case CDC_SEND_BREAK:
     /* Add your code here */
    break;    
    
  default:
    break;
  }
  
  return (USBD_OK);
}



/**
  * @brief  TIM_Config: Configure TIMx timer
  * @param  None.
  * @retval None
  */
//static void TIM_Config(void)
//{
//  /* Set TIMx instance */
//  TimHandle.Instance = TIMx;
//
//  /* Initialize TIM3 peripheral as follow:
//       + Period = 10000 - 1
//       + Prescaler = ((SystemCoreClock/2)/10000) - 1
//       + ClockDivision = 0
//       + Counter direction = Up
//  */
//  TimHandle.Init.Period = (CDC_POLLING_INTERVAL*1000) - 1;
//  TimHandle.Init.Prescaler = 84-1;
//  TimHandle.Init.ClockDivision = 0;
//  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
//  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
//  {
//    /* Initialization Error */
//    Error_Handler();
//  }
//}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
