
#include <string.h>
#include <stm32f7xx_hal.h>
#include <stm32746g_discovery_lcd.h>
#include <stm32746g_discovery_ts.h>
#include <stm32746g_discovery_sd.h>
#include "led.h"
#include "timers.h"
#include "system.h"
#include "comm.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"

#ifdef DEBUG
#define print(str, args...) printf(""str"%s",##args,"")
#define println(str, args...) printf("MAIN--> "str"%s",##args,"\r\n")
#else
#define print(str, args...) (void)0
#define println(str, args...) (void)0
#endif

/**
  * @brief  LCD FB_StartAddress
  * LCD Frame buffer start address : starts at beginning of SDRAM
  */
#define LCD_FRAME_BUFFER          SDRAM_DEVICE_ADDR

USBD_HandleTypeDef USBD_Device;

void softTimerCallback(void) {
  LED_Toggle(_LED0);
//  println("Test string sent from STM32F7!!!"); // Print test string
}

/**
 * @brief Main function
 * @return
 */
int main(void) {

  SYSTEM_Init(); // Initialize STM32F7 and HAL (SYSTICK)

  // Add a soft timer with callback running every 1000ms
  int8_t timerID = TIMER_AddSoftTimer(500, softTimerCallback);
  TIMER_StartSoftTimer(timerID); // start the timer

  LED_Init(_LED0); // Add an LED

  //*******************LCD test
//  BSP_LCD_Init();
//  BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FRAME_BUFFER);

  // Set LCD Foreground Layer
//  BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);

  // Clear the LCD
//  BSP_LCD_SetBackColor(LCD_COLOR_DARKMAGENTA);
//  BSP_LCD_Clear(LCD_COLOR_DARKMAGENTA);

  /* Set the LCD Text Color */
//  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

  /* Display LCD messages */
//  BSP_LCD_DisplayStringAtLine(0, (uint8_t *)"STM32F7 Discovery");
//  BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"Hello World!!!");
//
//  BSP_LCD_DrawLine(50, 50, 150, 150);
//  BSP_LCD_FillRect(100, 100, 100, 100);
//  BSP_LCD_FillCircle(300, 50, 40);

//  BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

//  TS_StateTypeDef tsState;

  //*******************LCD test END

//  BSP_SD_Init();

//  HAL_SD_CardInfoTypedef cardInfo;

//  if (BSP_SD_IsDetected()) {
//
//    BSP_SD_GetCardInfo(&cardInfo);
//
//    println("SD card has %d blocks", cardInfo.CardBlockSize);
//
//
//  }

  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);

  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);

  /* Start Device Process */
  USBD_Start(&USBD_Device);

  uint8_t buf[255]; // buffer for receiving commands from PC
  uint8_t len;      // length of command

  while (1) {

    // check for new frames from PC
//    if (!COMM_GetFrame(buf, &len)) {
//      println("Got frame of length %d: %s", (int)len, (char*)buf);
//
//      // control LED0 from terminal
//      if (!strcmp((char*)buf, ":LED0 ON")) {
//        LED_ChangeState(_LED0, LED_ON);
//      }
//      if (!strcmp((char*)buf, ":LED0 OFF")) {
//        LED_ChangeState(_LED0, LED_OFF);
//      }
//    }

    // check for touch screen events
//    BSP_TS_GetState(&tsState);
//    if (tsState.touchDetected == 1) {
//      LED_Toggle(_LED0);
//      BSP_TS_Get_GestureId(&tsState);
//      if (tsState.gestureId != 0)
//        println("Gesture id = %d", tsState.gestureId);
//
////      BSP_TS_ResetTouchData(&tsState);
//    }

    TIMER_SoftTimersUpdate(); // run timers
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLLSAI_N                       = 384
  *            PLLSAI_P                       = 8
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
//void SystemClock_Config(void)
//{
//  RCC_ClkInitTypeDef RCC_ClkInitStruct;
//  RCC_OscInitTypeDef RCC_OscInitStruct;
//  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
//
//  /* Enable HSE Oscillator and activate PLL with HSE as source */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 25;
//  RCC_OscInitStruct.PLL.PLLN = 432;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 9;
//  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Activate the OverDrive to reach the 216 Mhz Frequency */
//  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Select PLLSAI output as USB clock source */
//  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
//  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
//  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
//  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 7;
//  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
//  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct)  != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
//     clocks dividers */
//  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

