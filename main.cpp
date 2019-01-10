#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_rcc.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <misc.h>

#include "FreeRTOS.h"
#include "task.h"

#include "format.h"

#include "config.h"

#include "hal.h"

#include "uart1.h"
#include "uart3.h"

#include "gps.h"
#include "ctrl.h"

#include "parameters.h"

#include "uniqueid.h"
uint32_t getUniqueAddress(void)
{ uint32_t ID = UniqueID[0] ^ UniqueID[1] ^ UniqueID[2]; return ID&0x00FFFFFF; }

FlashParameters Parameters;

void print( const char* s)
{
  while (*s)
  { while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
    { }
    USART_SendData(USART3, *(s++));
  }
}


extern "C"
int main(void)
{
  RCC_Init();
  NVIC_InitTable();
  LED_Init();
  RFM_SPI_Init();

  // Parameters.setDefault();
  if(Parameters.ReadFromFlash()<0)             // read parameters from Flash
  { Parameters.setDefault();                   // if nov valid: set defaults
    Parameters.WriteToFlash(); }               // and write the defaults back to Flash

  UART1_Configuration(9600);                   // GPS UART
  UART3_Configuration(Parameters.CONbaud);     // Console UART
  // UART3_Configuration(115200);                 // Console UART
  CONS_Mutex = xSemaphoreCreateMutex();        // Concole MUTEX

  print("\r\nRS41OGN test\r\n");

  LED_GREEN_On(1);
  // Format_String(UART3_Write, "RS41OGN: interrupt/buffered\n");
  // LED_GREEN_On(0);

  xTaskCreate(vTaskGPS,   "GPS",    120, 0, tskIDLE_PRIORITY+1, 0);  // GPS: GPS NMEA/PPS, packet encoding

  xTaskCreate(vTaskCTRL,  "CTRL",   120, 0, tskIDLE_PRIORITY  , 0);  // CTRL: console interaction

  vTaskStartScheduler();

/*
  while(1)
  { if(!UART1_RxReady()) continue;
    LED_RED_On(1);
    char Byte=UART1_RxChar();
    UART3_TxChar(Byte);
    // while(!UART3_TxEmpty()) { }
    // UART3_TxChar('.');
    LED_RED_On(0);
    // NVIC_SystemLPConfig(NVIC_LP_SEVONPEND, DISABLE);
    // __WFI();
  }
*/

}

extern "C"
void vApplicationIdleHook(void) // when RTOS is idle: should call "sleep until an interrupt"
{ __WFI();                      // wait-for-interrupt
}

extern "C"
void vApplicationTickHook(void) // RTOS timer tick hook
{ // IWDG_ReloadCounter();         // reset watch-dog at every tick (primitive, but enough to start)

  LED_TimerCheck(1);
}

