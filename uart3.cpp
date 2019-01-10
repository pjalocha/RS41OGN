
#include <FreeRTOS.h>
#include <task.h>

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#include "fifo.h"

#include "config.h"
#include "uart3.h"

FIFO<uint8_t, UART3_RxFIFO_Size> UART3_RxFIFO;
FIFO<uint8_t, UART3_TxFIFO_Size> UART3_TxFIFO;

void UART3_Configuration (int BaudRate)
{
  UART_ConfigNVIC(USART3_IRQn, 0, 0);                   // Configure and enable the USART3 Interrupt

  // RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO ,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  // UART_ConfigGPIO(GPIOC, GPIO_Pin_11, GPIO_Pin_10);     // Configure USART3 Rx (PC11) as input, and USART3 Tx (PC10) as output
  UART_ConfigGPIO(GPIOB, GPIO_Pin_11, GPIO_Pin_10);     // Configure USART3 Rx (PB11) as input, and USART3 Tx (PB10) as output
  UART_ConfigUSART(USART3, BaudRate);

  UART3_RxFIFO.Clear(); UART3_TxFIFO.Clear();
  USART_Cmd(USART3, ENABLE);                            // Enable USART1
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);        // Enable Rx-not-empty interrupt
  // NVIC_EnableIRQ(USART3_IRQn);
}

#ifdef __cplusplus
  extern "C"
#endif
void USART3_IRQHandler(void)
{ if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
   while(UART3_RxReady()) { uint8_t Byte=UART3_RxChar(); UART3_RxFIFO.Write(Byte); } // write received bytes to the RxFIFO
  if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
   while(UART3_TxEmpty())
  { uint8_t Byte;
    if(UART3_TxFIFO.Read(Byte)<=0) { USART_ITConfig(USART3, USART_IT_TXE, DISABLE); break; }
    UART3_TxChar(Byte); }
  // USART_ClearITPendingBit(USART1,USART_IT_RXNE);
  // if other UART3 interrupt sources ...
  // USART_ClearITPendingBit(USART1, USART_IT_TXE);
}

int UART3_Read(uint8_t &Byte) { return UART3_RxFIFO.Read(Byte); } // return number of bytes read (0 or 1)

void UART3_Write(char Byte)
{ if(UART3_TxFIFO.isEmpty()) { UART3_TxFIFO.Write(Byte); UART3_TxKick(); return; }
  if(UART3_TxFIFO.Write(Byte)>0) return;
  UART3_TxKick();
  while(UART3_TxFIFO.Write(Byte)<=0) vTaskDelay(1); // taskYIELD();
  return; }

int UART3_Free(void) { return UART3_TxFIFO.Free(); }
int UART3_Full(void) { return UART3_TxFIFO.Full(); }
