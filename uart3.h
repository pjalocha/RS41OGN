#ifndef __UART3_H__
#define __UART3_H__

#include <stdint.h>

#include "stm32f10x_usart.h"

#include "uart.h"

void UART3_Configuration (int BaudRate=115200);
inline void UART3_SetBaudrate(int BaudRate=115200) { UART_ConfigUSART(USART3, BaudRate); }

int  inline UART3_TxDone(void)    { return USART_GetFlagStatus(USART3, USART_FLAG_TC)   != RESET; } // ready to transmit the next byte
int  inline UART3_TxEmpty(void)   { return USART_GetFlagStatus(USART3, USART_FLAG_TXE)  != RESET; } // all bytes have been transmitted
int  inline UART3_RxReady(void)   { return USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET; } // new byte recieved, to be picked up
// int inline UART3_TxEmpty(void)    { return USART3->SR & USART_FLAG_TXE; }

void inline UART3_TxChar(char ch) { USART_SendData(USART3, ch); }                                   // transmit a byte, but check first TxDone
char inline UART3_RxChar(void)    { return (uint8_t)USART_ReceiveData(USART3); }                    // pick up a new byte, but first check RxReady

int  UART3_Read(uint8_t &Byte);                                                                     // buffered/interrupt driven
void UART3_Write(char Byte);                                                                        // buffered/interrupt driven
void inline UART3_TxKick(void) { USART_ITConfig(USART3, USART_IT_TXE, ENABLE); }
int  UART3_Free(void);                                                                              // how much space in the transmit queue
int  UART3_Full(void);

#endif // __UART3_H__
