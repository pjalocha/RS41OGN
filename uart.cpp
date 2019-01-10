#include "uart.h"

// source:
// https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/DispForm.aspx?ID=27834&Source=/public/STe2ecommunities/mcu/Tags.aspx?tags=stm32%20usart%20interrupt
// https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/DispForm.aspx?ID=24064&Source=/public/STe2ecommunities/mcu/Tags.aspx?tags=stm32%20usart%20interrupt
// http://electronics.stackexchange.com/questions/100073/stm32-usart-rx-interrupts
// https://my.st.com/public/STe2ecommunities/mcu/Lists/STM32Discovery/Flat.aspx?RootFolder=%2Fpublic%2FSTe2ecommunities%2Fmcu%2FLists%2FSTM32Discovery%2FUART%20example%20code%20for%20STM32F$

// UART pins:
// Pin  Function
//
// PA8  USART1_CK
// PA11 USART1_CTS
// PA12 USART1_RTS
// PA9  USART1_TX
// PA10 USART1_RX
//
// PA4  USART2_CK
///PA0  USART2_CTS
// PA1  USART2_RTS
// PA2  USART2_TX
// PA3  USART2_RX
//
// PB12 USART3_CK
// PB13 USART3_CTS
// PB14 USART3_RTS
// PB10 USART3_TX
// PB11 USART3_RX

void UART_ConfigNVIC(uint8_t IRQ, uint8_t Priority, uint8_t SubPriority)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = IRQ;                     // Enable the USART Interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = Priority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void UART_ConfigGPIO(GPIO_TypeDef* GPIO, uint16_t InpPin, uint16_t OutPin)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin   = InpPin;         // Configure USART Rx (input) pin as input floating
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = OutPin;          // Configure USART Tx (output) pin as alternate function push-pull
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(GPIO, &GPIO_InitStructure);
}

void UART_ConfigUSART(USART_TypeDef* USART, int BaudRate)
{
  USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef USART_ClockInitStructure;

  USART_InitStructure.USART_BaudRate            = BaudRate; // UART2 at 9600bps (GPS)
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

  USART_ClockInitStructure.USART_Clock          = USART_Clock_Disable;
  USART_ClockInitStructure.USART_CPOL           = USART_CPOL_Low;
  USART_ClockInitStructure.USART_CPHA           = USART_CPHA_2Edge;
  USART_ClockInitStructure.USART_LastBit        = USART_LastBit_Disable;

  USART_Init     (USART, &USART_InitStructure);
  USART_ClockInit(USART, &USART_ClockInitStructure);   // write parameters
}

