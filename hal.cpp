#include <stm32f10x_flash.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_adc.h>
#include <stm32f10x.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_spi.h>
#include <misc.h>

#include "hal.h"

#include "uart1.h"
#include "uart3.h"

// #define ADC1_DR_Address    ((uint32_t)0x4001244C)

#if defined(STM32F10X_CL)
#error "clock oscillator problem!"
#endif

void NVIC_InitTable(void)
{
#ifdef  VECT_TAB_RAM
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  // VECT_TAB_FLASH
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
}

void RCC_Init(void)
{ ErrorStatus HSEStartUpStatus;
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if(HSEStartUpStatus == SUCCESS)
  { FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);
    RCC_HCLKConfig(RCC_SYSCLK_Div2); // RCC_SYSCLK_Div2 => 12MHz, RCC_SYSCLK_Div4 => 6MHz
    RCC_PCLK2Config(RCC_HCLK_Div4);
    RCC_PCLK1Config(RCC_HCLK_Div2);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);
    while(RCC_GetSYSCLKSource() != 0x04);
  }
}

#define LED_GREEN  GPIO_Pin_7
#define LED_RED    GPIO_Pin_8

void LED_Init(void)
{ GPIO_InitTypeDef GPIO_Conf;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_Conf.GPIO_Pin = GPIO_Pin_12;
  GPIO_Conf.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Conf.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_Conf);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_Conf.GPIO_Pin = LED_GREEN | LED_RED;
  GPIO_Conf.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Conf.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_Conf);
  GPIO_ResetBits(GPIOA, GPIO_Pin_12);
  GPIO_SetBits(GPIOB, LED_RED);
  GPIO_SetBits(GPIOB, LED_GREEN);
}

void LED_RED_On(uint8_t ON)
{ if(ON) GPIO_ResetBits(GPIOB, LED_RED);
    else GPIO_SetBits  (GPIOB, LED_RED); }

void LED_GREEN_On(uint8_t ON)
{ if(ON) GPIO_ResetBits(GPIOB, LED_GREEN);
    else GPIO_SetBits  (GPIOB, LED_GREEN); }

// ======================================================================

void RFM_RESET(uint8_t On) { }                                  // dummy, as there is no RESET line
bool RFM_IRQ_isOn(void) { return 0; }                           // dummy, as there is no IRQ line

void RFM_Select  (void) { GPIO_ResetBits(GPIOC, GPIO_Pin_13); } // PC13 = LOW
void RFM_Deselect(void) { GPIO_SetBits  (GPIOC, GPIO_Pin_13); } // PC13 = HIGH

uint8_t RFM_TransferByte(uint8_t Byte)
{ SPI_I2S_SendData(SPI2, Byte);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(SPI2); }

void RFM_SPI_Init(void)
{ SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_Conf;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_Conf.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;        // PB13=SPI2_SCK, PB15=SPI2_MOSI
  GPIO_Conf.GPIO_Mode = GPIO_Mode_AF_PP;                 // both pins are outputs
  GPIO_Conf.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_Conf);

  GPIO_Conf.GPIO_Pin = GPIO_Pin_14;                      // PB14 = SPI2_MISO
  GPIO_Conf.GPIO_Mode = GPIO_Mode_IN_FLOATING;           // input
  GPIO_Init(GPIOB, &GPIO_Conf);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_Conf.GPIO_Pin = GPIO_Pin_13;                      // PC13 = si4032 NSS
  GPIO_Conf.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Conf.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOC, &GPIO_Conf);
  GPIO_SetBits(GPIOC, GPIO_Pin_13);                      // set HIGH = inactive

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // _16
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);
  SPI_CalculateCRC(SPI2, DISABLE);
  // SPI_SSOutputCmd(SPI2, ENABLE);
  SPI_Cmd(SPI2, ENABLE);
  // SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  // SPI_Init(SPI2, &SPI_InitStructure);
}

// ======================================================================

SemaphoreHandle_t CONS_Mutex; // console port Mutex

int  CONS_UART_Read  (uint8_t &Byte)  { return UART3_Read (Byte); }
void CONS_UART_Write (char     Byte)  {        UART3_Write(Byte); }
int  CONS_UART_Free  (void)           { return UART3_Free(); }
int  CONS_UART_Full  (void)           { return UART3_Full(); }
void CONS_UART_SetBaudrate(int BaudRate) { UART3_SetBaudrate(BaudRate); }

int   GPS_UART_Read  (uint8_t &Byte)  { return UART1_Read (Byte); }
void  GPS_UART_Write (char     Byte)  {        UART1_Write(Byte); }
void  GPS_UART_SetBaudrate(int BaudRate) { UART1_SetBaudrate(BaudRate); }

// =======================================================================

volatile uint8_t LED_PCB_Counter = 0;
void LED_PCB_Flash(uint8_t Time) { if(Time>LED_PCB_Counter) LED_PCB_Counter=Time; } // [ms]

#ifdef WITH_LED_TX
volatile uint8_t LED_TX_Counter = 0;
void LED_TX_Flash(uint8_t Time) { if(Time>LED_TX_Counter) LED_TX_Counter=Time; } // [ms]
#endif

void LED_TimerCheck(uint8_t Ticks)
{ uint8_t Counter=LED_PCB_Counter;
  if(Counter)
  { if(Ticks<Counter) Counter-=Ticks;
                 else Counter =0;
    if(Counter) LED_PCB_On();
           else LED_PCB_Off();
    LED_PCB_Counter=Counter; }
#ifdef WITH_LED_TX
  Counter=LED_TX_Counter;
  if(Counter)
  { if(Ticks<Counter) Counter-=Ticks;
                 else Counter =0;
    if(Counter) LED_TX_On();
           else LED_TX_Off();
    LED_TX_Counter=Counter; }
#endif
}
