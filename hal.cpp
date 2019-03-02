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

// ======================================================================
// list of (known) STM32F1 pins

// PA5  = ADC1 AIN
// PA6  = ADC1 AIN
// PA9  = UART1_TX (GPS)
// PA10 = UART1_RX (GPS)
// PA11 =
// PA12 = power converter: HIGH = power OFF ?

// PB7  = Green LED, low-active
// PB8  = Red   LED, low-active
// PB9  =
// PB10 = UART3_TX
// PB11 = UART3_RX
// PB12 =
// PB13 = SPI2_SCK
// PB14 = SPI2_MISO
// PB15 = SPI2_MOSI

// PC13 = Si4032 chip select (with SPI2)

// ======================================================================

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
  RCC_HSEConfig(RCC_HSE_ON);                                 // High Speed External oscilator thus Xtal
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if(HSEStartUpStatus == SUCCESS)
  { FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);
    RCC_HCLKConfig(RCC_SYSCLK_Div2);                         // _Div2 => 12MHz, _Div4 => 6MHz
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
  GPIO_Conf.GPIO_Pin = GPIO_Pin_12;                          // this pin has to do with power control but not clear how ?
  GPIO_Conf.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Conf.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_Conf);
  GPIO_ResetBits(GPIOA, GPIO_Pin_12);                        // turn the power ON

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_Conf.GPIO_Pin = LED_GREEN | LED_RED;                  // red and green LED
  GPIO_Conf.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Conf.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_Conf);
  GPIO_SetBits(GPIOB, LED_RED);                              // turn off the LED's
  GPIO_SetBits(GPIOB, LED_GREEN);
}

void LED_RED_On(uint8_t ON)
{ if(ON) GPIO_ResetBits(GPIOB, LED_RED);                     // LED control is low-active
    else GPIO_SetBits  (GPIOB, LED_RED); }

void LED_GREEN_On(uint8_t ON)
{ if(ON) GPIO_ResetBits(GPIOB, LED_GREEN);                   // LED control is low-active
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

void ADC1_Init(void)
{
  ADC_InitTypeDef  ADC_InitStructure;
  RCC_ADCCLKConfig(RCC_PCLK2_Div2);                                    // PCLK2 is the APB2 clock, ADCCLK = PCLK2/6 = 12/2 = 6MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);                 // Enable ADC1 clock so that we can talk to it
  ADC_DeInit(ADC1);                                                    // Put everything back to power-on defaults

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                   // ADC2 not depenedent on ADC1
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;                        // Disable the scan conversion so we do one at a time
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                  // Don't do contimuous conversions - do them on demand
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  // Start conversin by software, not an external trigger
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;               // Conversions are 12 bit - put them in the lower 12 bits of t$
  ADC_InitStructure.ADC_NbrOfChannel = 1;                              // How many channels would be used by the sequencer
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_Cmd(ADC1, ENABLE);

  ADC_ResetCalibration(ADC1);                                           // Enable ADC1 reset calibaration register
  while(ADC_GetResetCalibrationStatus(ADC1));                           // Check the end of ADC1 reset calibration register
  ADC_StartCalibration(ADC1);                                           // Start ADC1 calibaration
  while(ADC_GetCalibrationStatus(ADC1));                                // Check the end of ADC1 calibration
  ADC_TempSensorVrefintCmd(ENABLE);                                     // enable Vrefint and Temperature sensor

  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;                           // Pin #5
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;                        // as analog input (knob)
  GPIO_Init(GPIOA, &GPIO_InitStructure);                                // for Port A
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;                           // Pin #6
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;                        // as analog input (battery voltage sense)
  GPIO_Init(GPIOA, &GPIO_InitStructure);                                // for Port A
}

uint16_t ADC1_Read(uint8_t Channel)                                     // convert and read given channel
{
  ADC_RegularChannelConfig(ADC1, Channel, 1, ADC_SampleTime_55Cycles5);
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);                               // Start the conversion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);                // Wait until conversion complete
  return ADC_GetConversionValue(ADC1);                                  // Get the conversion value
}

// temperatue sensor channel = ADC_Channel_TempSensor = ADC_Channel_16
// internal reference channel = ADC_Channel_Vrefint   = ADC_Channel_17
// PA5 = ADC_Channel_5
// PA6 = ADC_Channel_6

uint16_t ADC_Read_Vsupply  (void) { return ADC1_Read(ADC_Channel_5); }
uint16_t ADC_Read_Vbutton  (void) { return ADC1_Read(ADC_Channel_6); }
uint16_t ADC_Read_MCU_Vtemp(void) { return ADC1_Read(ADC_Channel_16); }
uint16_t ADC_Read_MCU_Vref (void) { return ADC1_Read(ADC_Channel_17); }

/*

Readout on external power:
ADC: Vref=01643, Vtemp=01944, Vsupply=00037, Vbutton=00030
ADC: Vref=01642, Vtemp=01944, Vsupply=00037, Vbutton=00031
ADC: Vref=01642, Vtemp=01945, Vsupply=00037, Vbutton=00031
ADC: Vref=01642, Vtemp=01944, Vsupply=00037, Vbutton=00030

Readout after pressing the Power-button
ADC: Vref=01643, Vtemp=01945, Vsupply=02264, Vbutton=02234
ADC: Vref=01643, Vtemp=01944, Vsupply=02232, Vbutton=02203
ADC: Vref=01643, Vtemp=01944, Vsupply=02238, Vbutton=02208
ADC: Vref=01643, Vtemp=01943, Vsupply=02238, Vbutton=02208
ADC: Vref=01643, Vtemp=01944, Vsupply=02245, Vbutton=02215
ADC: Vref=01643, Vtemp=01945, Vsupply=02231, Vbutton=02201
ADC: Vref=01642, Vtemp=01945, Vsupply=02291, Vbutton=02260

*/

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
