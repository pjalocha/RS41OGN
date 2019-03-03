#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "config.h"

#include "parameters.h"

#ifndef __HAL_H__
#define __HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

extern FlashParameters Parameters;

void NVIC_InitTable(void);

void RCC_Init(void);

void IO_Init(void);

void Power_On(uint8_t ON);
void LED_RED_On(uint8_t ON);
void LED_GREEN_On(uint8_t ON);

inline void LED_PCB_On   (void) { LED_GREEN_On(1); }                // LED on the PCB for vizual indications
inline void LED_PCB_Off  (void) { LED_GREEN_On(0); }

inline void LED_TX_On   (void) { LED_RED_On(1); }                   // LED on the PCB for vizual indications
inline void LED_TX_Off  (void) { LED_RED_On(0); }

void ADC1_Init(void);
uint16_t ADC1_Read(uint8_t Channel);
uint16_t ADC_Read_Vsupply  (void);
uint16_t ADC_Read_Vbutton  (void);
uint16_t ADC_Read_MCU_Vtemp(void);
uint16_t ADC_Read_MCU_Vref (void);

// =========================================================================================================

extern SemaphoreHandle_t CONS_Mutex; // console port Mutex

int  CONS_UART_Read       (uint8_t &Byte); // non-blocking
void CONS_UART_Write      (char     Byte); // blocking
int  CONS_UART_Free       (void);          // how many bytes can be written to the transmit buffer
int  CONS_UART_Full       (void);          // how many bytes already in the transmit buffer
void CONS_UART_SetBaudrate(int BaudRate);
int   GPS_UART_Read       (uint8_t &Byte); // non-blocking
void  GPS_UART_Write      (char     Byte); // blocking
void  GPS_UART_SetBaudrate(int BaudRate);

void LED_PCB_Flash(uint8_t Time);     // [ms] turn on the PCB LED for a given time
void LED_TX_Flash(uint8_t Time);     // [ms] turn on the PCB LED for a given time

// =========================================================================================================

void RFM_SPI_Init(void);

void    RFM_RESET(uint8_t On);           // RF module reset
void    RFM_Select  (void);              // SPI select
void    RFM_Deselect(void);              // SPI de-select
uint8_t RFM_TransferByte(uint8_t Byte);  // SPI transfer/exchange a byte
bool    RFM_IRQ_isOn(void);              // query the IRQ state

// =========================================================================================================

void LED_TimerCheck(uint8_t Ticks);

#ifdef __cplusplus
}
#endif

#endif // __HAL_H__
