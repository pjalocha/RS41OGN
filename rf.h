#include <stdint.h>

#include "hal.h"

// #ifdef __cplusplus

#include "ogn.h"
#include "rfm.h"
#include "fifo.h"
#include "freqplan.h"

  extern FIFO<OGN_TxPacket<OGN_Packet>, 4> RF_TxFIFO;   // buffer for transmitted packets

  extern int16_t        RF_Temp;              // [degC] temperature of the RF chip
  extern uint16_t       RF_VCC;               // [0.01V]
  extern FreqPlan   RF_FreqPlan;              // frequency hopping pattern calculator
  extern uint16_t     TX_Credit;              // counts transmitted packets vs. time to avoid using more than 1% of the time
  extern uint32_t     RX_Random;              // Random number from LSB of RSSI readouts

// #endif

// #ifdef __cplusplus
extern "C"
// #endif
 void vTaskRF(void* pvParameters);
