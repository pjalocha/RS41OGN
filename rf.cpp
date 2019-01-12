#include "hal.h"
#include "rf.h"

#include "gps.h"

#include "timesync.h"

// OGNv1 SYNC:       0x0AF3656C encoded in Manchester
// static const uint8_t OGN1_SYNC[8] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A };
// OGNv2 SYNC:       0xF56D3738 encoded in Machester
// static const uint8_t OGN2_SYNC[8] = { 0x55, 0x99, 0x96, 0x59, 0xA5, 0x95, 0xA5, 0x6A };
//
//
// #ifdef WITH_OGN1
// static const uint8_t *OGN_SYNC = OGN1_SYNC;
// #endif

// #ifdef WITH_OGN2
// static const uint8_t *OGN_SYNC = OGN2_SYNC;
// #endif

const uint32_t OGN1_SYNC = 0x0AF3656C;
const uint32_t OGN2_SYNC = 0xF56D3738;

#ifdef WITH_OGN1
static const uint32_t OGN_SYNC = OGN1_SYNC;
#endif

#ifdef WITH_OGN2
static const uint32_t OGN_SYNC = OGN2_SYNC;
#endif

static RFM_TRX           TRX;               // radio transceiver

        int8_t       RF_Temp;               // [degC] temperature of the RF chip: uncalibrated

static uint32_t  RF_SlotTime;               // [sec] UTC time which belongs to the current time slot (0.3sec late by GPS UTC)
       FreqPlan  RF_FreqPlan;               // frequency hopping pattern calculator

       FIFO<OGN_TxPacket<OGN_Packet>,   4> RF_TxFIFO;   // buffer for transmitted packets

       uint16_t TX_Credit  =0;              // counts transmitted packets vs. time to avoid using more than 1% of the time

      uint32_t RX_Random=0x12345678;        // Random number from LSB of RSSI readouts

/*
static void SetTxChannel(uint8_t TxChan=RX_Channel)         // default channel to transmit is same as the receive channel
{ TRX.WriteTxPower(Parameters.getTxPower());                // set TX for transmission
  TRX.setChannel(TxChan&0x7F);
  TRX.WriteSYNC(8, 7, OGN_SYNC); }                          // Full SYNC for TX

static uint8_t Transmit(uint8_t TxChan, const uint8_t *PacketByte)
{
  if(PacketByte==0) return 0;                                   // if no packet to send: simply return

#ifdef WITH_LED_TX
  LED_TX_Flash(20);
#endif
  TRX.WriteMode(RF_OPMODE_STANDBY);                              // switch to standby
  vTaskDelay(1);
  SetTxChannel(TxChan);

  TRX.ClearIrqFlags();
  TRX.WritePacket(PacketByte);                                   // write packet into FIFO
  TRX.WriteMode(RF_OPMODE_TRANSMITTER);                          // transmit
  vTaskDelay(5);                                                 // wait 5ms

  uint8_t Break=0;
  for(uint16_t Wait=400; Wait; Wait--)                           // wait for transmission to end
  { uint16_t Flags=TRX.ReadIrqFlags();
    // if(Mode!=RF_OPMODE_TRANSMITTER) break;
    if(Flags&RF_IRQ_PacketSent) Break++;
    if(Break>=2) break; }
  TRX.WriteMode(RF_OPMODE_STANDBY);                              // switch to standy

  return 1; }

static uint32_t IdleUntil(TickType_t End)
{ uint32_t Count=0;
  for( ; ; )
  { int32_t Left = End-xTaskGetTickCount();
    if(Left<=0) break;
    vTaskDelay(1); }
  return Count; }

static void TimeSlot(uint8_t TxChan, uint32_t SlotLen, const uint8_t *PacketByte, uint32_t TxTime=0)
{ TickType_t Start = xTaskGetTickCount();                                  // when the slot started
  TickType_t End   = Start + SlotLen;                                      // when should it end
  uint32_t MaxTxTime = SlotLen-8-MaxWait;                                  // time limit when transmision could start
  if( (TxTime==0) || (TxTime>=MaxTxTime) ) TxTime = RX_Random%MaxTxTime;   // if TxTime out of limits, setup a random TxTime
  TickType_t Tx    = Start + TxTime;                                       // Tx = the moment to start transmission
  IdleUntil(Tx);                                                        // listen until this time comes
  if( (TX_Credit) && (PacketByte) )                                        // when packet to transmit is given and there is still TX $
    TX_Credit-=Transmit(TxChan, PacketByte, Rx_RSSI, MaxWait);             // attempt to transmit the packet
  IdleUntil(End);                                                       // listen till the end of the time-slot
}
*/

static void SetFreqPlan(void)                                // set the RF TRX according to the selected frequency hopping plan
{ TRX.setBaseFrequency(RF_FreqPlan.BaseFreq);                // set the base frequency (recalculate to RFM69 internal synth. units)
  TRX.setChannelSpacing(RF_FreqPlan.ChanSepar);              // set the channel separation
  TRX.setFrequencyCorrection(Parameters.RFchipFreqCorr);     // set the fine correction (to counter the Xtal error)
}

static uint8_t StartRFchip(void)
{ // TRX.RESET(1);                                              // RESET active
  // vTaskDelay(10);                                            // wait 10ms
  // TRX.RESET(0);                                              // RESET released
  // vTaskDelay(10);                                            // wait 10ms
  TRX.SoftReset();
  vTaskDelay(2);                                                // wait 2ms
  SetFreqPlan();                                                // set TRX base frequency and channel separation after the frequency hop$
  TRX.Configure(0, OGN_SYNC);                                   // setup RF chip parameters and set to channel #0
  // TRX.WriteMode(RF_OPMODE_STANDBY);                          // set RF chip mode to STANDBY
  TRX.WriteTxPower(Parameters.RFchipTxPower);                   // [dBm]
  uint8_t Version = TRX.ReadVersion();
// #ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "StartRFchip() v");
  Format_Hex(CONS_UART_Write, Version);
  CONS_UART_Write(' ');
  Format_UnsDec(CONS_UART_Write, RF_FreqPlan.BaseFreq, 4, 3);
  CONS_UART_Write('+');
  Format_UnsDec(CONS_UART_Write, (uint16_t)RF_FreqPlan.Channels, 2);
  CONS_UART_Write('x');
  Format_UnsDec(CONS_UART_Write, RF_FreqPlan.ChanSepar, 4, 3);
  Format_String(CONS_UART_Write, "kHz\n");
  TRX.RegDump(CONS_UART_Write);
  xSemaphoreGive(CONS_Mutex);
// #endif
  return Version; }                                          // read the RF chip version and return it

extern "C"
 void vTaskRF(void* pvParameters)
{
  RF_TxFIFO.Clear();

#ifdef USE_BLOCK_SPI
  TRX.TransferBlock = RFM_TransferBlock;
#else
  TRX.Select       = RFM_Select;
  TRX.Deselect     = RFM_Deselect;
  TRX.TransferByte = RFM_TransferByte;
#endif
  TRX.DIO0_isOn    = RFM_IRQ_isOn;
  TRX.RESET        = RFM_RESET;

  RF_FreqPlan.setPlan(Parameters.FreqPlan);     // 1 = Europe/Africa, 2 = USA/CA, 3 = Australia and South America

  vTaskDelay(5);

  for( ; ; )
  { uint8_t ChipVersion = StartRFchip();

    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "TaskRF: ");
    CONS_UART_Write('v'); Format_Hex(CONS_UART_Write, ChipVersion);
    CONS_UART_Write(' '); Format_SignDec(CONS_UART_Write, TRX.ReadChipTemp(), 2, 1);
    Format_String(CONS_UART_Write,"degC, ");
    Format_UnsDec(CONS_UART_Write, TRX.ReadBatVolt(), 3, 2);
    Format_String(CONS_UART_Write,"V\n");
    xSemaphoreGive(CONS_Mutex);

    if( (ChipVersion!=0x00) && (ChipVersion!=0xFF) ) break;  // only break the endless loop then an RF chip is detected
    vTaskDelay(1000);
  }

  TX_Credit      = 0;    // count slots and packets transmitted: to keep the rule of 1% transmitter duty cycle

  // TRX.WriteMode(RF_OPMODE_STANDBY);

  OGN_TxPacket<OGN_Packet> TxPacket;
  TxPacket.Packet.HeaderWord=0;
  TxPacket.Packet.Header.Address=0x123456;
  TxPacket.Packet.Header.AddrType=0x0;
  TxPacket.Packet.calcAddrParity();

  for( ; ; )
  { vTaskDelay(100);
    GPS_Position *Position = GPS_getPosition(); if(Position==0) continue;
    TxPacket.Packet.Position.AcftType=0xF;
    Position->Encode(TxPacket.Packet);
    TxPacket.Packet.Whiten();
    TxPacket.calcFEC();
    TRX.WritePacket((uint8_t *)(&TxPacket));
    TRX.Transmit();
    vTaskDelay(100);
  }

}



