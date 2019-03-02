#include "hal.h"
#include "rf.h"

#include "gps.h"

#include "timesync.h"

#ifdef WITH_FLASHLOG                  // log own track to unused Flash pages (STM32 only)
#include "flashlog.h"
#endif

const uint32_t OGN1_SYNC = 0x0AF3656C; // OGN SYNC
const uint32_t OGN2_SYNC = 0xF56D3738; // OGNv2 SYNC

#ifdef WITH_OGN1
static const uint32_t OGN_SYNC = OGN1_SYNC;
#endif

#ifdef WITH_OGN2
static const uint32_t OGN_SYNC = OGN2_SYNC;
#endif

static RFM_TRX           TRX;               // radio transceiver

        int16_t       RF_Temp;              // [0.1degC] temperature of the RF chip: uncalibrated
       uint16_t       RF_VCC;               // [0.01V]

static uint32_t  RF_SlotTime;               // [sec] UTC time which belongs to the current time slot (0.3sec late by GPS UTC)
       FreqPlan  RF_FreqPlan;               // frequency hopping pattern calculator

       FIFO<OGN_TxPacket<OGN_Packet>,   4> RF_TxFIFO;   // buffer for transmitted packets

       uint16_t TX_Credit  =0;              // counts transmitted packets vs. time to avoid using more than 1% of the time

      uint32_t RX_Random=0x12345678;        // Random number from LSB of RSSI readouts

static uint8_t Transmit(const uint8_t *PacketByte)
{ if(PacketByte==0) return 0;                                   // if no packet to send: simply return
#ifdef WITH_LED_TX
  LED_TX_Flash(20);
#endif
  TRX.WritePacket(PacketByte);                                   // write packet into FIFO
  TRX.Transmit();                                                // trigger the transmission
  vTaskDelay(6);
  return 1; }

static uint32_t IdleUntil(TickType_t End)
{ uint32_t Count=0;
  for( ; ; )
  { int32_t Left = End-xTaskGetTickCount();
    if(Left<=0) break;
    vTaskDelay(1); }
  return Count; }

static void TimeSlot(uint32_t SlotLen, const uint8_t *PacketByte, uint32_t TxTime=0)
{ TickType_t Start = xTaskGetTickCount();                                  // when the slot started
  TickType_t End   = Start + SlotLen;                                      // when should it end
  uint32_t MaxTxTime = SlotLen-8;                                          // time limit when transmision could start
  if( (TxTime==0) || (TxTime>=MaxTxTime) ) TxTime = RX_Random%MaxTxTime;   // if TxTime out of limits, setup a random TxTime
  TickType_t Tx    = Start + TxTime;                                       // Tx = the moment to start transmission
  IdleUntil(Tx);                                                           // listen until this time comes
  if( (TX_Credit) && (PacketByte) )                                        // when packet to transmit is given and there is still TX $
    TX_Credit-=Transmit(PacketByte);                                       // attempt to transmit the packet
  IdleUntil(End);                                                          // listen till the end of the time-slot
}

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
  TRX.WriteTxPower(Parameters.RFchipTxPower);                   // [dBm]
  uint8_t Version = TRX.ReadVersion();
#ifdef DEBUG_PRINT
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
#endif
  return Version; }                                          // read the RF chip version and return it

extern "C"
 void vTaskRF(void* pvParameters)
{
#ifdef WITH_FLASHLOG
  uint16_t kB = FlashLog_OpenForWrite();
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "TaskRF: ");
  Format_UnsDec(CONS_UART_Write, kB);
  Format_String(CONS_UART_Write, "KB FlashLog\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  RF_TxFIFO.Clear();

#ifdef USE_BLOCK_SPI
  TRX.TransferBlock = RFM_TransferBlock;
#else
  TRX.Select       = RFM_Select;
  TRX.Deselect     = RFM_Deselect;
  TRX.TransferByte = RFM_TransferByte;
#endif
  TRX.DIO0_isOn    = RFM_IRQ_isOn;              // dummy call
  TRX.RESET        = RFM_RESET;                 // dummy call

  RF_FreqPlan.setPlan(Parameters.FreqPlan);     // 1 = Europe/Africa:868MHz, 5 = Europe/Africe:433MHz

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

  OGN_TxPacket<OGN_Packet> TxPacket;
  TxPacket.Packet.HeaderWord=0;
  TxPacket.Packet.Header.Address=Parameters.Address;
  TxPacket.Packet.Header.AddrType=Parameters.AddrType;
  TxPacket.Packet.calcAddrParity();

  for( ; ; )
  { do
    { vTaskDelay(1); }
    while(TimeSync_msTime()<290);                // wait till 300ms after PPS

    SetFreqPlan();
    RF_Temp = TRX.ReadChipTemp();
    RF_VCC  = TRX.ReadBatVolt();
    StartRFchip();

    uint16_t MCU_Vref  = ADC_Read_MCU_Vref();
    uint16_t MCU_Vtemp = ADC_Read_MCU_Vtemp();
    uint16_t Vsupply   = ADC_Read_Vsupply();
    uint16_t Vbutton   = ADC_Read_Vbutton();
    uint16_t MCU_VCC   = ( ((uint32_t)120<<12)+(MCU_Vref>>1))/MCU_Vref; // [0.01V]
     int16_t MCU_Temp  = 250 + ( ( ( (int32_t)1430 - ((int32_t)1200*(int32_t)MCU_Vtemp+(MCU_Vref>>1))/MCU_Vref )*(int32_t)37 +8 )>>4); // [0.1degC]
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "ADC: Vref=");
    Format_UnsDec(CONS_UART_Write, MCU_Vref, 4);
    Format_String(CONS_UART_Write,", Vtemp=");
    Format_UnsDec(CONS_UART_Write, MCU_Vtemp, 4);
    Format_String(CONS_UART_Write,", Vsupply=");
    Format_UnsDec(CONS_UART_Write, Vsupply, 4);
    Format_String(CONS_UART_Write,", Vbutton=");
    Format_UnsDec(CONS_UART_Write, Vbutton, 4);
    Format_String(CONS_UART_Write,", VCC=");
    Format_UnsDec(CONS_UART_Write, MCU_VCC, 4, 2);
    Format_String(CONS_UART_Write,", Temp=");
    Format_SignDec(CONS_UART_Write, MCU_Temp, 3, 1);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);

    GPS_Position *Position = GPS_getPosition();
    if( Position && Position->hasGPS && Position->isValid() )
    { TxPacket.Packet.Position.AcftType=Parameters.AcftType;
      Position->Encode(TxPacket.Packet);
#ifdef WITH_FLASHLOG
      bool Written=FlashLog_Process(TxPacket.Packet, Position->getUnixTime());
#endif
      TxPacket.Packet.Whiten();
      TxPacket.calcFEC();
      OGN_TxPacket<OGN_Packet> *TxPkt=RF_TxFIFO.getWrite();
      *TxPkt = TxPacket;
      RF_TxFIFO.Write(); }

    RF_SlotTime = TimeSync_Time();
    uint8_t TxChan = RF_FreqPlan.getChannel(RF_SlotTime, 0, 1);                // tranmsit channel
    TRX.setChannel(TxChan);

    TX_Credit+=2; if(TX_Credit>7200) TX_Credit=7200;                           // count the transmission credit
    XorShift32(RX_Random);
    uint32_t TxTime = (RX_Random&0x3F)+1; TxTime*=6; TxTime+=50;               // random transmission time: (1..64)*6+50 [ms]

    const uint8_t *TxPktData0=0;
    const uint8_t *TxPktData1=0;
    const OGN_TxPacket<OGN_Packet> *TxPkt0 = RF_TxFIFO.getRead(0);             // get 1st packet from TxFIFO
    const OGN_TxPacket<OGN_Packet> *TxPkt1 = RF_TxFIFO.getRead(1);             // get 2nd packet from TxFIFO
    if(TxPkt0) TxPktData0=TxPkt0->Byte();                                      // if 1st is not NULL then get its data
    if(TxPkt1) TxPktData1=TxPkt1->Byte();                                      // if 2nd if not NULL then get its data
          else TxPktData1=TxPktData0;                                          // but if NULL then take copy of the 1st packet

    TimeSlot(800-TimeSync_msTime(), TxPktData0, TxTime);                       // run a Time-Slot till 0.800sec

    TxChan = RF_FreqPlan.getChannel(RF_SlotTime, 1, 1);

    XorShift32(RX_Random);
    TxTime = (RX_Random&0x3F)+1; TxTime*=6;

    TimeSlot(1250-TimeSync_msTime(), TxPktData1, TxTime);

    if(TxPkt0) RF_TxFIFO.Read();
    if(TxPkt1) RF_TxFIFO.Read();
  }

}




