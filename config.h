#define WITH_OGN1                          // OGN protocol version 1/2
#define OGN_Packet OGN1_Packet

#define HARDWARE_ID 0x03
#define SOFTWARE_ID 0x01

#define DEFAULT_AcftType        1         // [0..15] default aircraft-type: glider
#define DEFAULT_GeoidSepar     40         // [m]
#define DEFAULT_CONbaud    115200         // [bps]
#define DEFAULT_PPSdelay       80         // [ms]
// #define WITH_XTAL26MHZ                    // when with 26MHz XTAL
// #define DEFAULT_FreqPlan        5         // then default plan is 433MHz
#define DEFAULT_FreqPlan        0         //
#define DEFAULT_Hard       "RS41"
#define DEFAULT_Soft       "0.0"

#define UART1_RxFIFO_Size 128             // GPS UART
#define UART1_TxFIFO_Size  32

#define UART3_RxFIFO_Size  64             // Console UART
#define UART3_TxFIFO_Size 256

#define WITH_CONFIG                       // can change parameters through the console

#define WITH_GPS_UBX                      // GPS is UBX-type
// #define WITH_GPS_UBX_PASS
#define WITH_GPS_CONFIG                   // try to setup the GPS baudrate and navigation mode

#define WITH_LED_TX

#define WITH_FLASHLOG

#define WITH_AUTOCR
