#define WITH_OGN1                          // OGN protocol version 1/2
#define OGN_Packet OGN1_Packet

#define HARDWARE_ID 0x01
#define SOFTWARE_ID 0x01

#define DEFAULT_AcftType        1          // [0..15] default aircraft-type: glider
#define DEFAULT_GeoidSepar     40          // [m]
#define DEFAULT_CONbaud    115200
#define DEFAULT_PPSdelay       80

#define UART1_RxFIFO_Size 128             // GPS UART
#define UART1_TxFIFO_Size  32

#define UART3_RxFIFO_Size  64             // Console UART
#define UART3_TxFIFO_Size 256

#define WITH_CONFIG                       // can change parameters through the console

#define WITH_GPS_UBX                      // GPS is UBX-type
// #define WITH_GPS_UBX_PASS
#define WITH_GPS_CONFIG                   // try to setup the GPS baudrate and navigation mode

#define WITH_TX_LED

#define WITH_FLASHLOG

#define WITH_AUTOCR