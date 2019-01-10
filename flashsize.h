#ifndef __FLASHSIZE_H__
#define __FLASHSIZE_H__

#include <stdint.h>

#ifdef WITH_STM32
uint32_t * const   FlashStart = (uint32_t *)0x08000000;                           // where the Flash memory starts
#define            FlashSize   ((uint16_t *)0x1FFFF7E0)                           // [KB] Flash memory size
uint16_t inline getFlashSizeKB(void) { return *FlashSize; }                       // [KB]
uint16_t inline getFlashPageSizeKB(void) { return 1+(getFlashSizeKB()>=256); }    // [KB]
uint8_t  inline getFlashPageSizeLog2(void) { return 10+(getFlashSizeKB()>=256); } // [log2([B])]
#endif // WITH_STM32

#ifdef WITH_SAMD21  // with Arduino support
uint32_t * const   FlashStart = (uint32_t *)0x00000000;                           // where the Flash memory starts
uint32_t inline getFlashPages(void) { return NVMCTRL->PARAM.bit.NVMP; }
uint16_t inline getFlashPageSize(void) { return (uint16_t)(NVMCTRL->PARAM.bit.PSZ)<<3; }
uint16_t inline getFlashSizeKB(void) { return (uint32_t)getFlashPages()*(getFlashPageSize()>>3)>>7; } // [KB]
#endif // WITH_SAMD21

#endif // __FLASHSIZE_H__
