/* 
 * File:   hardware.h
 * Author: t-bird
 *
 * Created on August 29, 2020, 8:12 PM
 */

#ifndef HARDWARE_H
#define	HARDWARE_H

#include "pic12f1822.h"
#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif
        
#define WRITE_FLASH_BLOCKSIZE    16

    void initHardware();
    uint8_t adcRead();
    void linWriteSync();
    uint8_t linWriteByte(uint8_t byte);
    uint8_t linReadByte(uint8_t* byte, uint8_t timeout);
    void flashErase(uint16_t address);
    void flashWrite(uint16_t address, uint16_t* data);
    uint16_t flashReadWord(uint16_t address);
    void sleep(uint8_t timeMs);
    void sleep262ms();

#define ENABLE_UART     (RCSTA = 0x90)
#define LED_ON          (LATAbits.LATA0 = 1)
#define LED_OFF         (LATAbits.LATA0 = 0)
#define LIN_LINE_STATE  (PORTAbits.RA5)
#define LED_LATCH       (LATAbits.LATA0)

#ifdef	__cplusplus
}
#endif

#endif	/* HARDWARE_H */

