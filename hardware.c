#include <pic12f1822.h>
#include <stdint.h>
#include "hardware.h"

//uint8 res; 

void initHardware()
{
    OSCCON = 0x58;
    APFCON = 0x84;
    TRISA = 0x24;
    ANSELA = 0x04;
    PORTA = 0x00;
    LATA = 0x00;
    WPUA = 0x00;
    SPBRGH = 0;
    SPBRGL = 12;
    ADCON0 = 0x09;
    ADCON1 = 0x00;
    BAUDCON = 0x08;
    TXSTA = 0x24;
    // UART RECEIVER DISABLED
    RCSTA = 0x00;
    OPTION_REG = 0x87;
    INTCON = 0;
    WDTCON = 0;
}

uint8_t adcRead()
{
    ADCON0bits.ADGO = 1;
    while(ADCON0bits.ADGO == 1)
    {}
    return ADRESH;
}

void linWriteSync()
{
    RCSTAbits.CREN = 0;
    TXSTAbits.SENDB = 1;    
    while (TXSTAbits.TRMT == 0)
    {}
    TXREG = 0x55;
    TXREG = 0x55;
    while (TXSTAbits.SENDB == 1)
    {}
    while (TXSTAbits.TRMT == 0)
    {}    
    RCSTAbits.CREN = 1;
}

uint8_t linWriteByte(uint8_t byte)
{
//    if (RCSTAbits.OERR)
//    {
//        RCSTA = 0;
//        asm("nop");
//        RCSTA = 0x90;
//    }
    TXREG = byte;
//    TMR0 = 0xFE;
//    INTCONbits.TMR0IF = 0;
//    while((PIR1bits.RCIF == 0) && (INTCONbits.TMR0IF == 0))
//    {}
//    if (PIR1bits.RCIF == 0)
//        return 1;
//    PIR1bits.RCIF = 0;
//    if (RCREG != byte)
//        return 2;
//    return 0;
    uint8_t readedByte;
    return (linReadByte(readedByte, 2));
}

uint8_t linReadByte(uint8_t* byte, uint8_t timeout)
{
    TMR0 = 255 - timeout;
    INTCONbits.TMR0IF = 0;
    if (RCSTAbits.OERR)
    {
        RCSTA = 0;
        asm("nop");
        RCSTA = 0x90;
    }
    while((PIR1bits.RCIF == 0) && (INTCONbits.TMR0IF == 0))
    {}
    if (PIR1bits.RCIF == 0)
        return 1;
    PIR1bits.RCIF = 0;
    byte[0] = RCREG;
    return 0;
}

void flashErase(uint16_t address)
// ERASING 16 WORDS
{
    EEADRL = (address & 0xFF);
    // Load upper 6 bits of erase address boundary
    EEADRH = ((address & 0xFF00) >> 8);

    // Block erase sequence
    EECON1bits.CFGS = 0;    // Deselect Configuration space
    EECON1bits.EEPGD = 1;   // Select Program Memory
    EECON1bits.FREE = 1;    // Specify an erase operation
    EECON1bits.WREN = 1;    // Allows erase cycles

    // Start of required sequence to initiate erase
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;      // Set WR bit to begin erase
        
    asm("nop");
    asm("nop");
    
    EECON1bits.WREN = 0;
}

// SIZE OF DATA - 32 bytes or 16 words
void flashWrite(uint16_t address, uint16_t* data)
{
    uint8_t i;
    // Block write sequence
    EECON1bits.EEPGD = 1;   // Select Program Memory
    EECON1bits.CFGS = 0;    // Deselect Configuration space
    EECON1bits.WREN = 1;    // Enable writes
    EECON1bits.LWLO = 1;    // Only load write latches
    
    // Load lower 8 bits of write address
    EEADRL = (address & 0xFF);
    // Load upper 6 bits of write address
    EEADRH = ((address & 0xFF00) >> 8);

    for (i = 0; i < WRITE_FLASH_BLOCKSIZE; i++)
    {
//        // Load lower 8 bits of write address
//        EEADRL = (address & 0xFF);
//        // Load upper 6 bits of write address
//        EEADRH = ((address & 0xFF00) >> 8);

        // Load data in current address
        EEDATL = data[i];
        EEDATH = ((data[i] & 0xFF00) >> 8);

        if(i == (WRITE_FLASH_BLOCKSIZE-1))
        {
            // Start Flash program memory write
            EECON1bits.LWLO = 0;
        }

        EECON2 = 0x55;
        EECON2 = 0xAA;
        EECON1bits.WR = 1;
        asm("nop");
        asm("nop");        
        
        EEADRL++;
    }
    
    EECON1bits.WREN = 0;
}

uint16_t flashReadWord(uint16_t address)
{
    EEADRL = (address & 0x00FF);
    EEADRH = ((address & 0xFF00) >> 8);

    EECON1bits.CFGS = 0;    // Deselect Configuration space
    EECON1bits.EEPGD = 1;   // Select Program Memory
    EECON1bits.RD = 1;      // Initiate Read
    asm("nop");
    asm("nop");

    return ((uint16_t)((EEDATH << 8) | EEDATL));
}

void sleep(uint8_t timeMs)
{
    TMR0 = 255 - timeMs;
    INTCONbits.TMR0IF = 0;
    while(INTCONbits.TMR0IF == 0)
    {}
}

void sleep262ms()
{
    TMR0 = 0;
    INTCONbits.TMR0IF = 0;
    while(INTCONbits.TMR0IF == 0)
    {}
}