/*
 * File:   main.c
 * Author: t-bird
 *
 * Created on 24 06 2020 , 14:37
 */

// PIC12F1822 Configuration Bit Settings

// 'C' source line config statements

// PIC12F1822 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable (WDT controlled by the SWDTEN bit in the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = BOOT       // Flash Memory Self-Write Protection (000h to 1FFh write protected, 200h to 7FFh may be modified by EECON control)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#include "hardware.h"

typedef struct  {
    uint8_t header;
    uint8_t len;
    uint8_t sum;
    uint8_t code;
    uint16_t address;
    uint16_t data[WRITE_FLASH_BLOCKSIZE];
}   frame_t;

#define MAX_FRAME_LEN   (WRITE_FLASH_BLOCKSIZE*2+4)

frame_t frame;
uint8_t* frameAddress = (uint8_t*)(&frame);
uint8_t frameByteCounter;
uint8_t i;
uint8_t checksum;

void handleFrame();
void generateErrorFrame(uint8_t errorCode);
void sendFrame();
void displayError(int code);
void calcChecksum();

void main(void)
{
    initHardware();
    LED_ON;
    sleep262ms();//sleep(255);
    frameByteCounter = 0;
    if (LIN_LINE_STATE == 1)
        asm("goto 0x200");
    ENABLE_UART;

    while (1)
    {
        if (!linReadByte(&(frameAddress[frameByteCounter]), 255))
        {
            // PROTOCOL MANAGER MUST BE HERE
            if (frameByteCounter == 0)
            {
                if (frame.header != 0xE2)
                    displayError(2);
                else
                    frameByteCounter++;
            }
            else if (frameByteCounter == 1)
            {
                if (frame.len > MAX_FRAME_LEN)
                    displayError(4);
                else
                    frameByteCounter++;
            }
            else
            {
                if (frameByteCounter < (frame.len + 1))
                   frameByteCounter++;
                else
                    handleFrame();
            }
        }
        else
        {
            LED_LATCH = LED_LATCH ^ 1;
//            frameByteCounter = 0;
            if (frameByteCounter != 0)
                displayError(4);
        }
    }
}

#define READ_MEMORY_FRAME   0x10
#define WRITE_MEMORY_FRAME  0x20
#define RESET_MEMORY_FRAME  0x50

#define FRAME_WITH_READED_MEMORY    0x12
#define FRAME_WITH_WRITE_STATUS     0x22
#define FRAME_WITH_ERROR_CODE       0x32

void handleFrame()
{    
    frameByteCounter = 0;
    calcChecksum();
    if (checksum != frame.sum)
    {
//        generateErrorFrame(10);
//        sendFrame();
        displayError(6);
        return;
    }
//    if (((frame.code == READ_MEMORY_FRAME) && (frame.len != 3)) || ((frame.code == WRITE_MEMORY_FRAME) && (frame.len != MAX_FRAME_LEN)) || ((frame.code == RESET_MEMORY_FRAME) && (frame.len != 1)))
//    {
//        generateErrorFrame(1);
//        sendFrame();
//        return;
//    }    
    // FRAME WITH CODE
    switch (frame.code)
    {
        case READ_MEMORY_FRAME:
            for (int i = 0; i < WRITE_FLASH_BLOCKSIZE; i++)
                frame.data[i] = flashReadWord(frame.address + i);
            frame.code = FRAME_WITH_READED_MEMORY;
            frame.len = WRITE_FLASH_BLOCKSIZE * 2 + 4;
            //sendFrame();
            break;
        case WRITE_MEMORY_FRAME:
            if ((frame.address < 0x200) || (frame.address >= 0x800))
                return;
            flashErase(frame.address);
            flashWrite(frame.address, frame.data);
            frame.code = FRAME_WITH_WRITE_STATUS;
            frame.len = 2;
            //((uint8_t*)(frame.address))[0] = 0;
//            for (i = 0; i < WRITE_FLASH_BLOCKSIZE; i++)
//            {
//                uint16_t readData = flashReadWord(frame.address + i);
//                if (readData != frame.data[i])
//                {
//                    ((uint8_t*)(frame.address))[0] = 1;
//                    break;
//                }
//            }
            //sendFrame();
            break;
        case RESET_MEMORY_FRAME:
            asm("reset");
            break;
        default:
            //return;
            //generateErrorFrame(2);
            displayError(5);
            break;
    }
    sendFrame();
}

void generateErrorFrame(uint8_t errorCode)
{
    frame.code = FRAME_WITH_ERROR_CODE;
    ((uint8_t*)(frame.address))[0] = errorCode;
    frame.len = 2;
}

void sendFrame()
{
    calcChecksum();
    frame.sum = checksum;
    for (i = 0; i < (frame.len + 2); i++)
    {
        linWriteByte(frameAddress[i]);
//        if (!linWriteByte(frameAddress[i]))
//        {
//            //displayError(10);
//            //return;
////            while(1)
////            {
////                sleep(100);
////                LED_LATCH = LED_LATCH ^ 1;
////            }
//        }
    }
}

void calcChecksum()
{
    checksum = 0;
    for (i = 3; i < (frame.len + 2); i++)
        checksum = checksum + frameAddress[i];
}

void displayError(int code)
{
    for (i = 0; i < code; i++)
        sleep262ms();//sleep(255);
        //sleep262ms();
    frameByteCounter = 0;  
}

/**
 End of File
*/