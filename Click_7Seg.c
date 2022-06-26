/*
 * File:   Click_7Seg.c
 * Author: Henrik J. Koch - 2021
 *
 * Created on 29. marts 2021, 12:22
 * 
 * Used as a function library for different funcktions on the Microchip Couriosity Board rev. 4
 * 
 */

#include <xc.h>
#include "spi.h"
#include "Click_7Seg.h"

#if defined(_16LF1829) 



void int2bcd(int number, char comma) {
    // display a number between 0-99 in the two 7-segment diplays on Click module "7SegClick"
    // if comma == 1 then the first number has a comma turned on
    
    if (number < 100)
        if (comma == 1)
            spi1_send((C7SEG_CHAR_TABLE[(number/10)+3] | C7SEG_CHAR_TABLE[1]) ,C7SEG_CHAR_TABLE[(number%10)+3]);
        else 
            spi1_send(C7SEG_CHAR_TABLE[(number/10)+3],C7SEG_CHAR_TABLE[(number%10)+3]);
    else 
        spi1_send(0xF4, 0x64);  // "Er" for Overload error 
}

void Click_reset(void) {
    
    //  Resets the Click module slot on the PCB
    //
    LATAbits.LATA4 = 0;     // NOT RESET         
    LATAbits.LATA4 = 1;     // NOT RESET 
}

void Click_latch_data(void) {
    
    //  Latch the shiftregisters on the Click 7Seg module to the output buffers 
    //  by sending a positive going pulse to the Latch pins on the 74HC545 registers
    LATCbits.LATC6 = 1;     // CS / (latch)   
    LATCbits.LATC6 = 0;     // CS / (latch)
}

const unsigned char C7SEG_CHAR_TABLE[ 51 ]= {
    // lookup table to convert numbers into 7-segment display
    
    0x80, // '-'
    0x01, // '.'   Seven segment bit order
    0x00, // '/'   (g)(f)(e)(d)(c)(a)(b)(dp)
    0x7E, // '0'
    0x0A, // '1'    _a_
    0xB6, // '2'  f|   |b
    0x9E, // '3'   |_g_|
    0xCA, // '4'  e|   |c
    0xDC, // '5'   |_d_|.dp
    0xFC, // '6'
    0x0E, // '7'
    0xFE, // '8'
    0xDE, // '9'
    0x00, // ':'
    0x00, // ';'
    0x00, // '<'
    0x00, // '='
    0x00, // '>'
    0x00, // '?'
    0x00, // '@'
    0xEE, // 'A'
    0xF8, // 'B'
    0x74, // 'C'
    0xBA, // 'D'
    0xF4, // 'E'
    0xE4, // 'F'
    0x7C, // 'G'
    0xEA, // 'H'
    0x0A, // 'I'
    0x3A, // 'J'
    0xE0, // 'K'
    0x70, // 'L'
    0x2C, // 'M'
    0xA8, // 'N'
    0x7E, // 'O'
    0xE6, // 'P'
    0xCE, // 'Q'
    0x64, // 'R'
    0xDC, // 'S'
    0xF0, // 'T'
    0x7A, // 'U'
    0x7B, // 'V'
    0xFB, // 'W'
    0xEA, // 'X'
    0xDA, // 'Y'
    0xB6, // 'Z'
    0x00, // '['
    0x00, // '/'
    0x00, // ']'
    0x00, // '^'
    0x10  // '_' 
};

#endif