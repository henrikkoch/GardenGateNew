/*
 * File:   spi.c
 * Author: henrik
 *
 * Created on 27. marts 2021, 22:44
 */

#if defined(_16LF1829) 
    #include <xc.h>

//#include <pic16lf1823.h>

    #include <pic16lf1829.h>
    //#include "spi.h"
    #include "Click_7Seg.h"         // using Click_latch_data function

/* Pin connections from PIC to 74HC595
 * RCLK (RC1)   -> RCLK     (Pin 12)
 * SDO1 (RC2)   -> SER      (Pin 14)
 * SCLK1 (RC3)  -> SRCLK    (Pin 11)
*/
    #define RCLK LATCbits.LATC1

#define _XTAL_FREQ 16000000

void spi1_init(void){
    
   //TRISCbits.TRISC1 = 0x00; //Sets RC1 as an output
   //TRISCbits.TRISC2 = 0x00; //Sets RC2 as an output
   //TRISCbits.TRISC3 = 0x00; //Sets RC3 as an output
        
    // REGISTER 25-1: SSP1STAT: SSP1 STATUS REGISTER
    SSP1STATbits.SMP = 1; // SMP: SPI Data Input Sample bit SPI Master mode: 1 = Input data sampled at end of data output time
    SSP1STATbits.CKE = 1; // CKE: SPI Clock Edge Select bit (SPI mode only) 1=Transmit occurs on transition from active to Idle clock state
    
    // REGISTER 25-2: SSP1CON1: SSP1 CONTROL REGISTER 1 
    SSP1CON1bits.SSPM = 0x01;   // SPI Master mode, clock = FOSC/16
    SSP1CON1bits.CKP = 0;       // CKP: Clock Polarity Select bit, Idle state for clock is a low level

    // MSSP1 STATUS register (SSP1STAT)
    // MSSP1 Control Register 1 (SSP1CON1)
    // MSSP1 Control Register 3 (SSP1CON3)
    // MSSP1 Data Buffer register (SSP1BUF)
    // MSSP1 Address register (SSP1ADD)
    // MSSP1 Shift register (SSP1SR)
        
    //APFCON0bits
            
    // For 20 Pin Devices (PIC16(L)F1829):
    // Bit is read-only, ?0?
    // SDO function is always on RC7.            
        
    //RC2PPS = 0x10; //Sets RC2 as the SDO pin
    //RC3PPS = 0x0f; //RC3 is SCLK by default     
    
    SSP1CON1 = 0b00000001; //Sets SCLK to FOSC/16
    
    PIR1bits.TMR2IF = 0;        // 5) Clear the TMR2IF interrupt flag bit of the PIR1 register.
    
    PWM1CONbits.P1RSEN = 1;
    SSP1CON1bits.SSPEN = 1; // SSP1EN: Synchronous Serial Port Enable bit
       
    //RCLK = 0;
}

void spi1_send (char data1, char data2){
   
    
    //LATCbits.LATC4 = 1; 
    //LATCbits.LATC4 = 0; // RESET            
       
    
    // first digit left most (tens of the number)
    SSP1BUF = data2; //Data is sent out on SPI
    //while(!SSP1STATbits.BF); //Waits for transmission to complete 
    
    RCLK = 1;         //Data is latched to output upon rising edge 
    __delay_us(10);   //of RCLK
    RCLK = 0; 
    
    // now second digit right most (ones of the number)  NOTE this in fact pushes the first number to the left due to the two daisy chained shift registers
    SSP1BUF = data1; //Data is sent out on SPI
    while(!SSP1STATbits.BF); //Waits for transmission to complete. Check if the recieve is complete (SSPBUS full) 
    SSP1BUF; //Resets BF flag
    
    RCLK = 1;         //Data is latched to output upon rising edge 
    __delay_us(10);   //of RCLK
    RCLK = 0; 
    
    // __delay_us(50);

    Click_latch_data();     // latch register values on 74HC595 to output storage register)
}

#endif