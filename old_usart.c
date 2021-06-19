/*
 * File:   usart.c
 * Author: henrik
 *
 * Created on 31. maj 2021, 21:50
 */

#if defined(_16LF1829) 
    #include <xc.h>
    #include <stdio.h>
    #include "GardenGate_v4_config.h"

#define Baud_rate 115200


void UART_init(void) {
    
    //needs to use pin10 as Tx and pin12 as Rx
    // Pin function is selectable via the APFCON0 or APFCON1 registe (pin 123)
    
    APFCON0bits.RXDTSEL = 0;  // For 20 Pin Devices (PIC16(L)F1829): 0 = RX/DT function is on RB5 (pin 12)
    APFCON0bits.TXCKSEL = 0;  // For 20 Pin Devices (PIC16(L)F1829): 0 = TX/CK function is on RB7 (pin 10)
    
    // 26.0 ENHANCED UNIVERSAL SYNCHRONOUS ASYNCHRONOUS RECEIVER TRANSMITTER (EUSART) page 295

    // Initialize SPBRG register for required 
    // baud rate and set BRGH for fast baud_rate
    //SPBRG = ((_XTAL_FREQ/16)/Baud_rate) - 1;
    SPBRG = 1;
    
    BRGH  = 1;  // for high baud_rate
    BRG16 = 1;  // 16-bit Baud Rate Generator bit: 0 = 8-bit Baud Rate Generator is used


    TXEN = 1;   // enables the transmitter circuitry of the EUSART.    
    SYNC = 0;   // Asynchronous mode
    SPEN = 1;  
   
}

void putch(char data)
{
    while(!TXIF);
    TXREG = data;
}



void UART_write(unsigned char c)
{
     while(!TXSTAbits.TRMT)
     {
         TXREG = c;
     }
}

void UART_write_text(unsigned char data[])
 {
     // Takes a string and transmits each character one by one.
     // s++ increments the pointer address until the end of the string.
    int i = 0; 
    while(data[i] != '\0')
    {
        UART_write(data[i]);
        i++;
    }
    //__delay_mns(5);
 }





#endif
