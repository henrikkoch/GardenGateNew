/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   GardenGate_v4_config.h
 * Author: H.Koch 
 * Comments: contains common definitions for GardenGate V4 program
 * Revision history: 1.0
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_GardenGate_v4_config_H
#define	XC_GardenGate_v4_config_H

#include <xc.h> // include processor files - each processor file is guarded.  

#if defined(_16LF1829) // 20-pin PDIP
    #define DOOR_INPUT              PORTAbits.RA2   // PIN11 RA2 interrupt input
    #define OUTPUT_DOOR_OPEN        LATCbits.LATC0  // PIN16 RC0   PIN 16
    #define OUTPUT_DOOR_CLOSED      LATCbits.LATC2  // PIN14 RC2   PIN 14
    #define OUTPUT_BATTERY_LOW      LATCbits.LATC1  // PIN15 RC1
    #define LED4                    LATCbits.LATC5  // Rightmost LED on curiocity board
    // UART RX pin 12  (UART receive)  needs external wiring on curioucity board from VCOM TX pin to pin 10
    // UART TX pin 10  (UART transmit) needs external wiring on curioucity board from VCOM RX pin to pin 12
#elif defined(_12LF1822)  // 8-pin PDIP
    #define DEBUG_OUTPUT            LATAbits.LATA5  // (pin2)
    #define OUTPUT_BATTERY_LOW      LATAbits.LATA4  // (pin3)
    #define DOOR_INPUT              PORTAbits.RA2   // (pin5) external INT interrupt input
    #define OUTPUT_DOOR_CLOSED      LATAbits.LATA1  // (pin6)(ICSPCLK) 
    #define OUTPUT_DOOR_OPEN        LATAbits.LATA0  // (pin7)(ICSPCLK) 
    // #MCLR pin 4, VSS pin 8, VDD pin 1 
#endif

#define DOOR_OPEN   1   //CHANGE  due to using HALL sensor this is opposite than using Reed-relay
#define DOOR_CLOSED 0

//#define DOOR_OPEN   0 // when using NO Reed-relay
//#define DOOR_CLOSED 1

#define TRUE 1
#define FALSE 0

#define _XTAL_FREQ 16000000          // 16 MHz

//extern volatile int sleep_64ms_counter;
extern volatile int sleep_128ms_counter;
extern volatile int sleep_256ms_counter;
extern volatile int sleep_1s_counter;
extern volatile int sleep_4s_counter;
extern volatile int sleep_256s_counter;
extern volatile int sleep_256s_counter;

extern void timer2_init(void);
extern int getBatteryVoltage(void);         // return RAW ADC measurement on a 1.024V reference

#if defined(_16LF1829) 
extern void PWM_init(void);
extern void PWM_set(unsigned char duty_cycle);
#endif

//extern void set_watchdog_timer_64ms(void);
extern void set_watchdog_timer_128ms(void);
extern void set_watchdog_timer_256ms(void);
extern void set_watchdog_timer_1s(void); 
extern void set_watchdog_timer_4s(void);
extern void set_watchdog_timer_256s(void);
extern void enable_watchdog_timer(void);
extern void disable_watchdog_timer(void);
extern void enable_external_interrupt(void);
extern void disable_external_interrupt(void);
extern void watch_dog_incrementing_timer(void);

// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_GardenGate_V4_config_H */

