/*
 * File:   watchdog.c
 * Author: Henrik J. Koch - 2021
 *
 * Created on 16. june 2021, 21:31
 * 
 * Used as a function library for different funcktions on the Microchip Couriosity Board rev. 4
 * 
 */

#include <xc.h>

#if defined(_16LF1829) 
    #include <stdio.h>  // printf
#endif

void set_watchdog_timer_64ms(void) {
    WDTCONbits.WDTPS = 0b00110; // 00110 = 1:2048 (Interval 64 ms nominal)
} 
void set_watchdog_timer_128ms(void) {
    WDTCONbits.WDTPS = 0b00111; // 00111 = 1:4096 (Interval 128 ms nominal)
} 
void set_watchdog_timer_256ms(void) {
    WDTCONbits.WDTPS = 0b01000; // 01000 = 1:8192 (Interval 256 ms nominal)
} 
void set_watchdog_timer_1s(void) {
    WDTCONbits.WDTPS = 0b01010; // 01010 = 1:32768 (Interval 1s nominal)
#if defined(_16LF1829)  
    printf("set_watchdog_timer_1s\r\n");
#endif    
}    
void set_watchdog_timer_4s(void) {
    WDTCONbits.WDTPS = 0b01100; // 01100 = 1:131072 (2^17) (Interval 4s nominal)
#if defined(_16LF1829)  
    printf("set_watchdog_timer_4s\r\n");
#endif
    
}
void set_watchdog_timer_256s(void) {
    WDTCONbits.WDTPS = 0b10010; // 10010 = 1:8388608 (2^23) (Interval 256s nominal)  watch-dog timer
#if defined(_16LF1829)  
    printf("set_watchdog_timer_256s\r\n");
#endif  
}
void enable_watchdog_timer(void) {
    WDTCONbits.SWDTEN = 1;      // enable SW controlled watch-dog    
}
void disable_watchdog_timer(void) {
    WDTCONbits.SWDTEN = 0;      // disable SW controlled watch-dog    
}

