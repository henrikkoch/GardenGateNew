/*
 *                                    PIC16(L)F1829   
 *                                  +-------_-------+
 *                           VDD -> : 1 VDD   VSS 20: <> VSS
 *              DEBUG1       RA5 <> : 2       PGD 19: <- RA0  (reserved for ICSP data)                (SPI SDO)
 *           BATT.LOW OUTPUT RA4 <> : 3 RST   PGC 18: <> RA1  (reserved for ICSP clock)               (SPI CLK)
 *SPI nCC/(reserv. for MCLR) RA3->  : 4 nMRCL INT 17: <> RA2  DOOR INPUT Contact: (Normally Open)     (SPI SDI)
 *                           RC5->  : 5 PWM       16: <> RC0  
 *              DOOR_CLOSED  RC4->  : 6           15: <> RC1  OUTPUT BATTERY LOW 
 *                     {nSS] RC3->  : 7           14: <> RC2  OUTPUT DOOR CLOSED
 *                           RC6 <> : 8 CS    SDI 13: <> RB4
 *                           RC7 <> : 9 SDO       12: <> RB5  OUTPUT DOOR OPEN  
 *                           RB7 <> : 10      CLK 11: <> RB6  
 *                                  +---------------+
 *                                        DIP-20
 *
 *
 *                                    PIC12(L)F1822  (@ Tiny iCP07A modified PCB to fit correct on PCB modified pins)
 *                                  +-------_-------+
 *                           VDD -> : 1 VDD   VSS 8 : <- VSS
 *                           RA5 <> : 2       PGD 7 : <> RA0     DOOR OPEN OUTPUT       (reserved for ICSP data)         
 *     BATTERY LOW OUTPUT    RA4 <> : 3       PGC 6 : <> RA1     DOOR CLOSED OUTPUT     (reserved for ICSP clock)
 *                           RA3->  : 4 *MRCL INT 5 : <> RA2     DOOR INPUT 
 *                                  +---------------+
 *                                        DIP-8
 *  
 *              The code in this file has been made with compiler flag to se how big the microcontroller is (20 pin PIC16(L)F1929 or the 8 pin PIC12(L)F1822
 *              The controller type needs to be setup in the project|Properties page
 */

#if defined(_16LF1829) 
// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is enabled on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

#elif defined(_12LF1822)

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is enabled on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

#endif




// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#if defined(_16LF1829) 
    #include "spi.h"
    #include "Click_7Seg.h"
#endif

#include "statemachine.h"
#include "GardenGate_v4_config.h"

#if defined(_16LF1829) 
void PWM_init(void);
void PWM_set(unsigned char duty_cycle);
#endif 

extern enum STATE_DOOR_STATES          state_door;          // defined in statemachine.c  and typedef in statemachine.h
extern enum STATE_MACHINE1_STATES      state_machine;
extern enum STATE_OUTPUT_PULSE_STATES  state_output_pulse;
extern enum STATE_SELFTEST_STATES      state_selftest;

volatile int sleep_64ms_counter;
volatile int sleep_128ms_counter;
volatile int sleep_256ms_counter;
volatile int sleep_1s_counter;
volatile int sleep_4s_counter;
volatile int sleep_256s_counter;

#if defined(_16LF1829) 
    char write2display_copy;
#endif
//----------------------------------------------------------
void main (void)
{
    static __bit output_polarity;   

#if defined(_16LF1829)     
    static __bit pwm_direction;
    static unsigned char pwm_dc = 0;
    static unsigned char display_char = 0;
    char write2display_copy = 0;
#endif    
    
    // disable analog inputs for port A and C
    ANSELA = 0;                 // I/O pins selected. analog pin disabled
#if defined(_16LF1829) 
    ANSELC = 0;                 // Important setting to be able to use Port C for I/O
#endif
    
    // Oscillator setup  NOTE CLKOUT is Fosc/4 !!
    OSCCONbits.IRCF = 0b1111;   // 16 MHz HF
    OSCCONbits.SPLLEN = 0;      // disable 4xPLL
    OSCCONbits.SCS = 0b10;      // set the SCS bits to select internal oscillator block  (HFINTOSC or LFINTOSC)
    
    // input or outpus setting for port pins 
#if defined(_16LF1829)     
    TRISCbits.TRISC7 = 0;   // output     SPI SDO   
    TRISCbits.TRISC6 = 0;   // output     #CS
    TRISCbits.TRISC5 = 0;   // output     PWM     LED right most on Curiosity board
    TRISCbits.TRISC4 = 0;   // output     
    TRISCbits.TRISC3 = 0;   // output     
    
    TRISCbits.TRISC2 = 0;   // output     OUTPUT DOOR CLOSED
    TRISCbits.TRISC1 = 0;   // output     
    TRISCbits.TRISC0 = 0;   // output     
      
    TRISAbits.TRISA5 = 0;   // output    
    TRISAbits.TRISA4 = 0;   // output     #RESET
    TRISAbits.TRISA3 = 1;   // input      #MCLR   reset input    
    TRISAbits.TRISA2 = 1;   // input      DOOR_INPUT
    TRISAbits.TRISA1 = 0;   // 
    TRISAbits.TRISA0 = 0;   // 
    
    TRISBbits.TRISB7 = 0;   // 
    TRISBbits.TRISB6 = 0;   // OUTPUT     SPI CLK
    TRISBbits.TRISB5 = 0;   // OUTPUT     OUTPUT DOOR OPEN  
    TRISBbits.TRISB4 = 1;   // INPUT      SPI SDI   
#elif defined(_12LF1822)
    TRISAbits.TRISA5 = 0;   // RA5 = output     DEBUG OUTPUT                (pin2)
    TRISAbits.TRISA4 = 0;   // RA4 = output     BATTERY LOW PULSE           (pin3)
    TRISAbits.TRISA3 = 1;   // RA3 = input      #MCLR                       (pin4)
    TRISAbits.TRISA2 = 1;   // RA2 = input      DOOR input - low when open  (pin5)  
    TRISAbits.TRISA1 = 0;   // RA1 = output     DOOR CLOSED PULSE           (pin6)    
    TRISAbits.TRISA0 = 0;   // RA0 = output     DOOR OPEN PULSE             (pin7)
#endif   
    // no analog pins nessesary as we are only measuring on the internal 1.024 voltage reference
    ANSELAbits.ANSA4 = 0;   // Digital I/O  RA4 port
    ANSELAbits.ANSA2 = 0;   // Digital I/O  RA2 port
    ANSELAbits.ANSA1 = 0;   // Digital I/O  RA1 port
    ANSELAbits.ANSA0 = 0;   // Digital I/O  RA0 port
    
#if defined(_16LF1829)     
    ANSELBbits.ANSB5 = 0;   //
    ANSELBbits.ANSB4 = 0;   //
#endif
    
    OPTION_REGbits.nWPUEN = 0;  // weak pull-ups are enabled by individual WPUx latch values
    
    WPUAbits.WPUA3 = 1;     // RA3 pull-up  nMCLR
    WPUAbits.WPUA2 = 0;     // RA2 pull-up  DOOR_INPUT   //CHANGE
        
#if defined(_16LF1829)     
    INLVLAbits.INLVLA3 = 1;     // input nMCLR                  ST input used for port reads and interrupt-on-change    
    INLVLAbits.INLVLA2 = 1;     // input DOOR_INPUT             ST input used for port reads and interrupt-on-change
#endif
    
    INTCONbits.PEIE = 1;    // Peripheral Interrupt Enable   Nesseary for using timers! 
    
    PIE1bits.TMR1GIE = 0;   // Disables the Timer1 Gate Acquisition interrupt
    PIE1bits.SSP1IE = 0;    // Disables the MSSP interrupt
       
    INTCONbits.GIE = 1;         // INTCONbits.GPIE = 0;    // Disable GPIO Change Interrupt Enable bit (uses INT interrupt instead)
    
    MDCARHbits.MDCH = 0b0100;   // carrier signal disable
    MDCONbits.MDEN = 0;         // carrier signal disable
    MDSRCbits.MDMSODIS = 1;     // Modulator signal source

    PIE1bits.TMR2IE = 1;        // Enable Timer 2 interrupt for counter for pulse width    
        
    PIR1 = 0;
    PIR2 = 0;
    
#if defined(_16LF1829)     
    PIR3 = 0;
    PIR4 = 0;
#endif    
    
    IOCAF = 0;                  // clear Interrupt-on-Change PORTA Flag bits
#if defined(_16LF1829) 
    IOCBF = 0;
#endif  
    FVRCON = 0;
      
    MDSRCbits.MDMS = 0b0010;    // Modulator signal source
    MDCARHbits.MDCH = 0b0100;   // Modulator signal source
    MDCARLbits.MDCL = 0b0100;   // Modulator signal source
   
    SSP1CON1bits.SSPEN = 0;
    RCSTAbits.SPEN = 0;
    
    CM1CON0bits.C1ON = 0;       // disable comperator 1 & 2
#if defined(_16LF1829)     
    CM2CON0bits.C2ON = 0;
#endif    
    SRCON0bits.SRLEN = 0;       // SR-latch disable (page 149)

    RCSTAbits.SREN = 0;         // 
    TXSTAbits.TXEN = 0;         

    timer2_init();
#if defined(_16LF1829)     
    PWM_init();                 // Init of PWM to dim display
    PWM_set(50);
    output_polarity = 0;        // used to toogle port pins on every while round
    pwm_direction = 1;          // set direction of PWM count
    Click_latch_data();         // Set Click 7Seg module latch signal correct
    Click_reset();              // reset Click 7Seg Module
    spi1_init();                
    spi1_send(0x80,0x80);       // set a "-" in each 7Seg display
    PWMtoggle = 0;
    PWM_set(50);
#endif      
    
   disable_watchdog_timer();
   
#if defined(_16LF1829)    
    int2bcd(11,1);              // will write 1.1 in 7-seg display
#endif    
     __delay_ms(200);           // wait to be able to see bars in the 7Seg display
    
    NewStateChangeDetected = 0;
    
     // initiate state machine
    state_door = DOOR_INPUT;    // read door state
    state_machine = STATE_JUST_AWAKED;
    
    // check battery voltage at power up by measuring a fixed 1.024volt reference and back calculation the VDD from that 
    BatteryVoltage = getBatteryVoltage();
    millivolts = (8192 / BatteryVoltage) * 1024;
    millivolts = millivolts /8;    
    if (BatteryVoltage <= VOLTAGE_LOW) {  // this if voltage measured is lower than 2.4 volt
        // turn on IHC pulse to indicate low voltage
        OUTPUT_BATTERY_LOW = PULSE_ON;  // turn on battery voltage signal
        __delay_ms(IHC_PULSE_WIDTH*8);     // Timer2 takes 8ms therefore 5 * 8ms = ~40 ms
        OUTPUT_BATTERY_LOW = !PULSE_ON;  // turn on battery voltage signal
    }
        
    DoorStateBeforePrel = DOOR_INPUT;
    DoorStateAfterPrel = DOOR_INPUT;
    
    INTCONbits.INTF = 0;    // clear INT interrupt flag to be able to sleep
    INTCONbits.GIE = 1;     // enable global interrupt
    
    enable_external_interrupt();
        
    //sleep_64ms_counter = 0;
    sleep_128ms_counter = 0;
    sleep_256ms_counter = 0;
    sleep_1s_counter = 0;
    sleep_4s_counter = 0;
    sleep_256s_counter = 0;
    set_watchdog_timer_128ms();
        
    while (1) {
        // check if INT interrupt in INT interrupt service routine has set af flag 
        if (NewStateChangeDetected == TRUE) {
            state_machine = STATE_NEW_DOOR_STATE;   
            NewStateChangeDetected = FALSE;
        }
        
    #if defined(_16LF1829)     
        if (state_machine != write2display_copy) {
            if (state_machine != 12) {
                //int2bcd(state_machine);                 // show current state number in Click 7Seg display
               // __delay_ms(10);                       // by adding a delay here it helps seing what sequence of state the statemachine is in
                write2display_copy = state_machine;     // update to prevent running update to 7Seg display if not different data to show
            }
        }
    #endif           
        
        checkState1();                                  // run statemachine 
        
        watch_dog_incrementing_timer();                 // make sleep periods bigger and bigger 
        
        // <editor-fold defaultstate="collapsed" desc="PWM test routine">
/*          
        // SET PWM in counting up and down patterns 
        
        if (pwm_direction == 1) {
            if (pwm_dc < 99) {
                pwm_dc++;
                // send 10's and 1' of the pwm_dc value
                spi1_send(C7SEG_CHAR_TABLE[(pwm_dc/10)+3],C7SEG_CHAR_TABLE[(pwm_dc%10)+3]);
                
            }
            else {
                    pwm_direction = 0;
                    __delay_ms(300);   
                }
        }
        else {
            if (pwm_dc > 0) {
                pwm_dc--;
                // send 10's and 1' of the pwm_dc value
                spi1_send(C7SEG_CHAR_TABLE[(pwm_dc/10)+3],C7SEG_CHAR_TABLE[(pwm_dc%10)+3]);
            }
            else {
                    pwm_direction = 1;
                    __delay_ms(300);   
                }
        }
            
        PWM_set(pwm_dc);
        
        __delay_ms(10); 
*/   
 
        // </editor-fold>
    }  // while forever
}  
#if defined(_16LF1829) 

  void PWM_init(void) { // <editor-fold defaultstate="collapsed" desc="PWM_init function">

    // PWM output pin
    PSTR1CONbits.STR1A = 1;     // !!!! HUSK DENNE FOR PWM out!!  PxA pin has the PWM waveform with polarity control from CCPxM<1:0>
    PSTR1CONbits.STR1B = 0;     // PxB pin is assigned to port pin
    PSTR1CONbits.STR1C = 0;     // PxC pin is assigned to port pin
    PSTR1CONbits.STR1D = 0;     // PxC pin is assigned to port pin
    PSTR1CONbits.STR1SYNC = 1;  
    
    SSP1CON1bits.SSPEN = 0;     //     
    
    // PWM setup   
    CCP1CONbits.P1M = 0b00;     // Configure the CCP1 module for the PWM mode        (page 213))   00: Single output; P1A modulated; P1B, P1C, P1D assigned as port pins
    CCP1CONbits.CCP1M = 0b1100; // 1100 = PWM mode: P1A, P1C active-high; P1B, P1D active-high

    CCPR1L = 220;               // 4) Load the CCPR1L register and the DC1B1 bits of the CCP1CON register, with the PWM duty cycle value.
    CCP1CONbits.DC1B = 0b11;    // PWM Duty Cycle Least Significant bits
      
    TRISCbits.TRISC5 = 1;       // Make Port C5 as output pin    (enable CCP1 pin)
    PR2 = 249;                  // 2) Load PWM period       

    TRISCbits.TRISC5 = 0;       // enable CCP1 pin
    
    // Fosc = 4 MHZ (in this setup with INTOSC HighSpeed without 4xPLL)
    // pre scaler = 16
    // Periodetid = 4 MHz / 16 / (249+1) = = 1000ms  ~ PWM frequency = 1 kHz 

} // PWM_init </editor-fold>
  void PWM_set(unsigned char duty_cycle) { // <editor-fold defaultstate="collapsed" desc="PWM_set function">
    // input Duty Cycle in % [0-100]]
    unsigned int scaled_duty_cycle;  
    // set the pulse width using the given percentage in the PW_value
     
    // Load the CCPRxL register and the DCxBx bits of the CCPxCON register, with the PWM duty cycle value.
    // [8 bit CCPR1L ] + [2 bit CCP1CONbits.DC1B ]  
    
    // oonvert the received duty_cycle from 0-100  til 10-bit value
    // 10 bit = 2^10 = 1024  so nearly multiply by 10
        
    if (duty_cycle == 0) {
        CCPR1L = 0;             
        CCP1CONbits.DC1B = 0x03;
    } 
    else if (duty_cycle > 100) {
        // set PW_value to 100
        duty_cycle = 100;
        CCPR1L = 0xFF;
        CCP1CONbits.DC1B = 0x03;
    }
    else {
        scaled_duty_cycle = 1024/100*duty_cycle;            // scale the percentage to 10 bits (1024))
        CCPR1L = (unsigned char) (scaled_duty_cycle >> 2);  // now contains the upper MSB part of the 10 bits calculation
        CCP1CONbits.DC1B = (0x03 & scaled_duty_cycle);      // now contaions the 2 LSB bits from the calculation
    }    
    return;
}  // PWM_set  </editor-fold>
#endif

  
void timer2_init(void) {
    // Timer2 setup
    T2CONbits.T2OUTPS = 0b1111;
    T2CONbits.TMR2ON = 1;
    //while (PIR1bits.TMR2IF==0); // 6) Enable PWM output pin        
    T2CONbits.T2CKPS = 0b10;    // 0b10   Pre-scale = 16   Timer2 input frequency  Fosc/4 divied by 16 =             (Page 201)
    T2CONbits.T2OUTPS = 0b111;  // 0b1111 Timer 2 Postscaler = 16
    T2CONbits.TMR2ON = 1;       // Timer 2 ON     
}
  
int getBatteryVoltage(void) { // <editor-fold defaultstate="collapsed" desc="getBatteryVoltage function">
    // https://edeca.net/pages/measuring-pic-vdd-with-no-external-components-using-the-fvr/
    // returns raw maasurement on 1.024V ref with full scale on Vdd.
    // this can be used to back calculate the VDD voltage when we know how much 1.024Volt takes on a 10 bit ADC
    
#if defined(_16LF1829)     
    ADCON1bits.ADNREF = 0;      // 0 = VREF- is connected to VSS
#endif
    
    FVRCONbits.ADFVR = 0b01;    // ADC Fixed Voltage Reference Peripheral output is 1x (1.024V)       
        
    FVRCONbits.FVREN = 1;       // Fixed Voltage Reference is enabled
    while(!FVRCONbits.FVRRDY);  // Wait for FVR to be stable

    ADCON1bits.ADFM = 1;        // Right justify result
    ADCON0bits.CHS = 0b11111;   // FVR (Fixed Voltage Reference) Buffer 1 Output
    __delay_us(200);            // wait minimum 200 usec
    
    ADCON1bits.ADPREF = 0b00;   // VREF+ is connected to VDD
            
    ADCON0bits.ADON = 1;        // Turn on ADC module    
    
    // now make measurement  ------------
    int adc_val = 0;            

    ADCON0bits.GO = 1;          // Start a ADC conversion
    while (!ADCON0bits.GO_nDONE);   // Wait for it to be completed

    adc_val = (ADRESH << 8);    // Store the result in adc_val
    adc_val |= ADRESL;
        
    FVRCONbits.FVREN = 0;       // Fixed Voltage Reference is disabled to save power
    ADCON0bits.ADON = 0;        // Turn off ADC module    
    return adc_val;
} // getBatteryVoltage  </editor-fold>

void watch_dog_incrementing_timer(void) {
      
    if (sleep_128ms_counter < 8) {
        // 8 x 128ms ~ 1024 ms.
        sleep_128ms_counter++;
        set_watchdog_timer_128ms();
    }
    else if (sleep_256ms_counter < 40) {
        // 40 x 256 ~ 10 sec.
        sleep_256ms_counter++;
        set_watchdog_timer_256ms();
    }
    else if (sleep_1s_counter < 60) {
        // 60 x 1 sec. ~ 60 sec = 1 minute
        sleep_1s_counter++;
        set_watchdog_timer_1s();
    }
    else if (sleep_4s_counter < 75) {
        // 75 x 4 sec. = 5 minutes
        sleep_4s_counter++;
        set_watchdog_timer_4s();
    }
    else {
        sleep_256s_counter++;
        set_watchdog_timer_256s();
    }
}
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
}    
void set_watchdog_timer_4s(void) {
    WDTCONbits.WDTPS = 0b01100; // 01100 = 1:131072 (2^17) (Interval 4s nominal)
}
void set_watchdog_timer_256s(void) {
    WDTCONbits.WDTPS = 0b10010; // 10010 = 1:8388608 (2^23) (Interval 256s nominal)  watch-dog timer    
  
}
void enable_watchdog_timer(void) {
    WDTCONbits.SWDTEN = 1;      // enable SW controlled watch-dog    
}
void disable_watchdog_timer(void) {
    WDTCONbits.SWDTEN = 0;      // disable SW controlled watch-dog    
}
void enable_external_interrupt(void) {
    INTCONbits.INTE = 1;                     // Enable external INT interrupt
}
void disable_external_interrupt(void) {
    INTCONbits.INTE = 0;                     // Disable external INT interrupt
}
void __interrupt() my_isr(void) {  // <editor-fold defaultstate="open" desc="interrupt routine">

    // INT external input interrupt pin <editor-fold defaultstate="collapsed" desc="INT pin interrupt">
    if (INTE && INTF) {
        // Door STATE has changed
        //WDTCONbits.SWDTEN = 0;  // disable watchdog sleep until check on door has been done
        switch (DOOR_INPUT) {
            case DOOR_OPEN: // LOW ~ pulled down when door is open
                state_door = STATE_DOOR_OPEN;
                OPTION_REGbits.INTEDG = !DOOR_OPEN;         // interrupt on raising edge
                NewDoorPolarity = DOOR_OPEN;
                DoorStateBeforePrel = DOOR_OPEN;
                break;
                
            case DOOR_CLOSED: // HIGH - pulled high when door is closed
                state_door = STATE_DOOR_CLOSED;
                OPTION_REGbits.INTEDG = !DOOR_CLOSED;       // interrupt on next falling edge                
                NewDoorPolarity = DOOR_CLOSED;
                DoorStateBeforePrel = DOOR_CLOSED;    
                break;
        } 
        NewStateChangeDetected = 1;
        INTF = 0;                                           // clear INT interrupt flag
    } // INT interrrupt </editor-fold>
        
    // Timer 2 <editor-fold defaultstate="collapsed" desc="Timer2 interrupt">
    if (PIE1bits.TMR2IE && PIR1bits.TMR2IF) {
        if (prelCounter > 0) 
            prelCounter--; 
        
        if (ihcPulseTimer > 0) 
            ihcPulseTimer--;
        PIR1bits.TMR2IF = 0;  // clear timer 2 interrupt flag 
        //DEBUG_OUTPUT = 1;   // only used during test of Timer2 was running
        //DEBUG_OUTPUT = 0;
   }  // Timer2 Interrupt </editor-fold>
    
   return;
      
} // </editor-fold>
        
        
        
        
        
     

    
    
    
    
    