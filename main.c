/*
 *                                    PIC16(L)F1829   
 *                                  +-------_-------+
 *                           VDD -> : 1 VDD   VSS 20: <> VSS
 *              DEBUG1       RA5 <> : 2       PGD 19: <- RA0  (reserved for ICSP data)                (SPI SDO)
 *           BATT.LOW OUTPUT RA4 <> : 3 RST   PGC 18: <> RA1  (reserved for ICSP clock)               (SPI CLK)
 *SPI nCC/(reserv. for MCLR) RA3->  : 4 nMRCL INT 17: <> RA2  DOOR INPUT Contact: (Normally Open)     (SPI SDI)
 *                           RC5->  : 5 PWM       16: <> RC0  OUTPUT DOOR OPEN (NEW)
 *              DOOR_CLOSED  RC4->  : 6           15: <> RC1  OUTPUT BATTERY LOW 
 *                     {nSS] RC3->  : 7           14: <> RC2  OUTPUT DOOR CLOSED
 *                           RC6 <> : 8 CS    SDI 13: <> RB4
 *                           RC7 <> : 9 SDO       12: <> RB5  UART RX (TO VCOM, RX @Curiosity Board)
 *                  UART TX  RB7 <> : 10      CLK 11: <> RB6  
 *(TO VCOM, TX @Curiosity Board     +---------------+
 *                                        DIP-20
 *
 *
 *                                    PIC12(L)F1822
 *                                  +-------_-------+
 *                           VDD -> : 1 VDD   VSS 8 : <- VSS
 *                           RA5 <> : 2       PGD 7 : <> RA0  DOOR OPEN OUTPUT       (reserved for ICSP data)         
 *     BATTERY LOW OUTPUT    RA4 <> : 3       PGC 6 : <> RA1  DOOR CLOSED OUTPUT     (reserved for ICSP clock)
 *                           RA3->  : 4 *MRCL INT 5 : <> RA2  DOOR INPUT 
 *                                  +---------------+
 *                                        DIP-8
 *  
 *              The code in this file has been made with compiler flag to switch between two microcontrollers (20 pin PIC16(L)F1929 or the 8 pin PIC12(L)F1822)
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

#elif defined(_12LF1822) || defined(_12F1822) 
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

#include <xc.h>
#if defined(_16LF1829) // then include
#include "spi.h"
#include "Click_7Seg.h"
#include "usart.h"
#include <stdio.h>
#endif

// always include
#include "statemachine.h"
#include "GardenGate_v4_config.h"
#include "watchdog.h"

#if defined(_16LF1829)
void PWM_init(void);
void PWM_set(unsigned char duty_cycle);
#endif 

extern enum STATE_DOOR_STATES state_door; // defined in statemachine.c  and typedef in statemachine.h
extern enum STATE_MACHINE1_STATES state_machine;
extern enum STATE_OUTPUT_PULSE_STATES state_output_pulse;
extern enum STATE_SELFTEST_STATES state_selftest;

//volatile int sleep_64ms_counter;
volatile int sleep_128ms_counter;
volatile int sleep_256ms_counter;
volatile int sleep_1s_counter;
volatile int sleep_4s_counter;
volatile int sleep_256s_counter;

#if defined(_16LF1829)
char write2display_copy;
#endif
//----------------------------------------------------------

void main(void) {
    static __bit output_polarity;

#if defined(_16LF1829)   
    static __bit pwm_direction;
    static unsigned char pwm_dc = 0;
    static unsigned char display_char = 0;
    char write2display_copy = 0;

#endif
    // disable analog inputs for port A and C
    ANSELA = 0; // I/O pins selected. analog pin disabled
#if defined(_16LF1829) 
    ANSELC = 0; // Important setting to be able to use Port C for I/O
#endif
    // Oscillator setup  NOTE CLKOUT is Fosc/4 !!
    OSCCONbits.IRCF = 0b1111; // 16 MHz HF
    OSCCONbits.SPLLEN = 0; // disable 4xPLL
    OSCCONbits.SCS = 0b10; // set the SCS bits to select internal oscillator block  (HFINTOSC or LFINTOSC)

    // input or outpus setting for port pins 
#if defined(_16LF1829)
    TRISCbits.TRISC7 = 0; // output     SPI SDO
    TRISCbits.TRISC6 = 0; // output     #CS
    TRISCbits.TRISC5 = 0; // output     PWM     LED right most on Curiosity board
    TRISCbits.TRISC4 = 0; // output
    TRISCbits.TRISC3 = 0; // output

    TRISCbits.TRISC2 = 0; // output     OUTPUT DOOR CLOSED
    TRISCbits.TRISC1 = 0; // output
    TRISCbits.TRISC0 = 0; // output

    TRISAbits.TRISA5 = 0; // output
    TRISAbits.TRISA4 = 0; // output     #RESET
    TRISAbits.TRISA3 = 1; // input      #MCLR   reset input
    TRISAbits.TRISA2 = 1; // input      DOOR_INPUT
    TRISAbits.TRISA1 = 0; //
    TRISAbits.TRISA0 = 0; //

    TRISBbits.TRISB7 = 0; // OUTPUT     UART TX
    TRISBbits.TRISB6 = 0; // OUTPUT     SPI CLK
    TRISBbits.TRISB5 = 1; // INPUT      UART RX
    TRISBbits.TRISB4 = 1; // INPUT      SPI SDI

#elif defined(_12LF1822) || defined(_12F1822)

    TRISAbits.TRISA5 = 0; // RA5 = output     SPARE (pin2)
    TRISAbits.TRISA4 = 0; // RA4 = output     BATTERY LOW PULSE           (pin3)
    TRISAbits.TRISA3 = 1; // RA3 = input      DEBUG OUTPUT #MCLR          (pin4)
    TRISAbits.TRISA2 = 1; // RA2 = input      DOOR input - low when open  (pin5)
    TRISAbits.TRISA1 = 0; // RA1 = output     DOOR CLOSED PULSE           (pin6)
    TRISAbits.TRISA0 = 0; // RA0 = output     DOOR OPEN PULSE             (pin7)
#endif   
    // no analog pins nessesary as we are only measuring on the internal 1.024 voltage reference
    ANSELAbits.ANSA4 = 0; // Digital I/O  RA4 port
    ANSELAbits.ANSA2 = 0; // Digital I/O  RA2 port
    ANSELAbits.ANSA1 = 0; // Digital I/O  RA1 port
    ANSELAbits.ANSA0 = 0; // Digital I/O  RA0 port

#if defined(_16LF1829)     
    ANSELBbits.ANSB5 = 0; // Digital I/O  RB5 port
    ANSELBbits.ANSB4 = 0; // Digital I/O  RB4 port
#endif

    OPTION_REGbits.nWPUEN = 0; // weak pull-ups are enabled by individual WPUx latch values
    WPUAbits.WPUA3 = 1; // RA3 pull-up  nMCLR
    WPUAbits.WPUA2 = 0; // RA2 pull-up  DOOR_INPUT   //CHANGE

#if defined(_16LF1829)
    INLVLAbits.INLVLA3 = 1; // input nMCLR                  ST input used for port reads and interrupt-on-change
    INLVLAbits.INLVLA2 = 1; // input DOOR_INPUT             ST input used for port reads and interrupt-on-change
#endif

    INTCONbits.PEIE = 1; // Peripheral Interrupt Enable   Nesseary for using timers! 

    PIE1bits.TMR1GIE = 0; // Disables the Timer1 Gate Acquisition interrupt
    PIE1bits.SSP1IE = 0; // Disables the MSSP interrupt

    INTCONbits.GIE = 1; // INTCONbits.GPIE = 0;    // Disable GPIO Change Interrupt Enable bit (uses INT interrupt instead)

    MDCARHbits.MDCH = 0b0100; // carrier signal disable
    MDCONbits.MDEN = 0; // carrier signal disable
    MDSRCbits.MDMSODIS = 1; // Modulator signal source

    PIE1bits.TMR2IE = 1; // Enable Timer 2 interrupt for counter for pulse width    
    PIR1 = 0;
    PIR2 = 0;

#if defined(_16LF1829)
    PIR3 = 0;
    PIR4 = 0;
#endif    

    IOCAF = 0; // clear Interrupt-on-Change PORTA Flag bits
#if defined(_16LF1829)
    IOCBF = 0;
#endif  
    FVRCON = 0;

    MDSRCbits.MDMS = 0b0010;  // Modulator signal source
    MDCARHbits.MDCH = 0b0100; // Modulator signal source
    MDCARLbits.MDCL = 0b0100; // Modulator signal source

    SSP1CON1bits.SSPEN = 0;
    RCSTAbits.SPEN = 0;

    CM1CON0bits.C1ON = 0; // disable comperator 1 & 2
#if defined(_16LF1829)
    CM2CON0bits.C2ON = 0;
#endif
    SRCON0bits.SRLEN = 0; // SR-latch disable (page 149)

    RCSTAbits.SREN = 0; // 
    TXSTAbits.TXEN = 0;
    timer2_init();

#if defined(_16LF1829)
    PWM_init();                 // Init of PWM to dim display
    PWM_set(50);
    output_polarity = 0; // used to toogle port pins on every while round
    pwm_direction = 1; // set direction of PWM count
    Click_latch_data(); // Set Click 7Seg module latch signal correct
    Click_reset(); // reset Click 7Seg Module
    spi1_init();
    spi1_send(0x80, 0x80); // set a "-" in each 7Seg display
    PWMtoggle = 0;
    PWM_set(50);
    UART_init(); // setup UART
#endif      
    disable_watchdog_timer();

#if defined(_16LF1829)    
    int2bcd(11, 1); // will write 1.1 in 7-seg display  [number, comma[YES|NO]]
    printf("Welcome to KOCH Engineering GardenGate\r\n");
    printf("Microchip 16LF1829 starting up...\r\n");
#endif    
    __delay_ms(200); // wait to be able to see bars in the 7Seg display
    NewStateChangeDetected = 0;

    // initiate state machine
    state_door = DOOR_INPUT; // read door state
    state_machine = STATE_JUST_AWAKED;  
        
    DoorStateBeforePrel = DOOR_INPUT;
    DoorStateAfterPrel = DOOR_INPUT;

    INTCONbits.INTF = 0;    // clear external INT interrupt flag to be able to sleep
    INTCONbits.GIE = 1;     // enable global interrupt

    sleep_128ms_counter = 0;
    sleep_256ms_counter = 0;
    sleep_1s_counter = 0;
    sleep_4s_counter = 0;
    sleep_256s_counter = 0;
    set_watchdog_timer_128ms();
    
    // set correct INT edge polarity of interrupt edge
    switch (DOOR_INPUT) {  // check door input 
        case DOOR_OPEN: // LOW ~ pulled down when door is open
             state_door = STATE_DOOR_OPEN;
             OPTION_REGbits.INTEDG = !DOOR_OPEN; // interrupt on raising edge
             NewDoorPolarity = DOOR_OPEN;
             DoorStateBeforePrel = DOOR_OPEN;
             break;
        case DOOR_CLOSED: // HIGH - pulled high when door is closed
            state_door = STATE_DOOR_CLOSED;
            OPTION_REGbits.INTEDG = !DOOR_CLOSED; // interrupt on next falling edge                
            NewDoorPolarity = DOOR_CLOSED;
            DoorStateBeforePrel = DOOR_CLOSED;
            break;
    } // switch (DOOR_INPUT)
    
    enable_external_interrupt();
    
    while (1) {
        // check if INT interrupt in the INT interrupt service routine has set af flag 
        if (NewStateChangeDetected == TRUE) {
            state_machine = STATE_NEW_DOOR_STATE;
            NewStateChangeDetected = FALSE;
        }

#if defined(_16LF1829)     
        if (state_machine != write2display_copy) {
            if (state_machine != 12) {
                int2bcd(state_machine, 0);
                // __delay_ms(10);                       // by adding a delay here it helps seing what sequence of state the statemachine is in
                write2display_copy = state_machine; // update to prevent running update to 7Seg display if not different data to show
            }
        }
#endif
        checkState1(); // run statemachine 
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
        
    } // while forever
}

// function declarations

#if defined(_16LF1829) 
 void PWM_init(void) { // <editor-fold defaultstate="collapsed" desc="PWM_init function">

    // PWM output pin
    PSTR1CONbits.STR1A = 1; // !!!! HUSK DENNE FOR PWM out!!  PxA pin has the PWM waveform with polarity control from CCPxM<1:0>
    PSTR1CONbits.STR1B = 0; // PxB pin is assigned to port pin
    PSTR1CONbits.STR1C = 0; // PxC pin is assigned to port pin
    PSTR1CONbits.STR1D = 0; // PxC pin is assigned to port pin
    PSTR1CONbits.STR1SYNC = 1;

    SSP1CON1bits.SSPEN = 0; //     

    // PWM setup   
    CCP1CONbits.P1M = 0b00; // Configure the CCP1 module for the PWM mode        (page 213))   00: Single output; P1A modulated; P1B, P1C, P1D assigned as port pins
    CCP1CONbits.CCP1M = 0b1100; // 1100 = PWM mode: P1A, P1C active-high; P1B, P1D active-high

    CCPR1L = 220; // 4) Load the CCPR1L register and the DC1B1 bits of the CCP1CON register, with the PWM duty cycle value.
    CCP1CONbits.DC1B = 0b11; // PWM Duty Cycle Least Significant bits

    TRISCbits.TRISC5 = 1; // Make Port C5 as output pin    (enable CCP1 pin)
    PR2 = 249; // 2) Load PWM period       

    TRISCbits.TRISC5 = 0; // enable CCP1 pin

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
    } else {
        scaled_duty_cycle = 1024 / 100 * duty_cycle; // scale the percentage to 10 bits (1024))
        CCPR1L = (unsigned char) (scaled_duty_cycle >> 2); // now contains the upper MSB part of the 10 bits calculation
        CCP1CONbits.DC1B = (0x03 & scaled_duty_cycle); // now contaions the 2 LSB bits from the calculation
    }
    return;
} // PWM_set  </editor-fold>
#endif
 
void timer2_init(void) {
    // Timer2 setup  
    T2CONbits.T2OUTPS = 0b1111;
    T2CONbits.TMR2ON = 1;
    //while (PIR1bits.TMR2IF==0); // 6) Enable PWM output pin        
    T2CONbits.T2CKPS = 0b10; // 0b10   Pre-scale = 16   Timer2 input frequency  Fosc/4 divied by 16 =             (Page 201)
    T2CONbits.T2OUTPS = 0b111; // 0b1111 Timer 2 Postscaler = 16
    T2CONbits.TMR2ON = 1; // Timer 2 ON     
}
int getBatteryVoltage(void) { // <editor-fold defaultstate="collapsed" desc="getBatteryVoltage function">
    // https://edeca.net/pages/measuring-pic-vdd-with-no-external-components-using-the-fvr/
    // returns raw maasurement on 1.024V ref with full scale on Vdd.
    // this can be used to calculate back the VDD voltage when we know how much 1.024Volt takes on a 10 bit ADC 

    // HKOCH 27/6/-22
    // Some confusions here. In the method shown above the reference is the VDD!! 
    // This means that the sentence: "When the FVR is selected as the reference input, the FVR Buffer 1 output selection must be 2.048V or 4.096V (ADFVR<1:0> = 1x)"
    // THE REFERENCE OS NOT THE FVR but is instead the ADC input so it is not nessesary to usse the 2.048 here!!
   
    
    // set the ADC clock
    //ADCON1bits.ADCS = 0b110;    // ADC CLOCK PERIOD (TAD) setup:  Fosc/64  (slowest possible time to have the slowest sample time)
    // at Fosc = 16MHz this gives an TAD of 4us. Datasheet says 11,5 clocks cycles is needed. This gives a total conversion time of 4us x 11,5 = 184us 
    // Data sheet Table 16-1
        
    FVRCONbits.FVREN = 1; // Fixed Voltage Reference is enabled

#if defined(_16LF1829)    
    while (!FVRCONbits.FVRRDY); // Wait for FVR to be stable (bit is always 1 when using 12xF1823), FVRRDY is always ?1? on PIC12F1822/16F1823 only.
#else
    __delay_us(50); // as the 1822/23 only have this flag always '1' then we need a delay here
#endif
    
    ADCON1bits.ADPREF = 0b00; // VREF+ is connected to VDD   (works for both 16xL1829 and 12xF1822)
    __delay_us(50); // wait 50 usec to stabilize 
    
#if defined(_16LF1829)
    ADCON1bits.ADNREF = 0; // 0 = VREF- is connected to VSS   // set negative reference. Only possible on 16xF1829
#endif

    FVRCONbits.ADFVR = 0b01; // ADC Fixed Voltage Reference Peripheral output is 1x (1.024V)
    //FVRCONbits.ADFVR = 0b10; // ADC Fixed Voltage Reference Peripheral output is 2x (= 2.048V)

    __delay_us(50); // wait minimum 5 usec to stabilize 
    
    ADCON1bits.ADFM = 1; // Right justify result. Six Most Significant bits of ADRESH are set to ?0? when the conversion result is loaded.
    ADCON0bits.CHS = 0b11111; // FVR (Fixed Voltage Reference) Buffer 1 Output
    __delay_us(50); // wait minimum 5 usec to stabilize 

    ADCON0bits.ADON = 1; // Turn on ADC module    
    //__delay_us(200); // wait minimum 5 usec to stabilize 
    __delay_ms(10);     // sample time 10 ms before the sample is performed
    
    // now make measurement  ------------
    int adc_val = 0;

    ADCON0bits.GO = 1; // Start a ADC conversion
    while (!ADCON0bits.GO_nDONE); // Wait for it to be completed

    adc_val = (ADRESH << 8); // Store the result in adc_val
    adc_val |= ADRESL;

    FVRCONbits.FVREN = 0; // Fixed Voltage Reference is disabled to save power
    ADCON0bits.ADON = 0; // Turn off ADC module    
    return adc_val;
} // getBatteryVoltage  </editor-fold>
void watch_dog_incrementing_timer(void) { // <editor-fold defaultstate="collapsed" desc="watch_dog_incrementing_timer function">

    if (sleep_128ms_counter < 8) {
        // 8 x 128ms ~ 1024 ms.  ~ 1 sec.
        sleep_128ms_counter++;
        set_watchdog_timer_128ms(); }
    else if (sleep_256ms_counter < 20) {
        // 20 x 256 ~ 5 sec.
        sleep_256ms_counter++;
        set_watchdog_timer_256ms(); }
    else if (sleep_1s_counter < 10) {
        // 10 x 1 sec. ~ 10 sec.
        sleep_1s_counter++;
        set_watchdog_timer_1s(); }
    else if (sleep_4s_counter < 150) {
        // 150 x 4 sec. = 600 sec. = 10 minutes
        sleep_4s_counter++;
        set_watchdog_timer_4s(); }
    else if (sleep_256s_counter < 337) {
        sleep_256s_counter++;
        set_watchdog_timer_256s(); }
} // watch_dog_incrementing_timer  </editor-fold>
void enable_external_interrupt(void) {
    INTCONbits.INTE = 1; // Enable external INT interrupt
}
void disable_external_interrupt(void) {
    INTCONbits.INTE = 0; // Disable external INT interrupt
}
void int_disable_external_interrupt(void) {
    INTCONbits.INTE = 0; // Disable external INT interrupt
}
void __interrupt() my_isr(void) { // <editor-fold defaultstate="collapsed" desc="interrupt routine">

    // INT external input interrupt pin <editor-fold defaultstate="collapsed" desc="INT pin interrupt">
    if (INTE && INTF) {
        // Door STATE has changed
        switch (DOOR_INPUT) {
            case DOOR_OPEN: // LOW ~ pulled down when door is open
                state_door = STATE_DOOR_OPEN;
                OPTION_REGbits.INTEDG = !DOOR_OPEN; // interrupt on raising edge
                NewDoorPolarity = DOOR_OPEN;
                DoorStateBeforePrel = DOOR_OPEN;
                break;
            case DOOR_CLOSED: // HIGH - pulled high when door is closed
                state_door = STATE_DOOR_CLOSED;
                OPTION_REGbits.INTEDG = !DOOR_CLOSED; // interrupt on next falling edge                
                NewDoorPolarity = DOOR_CLOSED;
                DoorStateBeforePrel = DOOR_CLOSED;
                break;
        } // switch (DOOR_INPUT)
        NewStateChangeDetected = 1;
        //int_disable_external_interrupt();
        INTF = 0;                                     // clear INT interrupt flag
    } // INT interrrupt </editor-fold>

    // Timer 2 <editor-fold defaultstate="collapsed" desc="Timer2 interrupt">
    if (PIE1bits.TMR2IE && PIR1bits.TMR2IF) {
        if (prelCounter > 0)
            prelCounter--;

        if (ihcPulseTimer > 0)
            ihcPulseTimer--;
        
        PIR1bits.TMR2IF = 0; // clear timer 2 interrupt flag 

    } // Timer2 Interrupt </editor-fold>
    return;

} // </editor-fold>
