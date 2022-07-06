/*
 * File:   statemachine.c
 * Author: Henrik J. Koch
 *
 * Created on 16. june 2021, 12:56
 */

#include <xc.h>
#include "statemachine.h"
#include "GardenGate_v4_config.h"
#include "watchdog.h"
//#include <math.h>
//#include <stdlib.h>
        
#if defined(_16LF1829)
    #include "spi.h"
    #include "Click_7Seg.h"
    #include <stdio.h>
#endif

// Timing counters
volatile int prelCounter;               // timer for prel time
volatile int ihcPulseTimer;             // timer for IHC output pulse
volatile int WakeUpCounter;             // timer how  many times it needs to wake up before a voltage measurement is done
// voltages
volatile int BatteryVoltage;            // keeps til battery voltage measured
volatile unsigned int millivolts;
volatile int SevenSegVoltage;
//volatile float vdd_voltage;

int digit[4]; // used to signal voltage in milivotls per digit
int digit_no;
int digit_pulses;   


char DoorStateBeforePrel;     // check before and after prel timer
char DoorStateAfterPrel;      // -||-

__bit NewStateChangeDetected;  // flag to show new door interrupt has been seen
__bit NewDoorPolarity;         // 
#if defined(_16LF1829)      //__bit PWMtoggle;
__bit PWMtoggle;
#endif

enum STATE_DOOR_STATES          state_door;             // statemachine variables 
enum STATE_MACHINE1_STATES      state_machine;          // 
enum STATE_OUTPUT_PULSE_STATES  state_output_pulse;
enum STATE_SELFTEST_STATES      state_selftest;

void checkState1(void) {    // <editor-fold defaultstate="collapsed" desc="checkState1 statemachine">
    switch(state_machine) { 
        case STATE_JUST_AWAKED: // "00"
            disable_external_interrupt();
            disable_watchdog_timer();
            state_door = STATE_DOOR_UNKNOWN;    // start with unknown door state to make sure to check door state on first boot
        #if defined(_16LF1829)
            printf("STATE_JUST_AWAKED\r\n");
        #endif

        state_machine = STATE_MEASURE_BATTERY;    
//      state_machine = STATE_CHECK_DOOR;   // check what has happened due to the µP has woken up
            break;
            
            
        case STATE_NEW_DOOR_STATE: // "01"      // if interrupt routine detects an interrupt on the INT pin
            // state_door has the last state from the interrupt routine
            // DoorStateBeforePrel has the last read from the interrupt
            sleep_128ms_counter = 0;
            sleep_256ms_counter = 0;
            sleep_1s_counter = 0;
            sleep_4s_counter = 0;
            set_watchdog_timer_128ms();            
          
            prelCounter = PREL_DELAY;    // start prel timer (will be handled in Timer2 interrupt routine)
            state_machine = STATE_DOOR_CHANGED_WAIT_PREL;
        #if defined(_16LF1829)
            printf("STATE_NEW_DOOR_STATE\r\n");
        #endif
            break;
        case STATE_CHECK_DOOR: // "02"
            // check if door state is still the same as last time
            // ends up here after a wake-up - where we need to check if door state has changed during sleep
          #if defined(_16LF1829)
            printf("STATE_CHECK_DOOR: ");
          #endif
            
            if (state_door == DOOR_INPUT) {
                // check if door state is the same as before sent to sleep
                // if yes everything is find ---> go back to sleep
                state_machine = STATE_GO_SLEEP;
            #if defined(_16LF1829)
                printf("no new door state detected!\r\n");
            #endif
            }
            else {
                // if here, then door status has been changed since last check or it has the initial STATE_DOOR_UNKNOWN
                // Start wait to check again after prel-delay
                state_door = DOOR_INPUT; // read new door state
                DoorStateBeforePrel = state_door;
                prelCounter = PREL_DELAY;   // preset precounter to 1 x 16mS
                state_machine = STATE_DOOR_CHANGED_WAIT_PREL;
            #if defined(_16LF1829)
                printf("new door state detected. Going to wait for prel..\r\n");
            #endif
            }
            break;
        case STATE_DOOR_CHANGED_WAIT_PREL: // "04"
            if (prelCounter > 0) {
                state_machine = STATE_DOOR_CHANGED_WAIT_PREL;  // continue here until timer has been counted down in Timer2 interrupt routine
            }
            else {
                state_machine = STATE_DOOR_CHECK_AFTER_PREL;
            }
            #if defined(_16LF1829)
                printf("STATE_DOOR_CHANGED_WAIT_PREL\r\n");
            #endif
            break;
        case STATE_DOOR_CHECK_AFTER_PREL:  // "05"  
            // state after prel timer. Now check door if door is still the same door state as before prel check
            DoorStateAfterPrel = DOOR_INPUT;            // reads port and not output latch

            #if defined(_16LF1829)
                printf("STATE_DOOR_CHECK_AFTER_PREL\r\n");
            #endif
            
            // DoorStateBeforePrel was stored in the INT interrupt routine
            if (DoorStateAfterPrel == DoorStateBeforePrel) {
                
                //this means door is still same state after the prel timer delay. 
                state_door = DoorStateAfterPrel;  // read new door state in for future use
                
                // activate the right pulse output to IHC
                switch (state_door) {
                    case STATE_DOOR_OPEN:
                        OUTPUT_DOOR_OPEN = PULSE_ON;
                        #if defined(_16LF1829)   
                            printf("STATE_DOOR_OPEN\r\n");
                        #endif                           
                        break;
                    case STATE_DOOR_CLOSED:
                        OUTPUT_DOOR_CLOSED = PULSE_ON;
                        #if defined(_16LF1829)   
                            printf("STATE_DOOR_CLOSED\r\n");
                        #endif                          
                        break;
                    case STATE_DOOR_UNKNOWN:
                        // do nothing
                        break;
                    default:
                        //if (state_output_pulse==STATE_OUTPUT_PULSE_BATTERY_LOW) 
                        //  OUTPUT_BATTERY_LOW = PULSE_ON;
                        break;
                }
                ihcPulseTimer = IHC_PULSE_WIDTH;    // = 40ms. (Measured) That means we have 8ms. per ihcPulseTimer value  (5*8ms = 40ms)
                state_machine = STATE_SEND_IHC_PULSE;
            }
            else {
                // we have a mismatch before and after prel delay. Disqualify the pulse. Read new status and go sleep
 
                // no pulse have qualified for a correct door signal --> go back to sleep
                
                //sleep_64ms_counter = 0; // start again with short sleep intervals
                sleep_128ms_counter = 0; // start again with short sleep intervals
                sleep_256ms_counter = 0;
                sleep_1s_counter = 0;
                sleep_4s_counter = 0;
                //sleep_256s_counter = 0;
                
                set_watchdog_timer_1s();
                state_machine = STATE_GO_SLEEP;
            }
            break;
        case STATE_SEND_IHC_PULSE: // "06" 
            if (ihcPulseTimer > 0) 
                state_machine = STATE_SEND_IHC_PULSE; // continue until ihcPulseTimer counts to zero in timer interrupt routine
            else
                state_machine = STATE_STOP_SENDING_IHC_PULSE; // now IHC pulse goes in-active
          
            #if defined(_16LF1829)
                //  printf("STATE_SEND_IHC_PULSE\r\n");       // will repeat several times for each timer 2 interrupt to give the IHC pulse width
            #endif
            
            break;
        case STATE_STOP_SENDING_IHC_PULSE: // "07"
            // de-activate all IHC pulse outputs
            OUTPUT_DOOR_OPEN   = !PULSE_ON;     // means it goes off
            OUTPUT_DOOR_CLOSED = !PULSE_ON;
            OUTPUT_BATTERY_LOW = !PULSE_ON;
            // pulse have now been sent - go back to sleep
            
            //sleep_64ms_counter = 0; // start again with short sleep intervals
            sleep_128ms_counter = 0;
            sleep_256ms_counter = 0;
            sleep_1s_counter = 0;
            sleep_4s_counter = 0;
            //sleep_256s_counter = 0;   // dont't clear this as this is used for battery measurement
            
            set_watchdog_timer_128ms();
            state_machine = STATE_GO_SLEEP;
            
        #if defined(_16LF1829)
            printf("STATE_STOP_SENDING_IHC_PULSE\r\n");
        #endif
            
            break;
        case STATE_GO_SLEEP: // "09"

            // prepare to be put to sleep
            
            #if defined(_16LF1829)
                PWMtoggle = !PWMtoggle;
                // debug  to be able to see the sleep intervals
                if (PWMtoggle==1) { LATAbits.LATA5 = 1; }
                else              { LATAbits.LATA5 = 0; }
            #endif
            
            PSTR1CONbits.STR1A = 0; 
            // Important to be able to see 7Seg after sleep
            //PSTR1CONbits.STR1A = 1; 
            //CCP1CONbits.CCP1M = 0b0000;

            enable_external_interrupt();    
            enable_watchdog_timer();
            
            #if defined(_16LF1829)
                printf("go sleep...sleeping\r\n\n");
            #endif
            
                // ----------------- SLEEEEP ---------------------------------
            SLEEP();                // go sleep until WDT times out or INT input wakes it up
            NOP();                  // wakes from watch-dog timer on this line (the next line after a sleep)
            // ------------------ WAKE UP --------------------------------
            
            disable_watchdog_timer();
            
            #if defined(_16LF1829)
                printf("...now awake due to watchdog timer\r\n");
            #endif

            // check if we came here after af time out 
            if ((__timeout == 0) && (__powerdown == 1)) {   // nTO: if 0, a WDT time-out occurred
                // we are here as planned after a wake up
            }
            else {
                // if here and timeout flag is not "0", something must be wrong. Therefore a reset of the microcontroller to begin from start
               // RESET(); 
            }

            if (DEBUG == 1) {
                SPARE_OUTPUT = 1;      // turn on debug pin  (LED3 in Target HW)
                __delay_us(400);                    
                SPARE_OUTPUT = 0;
            }
            
            watch_dog_incrementing_timer();   // make sleep periods bigger and bigger
            
            // Important to be able to see 7Seg after sleep
            //CCP1CONbits.CCP1M = 0b1100;     // 1100 = PWM mode: PxA, PxC active-high; PxB, PxD active-high
            
            // !!!! The next line is important her. If not here the setting will be forgotten after a power down/sleep. And then the PWM is not running and therefore 
            // nothing will be visible in the 7Seg Click module
            PSTR1CONbits.STR1A = 1;     // !!!! HUSK DENNE FOR PWM out!!  PxA pin has the PWM waveform with polarity control from CCPxM<1:0>
                        
            if (sleep_256s_counter >= 294) {    // 337 times per day of 256 seconds equals 24 hours
                state_machine = STATE_MEASURE_BATTERY;
                sleep_256s_counter = 0;   // reset counter to make another voltage measurement in another 337 x 256 sec.
            }
            else {
                state_machine = STATE_CHECK_DOOR; }
            break;
        case STATE_DOOR_CHANGED:    // "03"
            #if defined(_16LF1829)
                printf("STATE_DOOR_CHANGED\r\n");
            #endif
            break;
        case  STATE_MEASURE_BATTERY: // "08"
            #if defined(_16LF1829)
                printf("STATE_MEASURE_BATTERY\r\n");
            #endif

            #if defined(_16LF1829)
                printf("measuring battery voltage\r\n");
            #endif
            
            BatteryVoltage = getBatteryVoltage();   // get raw DAC value
            // BatteryVoltage is the raw voltage measurement of the 2.048 voltage ref. measurement. Now we calculate back what the VDD is in millivolts 
            
            #if defined(_16LF1829)
                printf("BatteryVoltage (DAC value): %dd (%d out of 1024 DAC steps measured on 2.048V ref. (battery powered ADC))\r\n", BatteryVoltage, BatteryVoltage);
            #endif
                
            // do not have space enough for the floating point calculation
            //float vdd_voltage = (float)(1024.0 / BatteryVoltage) * 1.024f;     (1024 comes from 10 bit ~2^10 )
                
            //printf("float vdd_voltage measured: %8.6f\r\n", vdd_voltage);
                        
            millivolts = (unsigned int) (8192 / BatteryVoltage) * 1024;    //   
            millivolts /= 8;                                // divide by 8 as the number is already 8 times (8182/1024) bigger

            #if defined(_16LF1829)
                printf("voltage measured: %d mV\r\n", millivolts);
            #endif            
               
            // Show voltage in Click 7Seg display  
            #if defined(_16LF1829) // only write to 7Seg display if we are using the 16LF1829 microcontroller 
                // to show the voltage in the 7-Seg display 3300 milivolt is divided by 100 --> which gives 33 and the comman in the display is enable to show 3.3 (volt)
                SevenSegVoltage = millivolts /100;
                PWM_set(10);
                int2bcd(SevenSegVoltage, 1); // volt with comma
                __delay_ms(2000);        // wait 2 secs. to be able to see bars in the 7Seg display
            #endif
            
            if (millivolts <= VOLTAGE_LOW) {
                ihcPulseTimer = IHC_PULSE_WIDTH;                  
                OUTPUT_BATTERY_LOW = PULSE_ON;  // turn on battery voltage signal
                #if defined(_16LF1829)
                    printf("voltage is LOW (below threshold). Sending pulse\r\n");
                #endif     
                state_machine = STATE_SEND_IHC_PULSE;
            } 
            else {
                OUTPUT_BATTERY_LOW = PULSE_OFF;  // turn on battery voltage signal   (really not necessary to turn off here)                
                #if defined(_16LF1829)
                    printf("voltage is HIGH (above threshold). No pulse being sent\r\n", millivolts);
                #endif
                state_machine = STATE_CHECK_DOOR;
            }
            sleep_256s_counter = 0;   // reset counter to make another voltage measurement in another 337 x 256 sec.
            break;
        case STATE_SLEEPING:        // "10"
            #if defined(_16LF1829)
                printf("STATE_SLEEPING\r\n");
            #endif
            break;
            
        case STATE_INITIAL_TEST:    //
            // makes several tests during power up
            
            // check if battery is ok to proceed with. If not then signal that to the debug LED connected to pin SPARE_OUTPUT (LATAbits.LATA5)
            BatteryVoltage = getBatteryVoltage();   // get raw DAC value
           
            // calculate voltage in millivolt by using the internal 1.024 FVR volgtage
            millivolts = (unsigned int)(8192 / BatteryVoltage) * 1024;    //   
            millivolts /= 8;                                // divide by 8 as the number is already 8 times (8182/1024) bigger
                      
            // pulse voltage in millivolts
            // if we have 5V - that is 5000 millivolts  (zero is signaled as 10 pulses)
            // then we should pulse 5,10,10.10 with a small delay between pauses and pulses
            
            // first find the first number to signal 
            digit[3] = millivolts / 1000 % 10;      // get right most digit
            digit[2] = millivolts / 100 % 10;                                 
            digit[1] = millivolts / 10 % 10;
            digit[0] = millivolts % 10;             // get left most digit
                                    
            // now blink with the led for each digit
            
            //#define LED_BLINK_PULSE         80    // 80 ms
            //#define LED_BLINK_PAUSE         120   // 120 ms
            //#define LED_BLINK_DIGIT_DELAY   400   // 400 ms
                        
            for (digit_no=3; digit_no >= 0; digit_no--) {
                // loop through each digit
                
                for (digit_pulses=0; digit_pulses < digit[digit_no]; digit_pulses++) {
                    SPARE_OUTPUT = 1;
                    __delay_ms(LED_BLINK_PULSE);
                    SPARE_OUTPUT = 0;                        
                    __delay_ms(LED_BLINK_PAUSE);
                }
                __delay_ms(LED_BLINK_DIGIT_DELAY);
            }
                        
            
            // set state to have normal start up
            state_machine = STATE_JUST_AWAKED;  
            
            break;
        default:
            #if defined(_16LF1829)
                printf("STATE_default\r\n");
            #endif
            break;
    } // switch
}       // </editor-fold>


