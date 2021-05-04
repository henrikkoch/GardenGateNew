/*
 * File:   statemachine.c
 * Author: henrik
 *
 * Created on 11. april 2021, 23:10
 */

#include <xc.h>
#include "statemachine.h"
#include "GardenGate_v4_config.h"
        
#if defined(_16LF1829)    
    #include "spi.h"
    #include "Click_7Seg.h"
#endif

// Timing counters
volatile int prelCounter;               // timer for prel time
volatile int ihcPulseTimer;             // timer for IHC output pulse
volatile int WakeUpCounter;             // timer how  many times it needs to wake up before a voltage measurement is done
// voltages
volatile int BatteryVoltage;            // keeps til battery voltage measured
volatile int millivolts;

char DoorStateBeforePrel;     // check before and after prel timer
char DoorStateAfterPrel;      // -||-

__bit NewStateChangeDetected;  // flag to show new door interrupt has been seen
__bit NewDoorPolarity;         // 

        
#if defined(_16LF1829)    
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
            state_machine = STATE_CHECK_DOOR;   // check what has happened due to the µP has woken up
            break;
     
        case STATE_NEW_DOOR_STATE: // "01"      // if interrupt routine detects an interrupt on the INT pin
            // state_door has the last state from the interrupt routine
            // DoorStateBeforePrel has the last read from the interrupt
           
            sleep_128ms_counter = 0;    // start again with short sleep intervals
            sleep_256ms_counter = 0;
            sleep_1s_counter = 0;
            sleep_4s_counter = 0;
//            sleep_256s_counter = 0;
          
            prelCounter = PREL_DELAY;    // start prel timer
            state_machine = STATE_DOOR_CHANGED_WAIT_PREL;
            break;
                  
        case STATE_CHECK_DOOR: // "02"
            // check if door state is still the same as last time
            // ends up here after a wake-up - where we need to check if door state has changed during sleep

            if (state_door == DOOR_INPUT) {
                // check if door state is the same as before sent to sleep
                // if yes everything is find ---> go back to sleep
                state_machine = STATE_GO_SLEEP;
            }
            else {
                // IF here, then door status has been changed since last check. 
                // Start wait to check again after prel-delay
                state_door = DOOR_INPUT; // read new door state
                DoorStateBeforePrel = state_door;
                prelCounter = PREL_DELAY;   // preset precounter to 1 x 16mS
                state_machine = STATE_DOOR_CHANGED_WAIT_PREL;
            }    
            break;
            
        case STATE_DOOR_CHANGED_WAIT_PREL: // "04"
            if (prelCounter > 0) {
                state_machine = STATE_DOOR_CHANGED_WAIT_PREL;  // continue here until timer has been counted down in intr routine
            }
            else {
                state_machine = STATE_DOOR_CHECK_AFTER_PREL;
            }
            break;
            
            
        case STATE_DOOR_CHECK_AFTER_PREL:  // "05"  
            // state after prel timer. Now check door if door is still the same door state as before prel check
            DoorStateAfterPrel = DOOR_INPUT;            // reads port and not output latch 

            // DoorStateBeforePrel was stored in the INT interrupt routine
            if (DoorStateAfterPrel == DoorStateBeforePrel) {
                
                //this means door is still same state after the prel timer delay. 
                state_door = DoorStateAfterPrel;  // read new door state in for future use
                
                // activate the right pulse output to IHC
                switch (state_door) {
                    case STATE_DOOR_OPEN:
                        OUTPUT_DOOR_OPEN = PULSE_ON;
                        break;
                    case STATE_DOOR_CLOSED:
                        OUTPUT_DOOR_CLOSED = PULSE_ON;
                        break;
                    case STATE_DOOR_UNKNOWN:
                    default:
                        if (state_output_pulse==STATE_OUTPUT_PULSE_BATTERY_LOW) 
                          OUTPUT_BATTERY_LOW = PULSE_ON;
                        break;
                }
                ihcPulseTimer = IHC_PULSE_WIDTH;    // = 40ms. (Measured) That means we have 8ms. per ihcPulseTimer value  (5*8ms = 40ms)
                state_machine = STATE_SEND_IHC_PULSE;
            }
            else {
                // we have a mismatch before and after prel delay
                // disqualify the pulse. Read new status and go sleep
 
                // no pulse have qualified for a correct door signal --> go back to sleep
                sleep_64ms_counter = 0; // start again with short sleep intervals
                sleep_256ms_counter = 0;
                sleep_1s_counter = 0;
                sleep_4s_counter = 0;
                //sleep_256s_counter = 0;
                
                set_watchdog_timer_128ms();
                state_machine = STATE_GO_SLEEP;
            }
            break;
            
        case STATE_SEND_IHC_PULSE: // "06" 
            if (ihcPulseTimer > 0) 
                state_machine = STATE_SEND_IHC_PULSE; // continue until ihcPulseTimer counts to zero in timer interrupt routine
            else
                state_machine = STATE_STOP_SENDING_IHC_PULSE; // now IHC pulse goes in-active
            break;
           
        case STATE_STOP_SENDING_IHC_PULSE: // "07"
            // de-activate all IHC pulse outputs
            OUTPUT_DOOR_OPEN   = !PULSE_ON;     // means it goes off
            OUTPUT_DOOR_CLOSED = !PULSE_ON;
            OUTPUT_BATTERY_LOW = !PULSE_ON;
            // pulse have now been sent - go back to sleep
            
            sleep_64ms_counter = 0; // start again with short sleep intervals
            sleep_128ms_counter = 0;
            sleep_256ms_counter = 0;
            sleep_1s_counter = 0;
            sleep_4s_counter = 0;
            //sleep_256s_counter = 0;
            set_watchdog_timer_128ms();
                        
            state_machine = STATE_GO_SLEEP;
            break;
            
        case STATE_GO_SLEEP: // "09"

            // prepare to be put to sleep
            
        #if defined(_16LF1829)          
            PWMtoggle = !PWMtoggle;
            
            // debug  to be able to see the sleep intervals
            if (PWMtoggle == 1) {
                LATAbits.LATA5 = 1;
            }
            else {
                LATAbits.LATA5 = 0;
            }
        #endif            
            
            //PSTR1CONbits.STR1A = 0; 
           
            // Important to be able to see 7Seg after sleep
//            PSTR1CONbits.STR1A = 1; 
//            CCP1CONbits.CCP1M = 0b0000;

            enable_watchdog_timer();
            enable_external_interrupt();
            // ----------------- SLEEEEP ---------------------------------
            SLEEP();                // go sleep until WDT times out or INT input wakes it up
            NOP();                  // wakes from watch-dog timer on this line (the next line after a sleep{            WDTCONbits.SWDTEN = 0;  // disable watch-dog timer
            // ------------------ WAKE UP --------------------------------
            disable_watchdog_timer();
            
            // check if we came here after af time out 
            if ((__timeout == 0) && (__powerdown == 1)) {   // nTO: if 0, a WDT time-out occurred
                // we are here as planned after a wake up
            }
            else {
                // if here and timeout flag is not "0", something must be wrong. Therefore a reset of the microcontroller to begin from start
               // RESET();
            }
            
            
            // Important to be able to see 7Seg after sleep
            //CCP1CONbits.CCP1M = 0b1100;     // 1100 = PWM mode: PxA, PxC active-high; PxB, PxD active-high
            
            // !!!! The next line is important her. If not here the setting will be forgotten after a power down/sleep. And then the PWM is not running and therefore 
            // nothing will be visible in the 7Seg Click module
            //PSTR1CONbits.STR1A = 1;     // !!!! HUSK DENNE FOR PWM out!!  PxA pin has the PWM waveform with polarity control from CCPxM<1:0>
                                          // = PxA pin has the PWM waveform with polarity control from CCPxM<1:0>
            //PSTR1CONbits.STR1A = 1; 
            
            // 337 times per day of 256 seconds equals 24 hours
            if (sleep_256s_counter >= 337) {
                // time to check voltage on battery (~one time per day)
                BatteryVoltage = getBatteryVoltage();
                               
                // BatteryVoltage is the raw voltage measurement of the 1.024 voltage ref. measurement
                // now we calculate back what the VDD is in milivolts
                millivolts = (8192 / BatteryVoltage) * 1024;
                millivolts = millivolts /8;
                
#if defined(_16LF1829) // only write to 7Seg display if we are using the 16LF1829 microcontroller 
                // to show the voltage in the 7-Seg display 3300 milivolt is divided by 100 --> which gives 33 and the comman in the display is enable to show 3.3 (volt)
                millivolts = millivolts /100;
                PWM_set(10);
                int2bcd(millivolts,1);
                __delay_ms(800);           // wait to be able to see bars in the 7Seg display
#endif  
                
                if (BatteryVoltage <= VOLTAGE_LOW) {  // this if voltage measured is lower than 2.4 volt
                    OUTPUT_BATTERY_LOW = PULSE_ON;  // turn on battery voltage signal
                    __delay_ms(IHC_PULSE_WIDTH*8);     // Timer2 takes 8ms therefore 5 * 8ms = ~40 ms
                    OUTPUT_BATTERY_LOW = !PULSE_ON;  // turn on battery voltage signal
                }
                else
                    OUTPUT_BATTERY_LOW = !PULSE_ON; // or turn off
                sleep_256s_counter = 1;     // preset counter to make another voltage measurement in another 337 x 256 sec.
            }
            else {
                //OUTPUT_BATTERY_LOW = !PULSE_ON;     // on next wake - flag is cleared again. so flag is only set one timer every time WakeUpCounter exceeds its limit
                //WakeUpCounter++;                    // IHC stores this flag and continues to send SONOS message in a period defined in the IHC module code at IHC
                NOP();
            }
            // state_door = DOOR_INPUT;                // read current door state
            state_machine = STATE_CHECK_DOOR;       // 
            break;
            
        case STATE_DOOR_CHANGED:    // "03"
             STATE_MEASURE_BATTERY: // "08"
             STATE_SLEEPING: // "10"
             default:
             break;
            
    } // switch
}       // </editor-fold>


