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
 * File:                statemachine.h
 * Author:              Henrik J. Koch
 * Comments:            state machine header file
 * Revision history:     
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_STATEMACHINE_H
#define	XC_STATEMACHINE_H
#include <xc.h> // include processor files - each processor file is guarded.  


#define IHC_PULSE_WIDTH         5      // corresponds to ~40ms. (Measured) That means we have 8ms. per ihcPulseTimer value  (5*8ms = 40ms)
#define PREL_DELAY              1      // corresponds to ~8ms (1*8ms)
#define PULSE_ON                1      // "1" when pulse out is going high, "0" if pulse out is going low
#define PULSE_OFF               0
#define VOLTAGE_LOW             3000   // 1.0 V per AA battery cell 

#define LED_BLINK_PULSE         80    // 80 ms
#define LED_BLINK_PAUSE         320   // 120 ms
#define LED_BLINK_DIGIT_DELAY   800   // 400 ms

enum door_input_change {
                           change_door_opended = 0,
                           change_door_closed = 1
};

enum STATE_DOOR_STATES {
                           STATE_DOOR_CLOSED = 0,   // when using HALL sensor 
                           STATE_DOOR_OPEN = 1,     // when using HALL sensor
                           STATE_DOOR_UNKNOWN = 2        
};
enum STATE_MACHINE1_STATES {
                           STATE_JUST_AWAKED = 0,
                           STATE_NEW_DOOR_STATE = 1,
                           STATE_CHECK_DOOR = 2,
                           STATE_DOOR_CHANGED =3 ,
                           STATE_DOOR_CHANGED_WAIT_PREL = 4,
                           STATE_DOOR_CHECK_AFTER_PREL = 5,
                           STATE_SEND_IHC_PULSE = 6,                           
                           STATE_STOP_SENDING_IHC_PULSE = 7,
                           STATE_MEASURE_BATTERY = 8,                           
                           STATE_GO_SLEEP = 9,
                           STATE_SLEEPING = 10,
                           STATE_INITIALIZE = 11,
                           STATE_SELFTEST = 12,
                           STATE_INITIAL_TEST = 13
};
enum STATE_OUTPUT_PULSE_STATES {
                           STATE_OUTPUT_PULSE_DOOR_OPEN = 0,
                           STATE_OUTPUT_PULSE_DOOR_CLOSED = 1,
                           STATE_OUTPUT_PULSE_BATTERY_LOW = 2,
                           STATE_OUTPUT_PULSE_ERRROR = 3,
                           STATE_OUTPUT_TEST = 4
};

enum STATE_SELFTEST_STATES {
                           STATE_SELFTEST_OUTPUT_PULSE_DOOR_OPEN = 0,
                           STATE_SELFTEST_OUTPUT_PULSE_DOOR_CLOSED =1,
                           STATE_SELFTEST_OUTPUT_PULSE_BATTERY_LOW = 2
};
 

extern void checkState1(void); 



// Timing counters
extern volatile int prelCounter;                // timer for prel time
extern volatile int ihcPulseTimer;              // timer for IHC output pulse
extern volatile int WakeUpCounter;              // timer how  many times it needs to wake up before a voltage measurement is done
extern volatile int BatteryVoltage;             // keeps til battery voltage measured
extern volatile unsigned int millivolts;                 // keeps batteryvoltage in milivolts

extern char DoorStateBeforePrel;     // check before and after prel timer
extern char DoorStateAfterPrel;      // -||-
extern __bit NewStateChangeDetected;  // flag to show new door interrupt has been seen
extern __bit NewDoorPolarity;           // 

#if defined(_16LF1829) 
extern __bit PWMtoggle;
#endif

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */



#endif	/* XC_STATEMACHINE_H */
