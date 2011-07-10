/***************************************
               PINKIE
Timer driven PWM on up to 16 output pins

 Vincent Leclerc            03/13/2006
***************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define PWM_PINS_NUMBER 28    // for the array

/***************************************
           DEFINES AND GLOBALS
***************************************/


volatile uint8_t PWM_pins[PWM_PINS_NUMBER];
//volatile uint8_t PWM_pins_on[PWM_PINS_NUMBER];
//volatile uint8_t NbPinOn;
//extern uint8_t PWM_pin_numbers[];
//extern uint8_t PWM_sorted_pins[16];


//void PWM_prep(void);
//void RebuildListDiff0(void);

/***************************************
         FUNCTIONS DECLARATIONS
***************************************/
void PWM_init(void);
void PWM_SwitchPins(void);
void PWM_set(uint8_t PWM_pin, uint8_t PWM_duty_cycle);
void PWM_Reset(void);
void PWM_AllOff();
