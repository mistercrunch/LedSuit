/***************************************

Algo:

Each new frame, PWM_prep build an array of the leds that are on. Then flips all leds to off, resets the timer

Each first timer cycle, all leds that have PMW>0 light up.

Each clock tick, we look in the list of the leds that are still on if it needs to be switched off, then modify the list accordingly

***************************************/

#include "pwm.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define PWM_FREQUENCY   0x04
#define PauseClock() TCCR0A  &= ~0b00000100; // clk/256 0b00000101 for 1024
#define ResumeClock() TCCR0A |=  0b00000100;



/***************************************
                GLOBALS
***************************************/
volatile uint8_t PWM_pins[PWM_PINS_NUMBER];
//volatile uint8_t PWM_pins_on[PWM_PINS_NUMBER];
//volatile uint8_t NbLedOn;
/***************************************
         INITIALIZE THE TIMER
***************************************/

void PWM_SwitchPins()
{
  uint8_t i;
  for (i=1; i<4;  i++)  if (PWM_pins[i]  > TCNT0) PORTA &= ~(1 << i); else PORTA |= (1 << i);
  for (i=0; i<8;  i++)
  {
    //if (PWM_pins[i+4]    > TCNT0) PORTB &= ~(1 << i); else PORTB |= (1 << i);
    if (PWM_pins[i+12]   > TCNT0) PORTC &= ~(1 << i); else PORTC |= (1 << i);
    if (PWM_pins[i+20]   > TCNT0) PORTD &= ~(1 << i); else PORTD |= (1 << i);
  }
}
void PWM_AllOff()
{
    for(uint8_t i=0;i<PWM_PINS_NUMBER;i++)
        PWM_pins[i]=0;
    PWM_SwitchPins();
}
void PWM_Reset()
{
    PauseClock();

    TCNT0 = 0;

    PWM_SwitchPins();
    //FirstTick();

    OCR0A=1;
    // reset timer value


    // timer 2 on
    ResumeClock();
}

void PWM_init(void){

  for (uint8_t i=0; i<PWM_PINS_NUMBER; i++)PWM_pins[i]=0;

  PauseClock(); // timer off (turned on by PWM_set())
  TIMSK0    |= 0b00000011;           // output compare match interrupt enable // overflow interrupt enable
  //TIFR0     |= 0b00000011;
  OCR0A = 1;
  TCNT0 = 0;
  ResumeClock();
}

/***************************************
     CHANGE THE DUTY CYCLE OF A PIN
***************************************/
void PWM_set(uint8_t PWM_pin, uint8_t PWM_duty_cycle){
  // set the new duty cycle in the pin array
  PWM_pins[PWM_pin] =  PWM_duty_cycle;
}





/***************************************
               INTERRUPTS
***************************************/

SIGNAL(TIMER0_COMPA_vect){

    PauseClock();

    //MAX:
    PWM_SwitchPins();
    //Tick();
    //PORTC &= ~(1 << i)
    //Increment to stop at next clock tick
    OCR0A+=8;
    ResumeClock();

}

SIGNAL(TIMER0_OVF_vect ){
    //PulseOneBlue();//Debug
    PWM_Reset();

}
