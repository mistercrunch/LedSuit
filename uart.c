#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/crc16.h>
#define U8 uint8_t

#define HALF_BIT_LENGHT 1500
#define BIT_LENGHT 3000
#define ONE_AND_HALF_BIT_LENGHT BIT_LENGHT + HALF_BIT_LENGHT

#define PB_PINS 0b10000110;
#define PA_PINS 0b00000001;
//#define Push0() *PORTX &= ~_BV(PinNum); _delay_loop_2 (BIT_LENGHT);
//#define Push1() *PORTX |= _BV(PinNum); _delay_loop_2 (BIT_LENGHT);

//CONN1 (PB1, VCC, GND, PB7)
//CONN2 (PA0, VCC, GND, PB2)
//     <  |PB1 PA0| < PB1
//        |VCC VCC|   VCC
//        |GND GND|
//     >  |PB7 PB2| > PB7

//PB1 O5
//PB7 N1
//PA0 S3
//PB2 O6

//UartPin             UartPins[4];

typedef struct
{
  volatile uint8_t * PORTX;
  volatile uint8_t * PINX;
  volatile uint8_t * DDRX;
  uint8_t BV;
} UartPin;

void UART_Init()
{
    PCICR   |= 0b00001001;//Init interrupt on A and B
    PCMSK0  |=  PB_PINS;
    PCMSK3  |=  PA_PINS;
}
void UART_AllEars()
{
    DDRB    &= ~PB_PINS;
    PORTB   |=  PB_PINS;

    DDRA    &= ~PA_PINS;
    PORTA   |=  PA_PINS;
}

void UART_AllOut()
{
    DDRB    |= PB_PINS;
    PORTB   |= PB_PINS;

    DDRA    |= PA_PINS;
    PORTA   |= PA_PINS;
}

void UART_PinListen(UartPin * thePin)
{
    *thePin->DDRX   &= ~(*thePin).BV;
    *thePin->PORTX  |= (*thePin).BV;
}

void UART_PinTalk(UartPin * thePin)
{
    *thePin->DDRX   |= (*thePin).BV;
    *thePin->PORTX  |= (*thePin).BV;
}

void UART_Push0(UartPin * thePin)
{
    *thePin->PORTX &= ~(*thePin).BV;
    _delay_loop_2 (BIT_LENGHT);
}
void UART_Push1(UartPin * thePin)
{
    *thePin->PORTX |= (*thePin).BV;
    _delay_loop_2 (BIT_LENGHT);
}



void UART_SendByte(U8 byte, UartPin * thePin)
{
    U8 i;
    UART_PinTalk(thePin);   //Flipping the Pin to write, allowing to call this function at any time without explicitly flipping the pine

    UART_Push1(thePin);     //Line is not active at 1, this introduces a delay allowing flipping the line direction.
    UART_Push1(thePin);

    //Two line bellow represent the start bits 0 is up, 1 is down
    UART_Push0(thePin);
    UART_Push1(thePin);

    for(i=0; i<8; i++)
    {
        if (byte >> i & 1)
            UART_Push0(thePin);
        else
            UART_Push1(thePin);
    }
    UART_Push1(thePin);

}
uint8_t UART_CheckPin(UartPin * thePin){
    if ((*thePin->PINX & (*thePin).BV) !=0)
        return 1;
    else
        return 0;
}


void WaitWhileStatus(UartPin * thePin, uint8_t status){
    uint16_t i =0;
    uint16_t timeout = 65500;
    uint8_t keepLooping = 1;

    while (keepLooping==1)
    {
        if(UART_CheckPin(thePin)!=status)
            keepLooping=0;
        _delay_loop_2 (1);
        i++;
        if(i>timeout)keepLooping=0;
    }
}



uint8_t UART_ReadByte(UartPin * thePin)
{
    UART_PinListen(thePin); //Flipping the Pin to listen, allowing to call this function at any time without explicitly flipping the pine
    volatile U8 msg=0;


    WaitWhileStatus(thePin, 1);//Waiting for start bit to start, most liquely skipped if line down
    WaitWhileStatus(thePin, 0);//Waiting for the end of the start bit

    _delay_loop_2 (ONE_AND_HALF_BIT_LENGHT);

    uint8_t i =0;
    for(i=0; i<8; i++)
    {
        if((*thePin->PINX & (*thePin).BV) == 0)
            msg|= 1 << i;
        _delay_loop_2 (BIT_LENGHT);
    }
    return msg;
}

uint8_t UART_CheckCRC(uint8_t MyArray[] )
{
        uint8_t crc = 0;
        uint8_t i;
        for (i = 0; i < sizeof MyArray / sizeof MyArray[0]; i++)
            crc = _crc_ibutton_update(crc, MyArray[i]);
        return crc; // must be 0
}


