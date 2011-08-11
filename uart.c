#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/crc16.h>
#define U8 uint8_t

#define HALF_BIT_LENGHT 1500
#define BIT_LENGHT 3000

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


typedef struct
{
  volatile uint8_t * PORTX;
  volatile uint8_t * PINX;
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
    //PCMSK0  |=  PB_PINS;

    DDRA    &= ~PA_PINS;
    PORTA   |=  PA_PINS;
    //PCMSK3  |=  PA_PINS;

    //PCICR   |= 0b00001001;
}

void UART_AllOut()
{
    //PCICR   &= 0b11110110;

    //PCMSK0  &= ~PB_PINS;
    DDRB    |= PB_PINS;
    PORTB   |= PB_PINS;

    //PCMSK3  &= ~PA_PINS;
    DDRA    |= PA_PINS;
    PORTA   |= PA_PINS;
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
    UART_Push0(thePin);
    UART_Push1(thePin);

    U8 i;
    for(i=0; i<8; i++)
    {
        if (byte >> i & 1)
            UART_Push0(thePin);
        else
            UART_Push1(thePin);
    }
    UART_Push1(thePin);
    UART_Push1(thePin);
}

uint8_t UART_ReadByte(UartPin * thePin)
{
    volatile U8 msg=0;
    while((*thePin->PINX & (*thePin).BV) !=0 ){}
    //i=0;
    while((*thePin->PINX & (*thePin).BV) ==0 ){}//Waiting for start pulse end

    _delay_loop_2 (BIT_LENGHT);
    _delay_loop_2 (HALF_BIT_LENGHT);//Delaying half a bit to sample in the middle of the bit lenght

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


