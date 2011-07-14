#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/crc16.h>
#define U8 uint8_t

#define HALF_BIT_LENGHT 1000
#define BIT_LENGHT 2000

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

void UART_AllEars()
{
    DDRB    &= ~PB_PINS;
    PORTB   |=  PB_PINS;
    PCMSK0  |=  PB_PINS;

    DDRA    &= ~PA_PINS;
    PORTA   |=  PA_PINS;
    PCMSK3  |=  PA_PINS;

    PCICR   |= 0b00001001;
}

void UART_AllOut()
{
    PCICR   &= 0b11110110;

    PCMSK0  &= ~PB_PINS;
    DDRB    |= PB_PINS;
    PORTB   |= PB_PINS;

    PCMSK3  &= ~PA_PINS;
    DDRA    |= PA_PINS;
    PORTA   |= PA_PINS;
}

void UART_Push0()
{
    PORTA &= ~PA_PINS;
    PORTB &= ~PB_PINS;
    _delay_loop_2 (BIT_LENGHT);
}
void UART_Push1()
{
    PORTA |= PA_PINS;
    PORTB |= PB_PINS;
    _delay_loop_2 (BIT_LENGHT);
}
void UART_SendByte(U8 byte)
{
    UART_Push0();
    UART_Push1();

    U8 i;
    for(i=0; i<8; i++)
    {
        if (byte >> i & 1)
            UART_Push0();
        else
            UART_Push1();
    }
    UART_Push1();
    UART_Push1();
}
U8 UART_ReadByte(volatile uint8_t *PINX, uint8_t PinNum)
{
    volatile U8 msg=0;
    //int ii=0;

    //while((*PINX & _BV(PinNum)) == 0 && ii<1000){ii++;_delay_loop_2 (1);}//Waiting for start pulse start
    //i=0;
    while((*PINX & _BV(PinNum)) !=0){}
    while((*PINX & _BV(PinNum)) ==0){}//Waiting for start pulse end


    _delay_loop_2 (BIT_LENGHT);
    _delay_loop_2 (HALF_BIT_LENGHT);//Delaying half a bit to sample in the middle of the bit lenght

    U8 i;
    for(i=0; i<8; i++)
    {
        if((*PINX & _BV(PinNum)) == 0)
            msg|= 1 << i;
        _delay_loop_2 (BIT_LENGHT);
    }
    return msg;
}

uint8_t UART_CheckCRC(uint8_t MyArray[])
{
        uint8_t crc = 0;
        uint8_t i;
        for (i = 0; i < sizeof MyArray ; i++)
            crc = _crc_ibutton_update(crc, MyArray[i]);

        return crc; // must be 0
}


