#include <avr/io.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/delay_basic.h>
#include <avr/sleep.h>
#include "pwm.h"
#include "uart.h"
#define U8 uint8_t
#define NBLED 6
#define NB_EFFECT_PARAMS 7
#define START_TRANS_BYTE 0b10101010

#define STATE_NULL              0
#define STATE_BUTTON_PUSHED     11
#define STATE_MSG_RECEIVED      12
#define STATE_BAD_MSG_RECEIVED  13
#define STATE_OLD_MSG_RECEIVED  14
#define STATE_WAS_INTERUPTED    15


//#define RAND_MAX 255

////////////////////////////////////////////////////////////
//Strucstses!
////////////////////////////////////////////////////////////
uint8_t aCurrentEffect[NB_EFFECT_PARAMS];
int     UartDelay=0;
uint8_t SendDelay=0;
uint8_t Message=STATE_NULL;
/*
typedef struct
{
    uint8_t MsgNumber;
	uint8_t Hue;
	uint8_t Brightness;
	uint8_t ColorRange;
	uint8_t CycleDuration;
	uint8_t Mode;
	uint8_t Delay; //Delay before sending the msg
} Effect;
*/
/*
c1   __________
    /          \
   /            \
  /              \
 /                \____________
c2

[tA][tB      ][tC][tD          ]

*/

typedef struct
{
  U8 R,G,B;
} Color;

typedef struct
{
    Color c;
    Color BaseColor;
    uint8_t CyclePosition;
    uint8_t CycleDuration;
}LED;

////////////////////////////////////////////////////////////
//Gloabal Vars
////////////////////////////////////////////////////////////

//volatile uint8_t    LatestUartMessage = 255;
volatile uint8_t    State= STATE_NULL;
//Effect              CurrentEffect;
LED                 LEDs[NBLED];
Color cBlack;
uint8_t PercBetween(uint8_t v1, uint8_t v2, uint8_t nom, uint8_t denom)
{
    v2 += (((int16_t)v1 - (int16_t)v2) * (int16_t)nom) / (uint16_t)denom;
    return v2;


}
void ColorBetween(Color *c, Color *c1, Color *c2, uint8_t nom, uint8_t denom)
{
    (*c).R = PercBetween((*c1).R, (*c2).R, nom, denom);
    (*c).G = PercBetween((*c1).G, (*c2).G, nom, denom);
    (*c).B = PercBetween((*c1).B, (*c2).B, nom, denom);
}

void RandomEffect()
{
    //Modes
    //Mode 1 = flash
    //Mode 2 = fade in / fade out
    //Mode 3 = Command, set yourself to random pattern


    aCurrentEffect[5]               = (rand()%2)+1;            //Mode
    aCurrentEffect[0]++;                            //MsgNumber
	aCurrentEffect[1]               = rand();       //Hue
	aCurrentEffect[2]               = 127;          //Brightness
	aCurrentEffect[3]               = rand()%100;   //ColorRange
	aCurrentEffect[4]               = 2+rand()%100;  //CycleDuration

	if(aCurrentEffect[5]==1)
        aCurrentEffect[4]               = 2+rand()%10;       //Delay
	else if (aCurrentEffect[5]==2)
        aCurrentEffect[4]               = 30+rand()%50;       //Delay

//aCurrentEffect[4]  =100;
    aCurrentEffect[6]=(rand()%30) +2;
//aCurrentEffect[6]=100;


	SendDelay=aCurrentEffect[6];

}

void SendEffect()
{
    //This is to indicate that a message is coming
    cli();
    UART_AllOut();


    //UART_Push0();
    //UART_Push1();

    UART_SendByte(START_TRANS_BYTE);

    for(uint8_t i=0; i<NB_EFFECT_PARAMS;i++)
        UART_SendByte(aCurrentEffect[i]);

    UART_SendByte(UART_CheckCRC(aCurrentEffect));

    //UartDelay=(int)aCurrentEffect[6]+((int)aCurrentEffect[6]/2);
    UartDelay=((int)aCurrentEffect[6]) * 3;
    sei();

}

void ReceiveEffect(volatile uint8_t *PINX, uint8_t PinNum)
{

    uint8_t tmpEffect[NB_EFFECT_PARAMS];

    for(uint8_t i=0; i<NB_EFFECT_PARAMS;i++)
        tmpEffect[i] = UART_ReadByte(PINX, PinNum);

    uint8_t errorFlag = 0;

    uint8_t crc = UART_ReadByte(PINX, PinNum);
    if (crc != UART_CheckCRC(tmpEffect)) errorFlag=1;


    if(errorFlag==0)
    {
        if(aCurrentEffect[0]!=tmpEffect[0])
        {
            for(uint8_t i=0; i<NB_EFFECT_PARAMS;i++)
                aCurrentEffect[i]=tmpEffect[i];

            State=STATE_MSG_RECEIVED;
            SendDelay=aCurrentEffect[6];
        }
        //else Message=STATE_OLD_MSG_RECEIVED;
    }
    //else Message=STATE_BAD_MSG_RECEIVED;


}

void SetHue(Color *_c, uint8_t _Hue)
{
    uint8_t tmp=_Hue%85;
    if(_Hue <= 84)
    {
        (*_c).R =84 - tmp;
        (*_c).G =tmp;
        (*_c).B = 0;
    }
    else if(_Hue <= 170)
    {
        (*_c).R = 0;
        (*_c).G =84 - tmp;
        (*_c).B =tmp;
    }
    else
    {
        tmp = _Hue-171;
        (*_c).B =85 - tmp;
        (*_c).R =tmp;
        (*_c).G = 0;
    }
}
Color CloneColor(Color *_c)
{
    Color c;
    c.R = (*_c).R;
    c.G = (*_c).G;
    c.B = (*_c).B;
    return c;
}

void MatchColor(Color *Source, Color *Destination)
{
    (*Destination).R = (*Source).R;
    (*Destination).G = (*Source).G;
    (*Destination).B = (*Source).B;
}

void DimColor(Color *_c, uint8_t v)
{
        (*_c).R = ((*_c).R * v) /255;
        (*_c).G = ((*_c).G * v) /255;
        (*_c).B = ((*_c).B * v) /255;
}

uint8_t Clip(int16_t v)
//Used to make sure the result of an equation is between 0-255
{
    if(v > 255)     return 255;
    else if (v <0)  return 0;
    else            return ((uint8_t)v);
}

void SetRGB(U8 LedNum, U8 _R, U8 _G, U8 _B)
{
    LEDs[LedNum].c.R = _R;
    LEDs[LedNum].c.G = _G;
    LEDs[LedNum].c.B = _B;
}

void SetAllRGB(U8 R, U8 G, U8 B)
{
    for(U8 i=0;i<NBLED;i++)
        SetRGB(i,R,G,B);
}
void AllBlack()
{
    SetAllRGB(0,0,0);
}


void TransferToPWM()
{
//    InitPWM();
    PWM_pins[22] = LEDs[0].c.R;//PD2
    PWM_pins[21] = LEDs[0].c.G;//PD1
    PWM_pins[20] = LEDs[0].c.B;//PD0

    PWM_pins[3]  = LEDs[1].c.R;//PA3
    PWM_pins[17] = LEDs[1].c.G;//PC5
    PWM_pins[16] = LEDs[1].c.B;//PC4

    PWM_pins[15] = LEDs[2].c.R;//PC3
    PWM_pins[14] = LEDs[2].c.G;//PC2
    PWM_pins[13] = LEDs[2].c.B;//PC1

    PWM_pins[19] = LEDs[3].c.R;//PC7
    PWM_pins[1]  = LEDs[3].c.G;//PA1
    PWM_pins[12] = LEDs[3].c.B;//PC0

    PWM_pins[25] = LEDs[4].c.R;//PD5
    PWM_pins[26] = LEDs[4].c.G;//PD6
    PWM_pins[27] = LEDs[4].c.B;//PD7

    PWM_pins[23] = LEDs[5].c.R;//PD3
    PWM_pins[24] = LEDs[5].c.G;//PD4
    PWM_pins[2]  = LEDs[5].c.B;//PA2

    //PWM_prep();
}



void Msg()
{
    if(Message!=STATE_NULL)
    {
        if (Message==STATE_BAD_MSG_RECEIVED)
        {
            SetAllRGB(255,0,0);
        }
        else if (Message==STATE_OLD_MSG_RECEIVED)
        {
            SetAllRGB(0,0,255);
        }
        else if (Message==STATE_BUTTON_PUSHED)
        {
            SetAllRGB(0,255,0);
        }
        else if (Message==STATE_WAS_INTERUPTED)
        {
            SetAllRGB(255,255,255);
        }
        else if(Message<6)
        {
            SetAllRGB(0,0,0);
            SetRGB(State,0,255,0);
        }

        TransferToPWM();
        _delay_ms(500);
        Message=STATE_NULL;
    }
}

//////////////////////////////////////////////////////
//Core
//////////////////////////////////////////////////////
void setup()
{
    //Enabling interupts on reading pins PA0 & PB7

    UART_AllEars();

        //UART_ENABLE_LISTEN();


    DDRA    |=0b00001110;
    DDRD    |=0b11111111;
    DDRC    |=0b11111111;

//POWER REDUCTION
    PRR |=0b10001101;
//Turning off the watchdog
    WDTCSR&=0b10110111;
    PWM_init();

}
int main(void)
{
    setup();
    RandomEffect();
    sei();

    Message=1;

    U8 i=0;

    while(1)
    {
        Msg();

        if(UartDelay>0)
            UartDelay--;
        else UART_AllEars();

        if(State==STATE_BUTTON_PUSHED)
        {
            //Someone Pushed the button
            srand(TCNT0);
            RandomEffect();
            //for(int i=0;i<10;i++)

            State=STATE_MSG_RECEIVED;
        }
        else if(State==STATE_MSG_RECEIVED)
        {
            if(SendDelay==0)
            {
                SendEffect();
                State=STATE_NULL;
            }
            else
                SendDelay--;
        }

        for(i=0;i<NBLED;i++)
        {
            if(aCurrentEffect[5]==1)
            {
                if(LEDs[i].CyclePosition >= (LEDs[i].CycleDuration/4))
                    SetRGB(i,0,0,0);
                else

                    MatchColor(&LEDs[i].BaseColor, &LEDs[i].c);
            }
            else if(aCurrentEffect[5]==2)
            {
                uint8_t HalfCycleDuration = LEDs[i].CycleDuration/2;
                if(LEDs[i].CyclePosition<LEDs[i].CycleDuration/2)
                    ColorBetween(&LEDs[i].c, &LEDs[i].BaseColor, &cBlack, LEDs[i].CyclePosition, HalfCycleDuration);
                else
                    ColorBetween(&LEDs[i].c, &LEDs[i].BaseColor, &cBlack, HalfCycleDuration - (LEDs[i].CyclePosition- HalfCycleDuration), HalfCycleDuration );

            }
            LEDs[i].CyclePosition++;
            if(LEDs[i].CyclePosition >= LEDs[i].CycleDuration)
            {
                    LEDs[i].CyclePosition = 0;
                    LEDs[i].CycleDuration = aCurrentEffect[4]/*CycleDuration*/+ (rand()%5);
                    SetHue(&LEDs[i].BaseColor, aCurrentEffect[1]/*Hue*/+ (rand()%aCurrentEffect[3]/*ColorRange*/));
            }
        }
        TransferToPWM();
        _delay_ms(5);
    }
}

//////////////////////////////////////////////////////
//Interupts
//////////////////////////////////////////////////////
void Test()
{
    PWM_AllOff();
    PORTD=0b00000000;
    _delay_ms(250);
    PWM_AllOff();
}

void TreatInterupt(volatile uint8_t *PINX, uint8_t PinNum)
{


    cli();
    PWM_AllOff();

    //while((PINB & _BV(7)) ==0){}

    uint8_t UartByte = UART_ReadByte(PINX, PinNum);

    if (UartByte==START_TRANS_BYTE)
    {
        ReceiveEffect(PINX, PinNum);
    }
    else if(UartByte==0)
    {
        if(PinNum==7)
        {
            State=STATE_BUTTON_PUSHED;//Someone hit the button
        }
    }
    //else Message=STATE_BAD_MSG_RECEIVED;

    //Message=STATE_WAS_INTERUPTED;

    sei();
}

ISR(PCINT0_vect)
{
    if((PINB & _BV(7))== 0)
         TreatInterupt(&PINB, 7);
    else if((PINB & _BV(1))==0)
        TreatInterupt(&PINB, 1);
    else if((PINB & _BV(2))==0)
        TreatInterupt(&PINB, 2);
}

ISR(PCINT3_vect)
{

    if((PINA & _BV(0))==0)
        TreatInterupt(&PINA, 0);
}
