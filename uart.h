
#define U8 uint8_t
typedef struct
{
  volatile uint8_t * PORTX;
  volatile uint8_t * PINX;
  uint8_t BV;
} UartPin;


void        UART_SendByte(U8 byte, UartPin * thePin);
void        UART_AllEars();
void        UART_AllOut();
//void        UART_Push0(UartPin * thePin);
//void        UART_Push1(UartPin * thePin);
uint8_t     UART_ReadByte(UartPin * thePin);
uint8_t     UART_CheckCRC(uint8_t MyArray[]);
