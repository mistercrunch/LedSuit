
#define U8 uint8_t

void        UART_SendByte(U8 byte);
void        UART_AllEars();
void        UART_AllOut();
void        UART_Push0();
void        UART_Push1();
uint8_t     UART_ReadByte(volatile uint8_t *PORTX, uint8_t PinNum);
uint16_t    UART_CheckCRC(uint8_t MyArray[]);
