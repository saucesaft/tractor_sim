#ifndef UART_H_
#define UART_H_

#include <stdio.h>

int _write(int file, char *ptr, int len);

// config registers
#define USARTDIV        0x22C				// 115200 baud rate

void USER_USART1_Init( void );
void USER_USART1_Transmit( uint8_t *pData, uint16_t size );
uint8_t USER_USART1_Read_8bit();
void USER_USART1_Send_8bit( uint8_t );
void USER_UART_Send_Message( uint8_t *msg, uint16_t size );

#endif /* UART_H_ */