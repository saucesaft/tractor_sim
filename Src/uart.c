#include <stdint.h>
#include "main.h"
#include "uart.h"

void USER_USART1_Init( void ){
	USART1->CR1	|=	 USART_CR1_UE;//	Step 1 Usart enabled
	USART1->CR1	&=	~USART_CR1_M;//		Step 2 8 Data bits
	USART1->CR2	&=	~USART_CR2_STOP;//	Step 3 1 Stop bit
	USART1->BRR	=	 USARTDIV;//		Step 5 Desired baud rate
	USART1->CR1	|= 	 USART_CR1_TE;//	Step 6 Transmitter enabled
	USART1->CR1	|= 	 USART_CR1_RE;//	Step 7 Receiver enabled
}

void USER_USART1_Send_8bit( uint8_t Data ){
	while(!( USART1->SR & USART_SR_TXE ));//	wait until next data can be written
	USART1->DR = Data;//				Step 7 Data to send
}

uint8_t USER_USART1_Read_8bit() {
	while(!( USART1->SR & USART_SR_RNXE ));//	wait until next data can be read
	return USART1->DR;
}

int _write(int file, char *ptr, int len) {
	for (int data_idx = 0; data_idx < len; data_idx++) {
		while ( !(USART1->SR & USART_SR_TXE) );
		USART1->DR = *ptr++;
	}
	return len;
}

void USER_UART_Send_Message( uint8_t *msg, uint16_t size ){
	for( int i = 0; i < size; i++ ){
		USER_USART1_Send_8bit( *msg++ );
	}
}
